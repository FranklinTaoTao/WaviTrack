/*
 * Multi-node chained SS-TWR for DW3000 + ESP32C6
 * + Per-link CIR capture (lower idx -> higher idx)
 * + 60 Hz framing (node 0 poll every 16667 us)
 * + AsyncUDP streaming (one packet per frame per node)
 * + IMU (BNO085) appended at end of each UDP packet
 *
 * NOTE:
 * - UWB frame layout/length logic is preserved.
 * - SPI / DW3000 calls are kept in the same style.
 */

#include "dw3000.h"
#include <Preferences.h>

// -------------------- WiFi / UDP --------------------
#include <WiFi.h>
#include <AsyncUDP.h>
#include <esp_timer.h>

// -------------------- IMU (BNO085) --------------------
#include "imu_bno08x.h"

#define ENABLE_IMU       1
#define I2C_CLOCK_HZ     200000
#define IMU_I2C_ADDR     0x4A
#define IMU_IRQ_PIN      16
#define IMU_REPORT_INTERVAL_MS 16

// If 1: require IMU present (block in setup until detected)
// If 0: allow running without IMU (zeros + error flag)
#define REQUIRE_IMU 0

#define WIFI_SSID   "YOUR_WIFI_SSID"
#define WIFI_PSK    "YOUR_WIFI_PASSWORD"

static const IPAddress UDP_SERVER_IP(192, 168, 1, 100);
#define UDP_PORT_BASE 20000

// If 1: block in setup until WiFi connected, and in loop do nothing if disconnected.
// If 0: allow UWB to run without WiFi (UDP sends are skipped if disconnected).
#define REQUIRE_WIFI_CONNECTED 1

// Serial logs are expensive at 60 Hz; keep disabled by default.
#define ENABLE_SERIAL_LOG 0

#if ENABLE_SERIAL_LOG
  #define LOGI(...)   Serial.printf(__VA_ARGS__)
  #define LOGLN(...)  do { Serial.printf(__VA_ARGS__); Serial.println(); } while (0)
#else
  #define LOGI(...)   do {} while (0)
  #define LOGLN(...)  do {} while (0)
#endif

static volatile bool g_wifi_connected = false;
static AsyncUDP udp;
static uint16_t g_udp_local_port  = 0;
static uint16_t g_udp_remote_port = 0;
static volatile bool g_udp_ready = false;

void WiFiEvent(WiFiEvent_t event);

// -------------------- App --------------------
#define APP_NAME "Multi-node SS TWR (ESP32 IRQ + AsyncUDP + CIR + BNO085)"

// -------------------- UWB / hardware config --------------------
#define ANT_DELAY 16380

const uint8_t PIN_IRQ = D0; // irq pin
const uint8_t PIN_SS  = D7; // spi select pin
const uint8_t PIN_RST = D2; // reset pin
const uint8_t PIN_LED = D3; // debug LED

// Max nodes in the network (compile-time)
#define MAX_NODES  6

// Number of nodes in THIS deployment (can also be read from NVS)
uint8_t g_num_nodes = 6;   // default; may be overridden from prefs

// Slot timing (in UUS)
#define RESP_GAP_UUS           1000     // gap after previous frame to next TX

// 60 Hz framing + ranging budget
#define FRAME_PERIOD_US        16667   // 60 Hz
#define RANGING_TIMEOUT_US     10000   // ranging must finish within 10 ms

// Frame field indices
#define ALL_MSG_SN_IDX         2
#define APP_CODE_IDX           9
#define MSG_TYPE_IDX           10
#define ROUND_ID_L_IDX         11
#define ROUND_ID_H_IDX         12
#define NODE_ID_IDX            13
#define PAYLOAD_IDX            14
#define ALL_MSG_COMMON_LEN     14  // header size before payload

// Message types
#define MSG_TYPE_POLL          0x01
#define MSG_TYPE_NODE_TX       0x02

// Timestamp size (truncated 32-bit)
#define TS_SIZE                4

// Buffers
#define RX_BUF_LEN             64
#define TX_BUF_LEN             64

static uint8_t rx_buffer[RX_BUF_LEN];
static uint8_t tx_buffer[TX_BUF_LEN];

// Base header template
static uint8_t uwb_header[] = {
  0x41, 0x88, 0x00,     // frame control + seq (filled later)
  0xCA, 0xDE,           // PAN ID
  'W','A','V','E',      // dest/src (just a tag)
  0xE0,                 // app code
  0x00,                 // msg_type
  0x00,                 // round_id low byte
  0x00,                 // round_id high byte
  0x00                  // node_id
};

static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. TX only. */
    DWT_PAC8,        /* PAC. RX only. */
    9,               /* TX preamble code. */
    9,               /* RX preamble code. */
    2,               /* SFD: 2 = non-std 16-sym. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_DTA, /* PHY header rate. */
    (128 + 1 + 16 - 8), /* SFD timeout. RX only. */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

extern dwt_txconfig_t txconfig_options;

// -------------------- CIR capture + compression --------------------
// Default: read 96 samples from 725..820 inclusive.
// NOTE: dwt_readaccdata includes a dummy byte at the beginning (handled below).

#define CIR_SAMPLES 96
#define CIR_OFFSET  725

static uint8_t  cir_buf[1 + CIR_SAMPLES * 6]; // dummy + 6 bytes per sample
static uint16_t cir_mag16[MAX_NODES][CIR_SAMPLES]; // this node stores CIR from lower senders
static uint16_t cir_rx_pacc[MAX_NODES];           // optional, per sender
static uint16_t g_rx_pacc = 0;                    // last capture
static uint8_t  g_cir_valid_mask = 0;             // bit i => CIR from i valid this round

static inline int32_t sign_extend_24(uint32_t x) {
  if (x & 0x00800000UL) x |= 0xFF000000UL;
  return (int32_t)x;
}

static inline uint32_t uabs32(int32_t v) {
  return (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
}

void compress_cir_to_mag16(const uint8_t *buf, uint16_t *out16, size_t nSamples)
{
  const uint8_t *p = buf + 1; // skip dummy byte
  for (size_t k = 0; k < nSamples; k++) {
    uint32_t r24u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
    int32_t  r24  = sign_extend_24(r24u);
    uint32_t i24u = (uint32_t)p[3] | ((uint32_t)p[4] << 8) | ((uint32_t)p[5] << 16);
    int32_t  i24  = sign_extend_24(i24u);
    p += 6;

    uint32_t a = uabs32(r24);
    uint32_t b = uabs32(i24);
    uint32_t mx = (a > b) ? a : b;
    uint32_t mn = (a > b) ? b : a;

    uint32_t mag = mx + (mn >> 2);
    if (mag > 0x3FFFFUL) mag = 0x3FFFFUL;

    uint16_t c16 = (uint16_t)(mag & 0xFFFFU);
    out16[k] = c16;
  }
}

static void capture_cir_from_sender(uint8_t sender_idx)
{
  uint32_t rxpacc_reg = dwt_read32bitoffsetreg(0x00, 0x4C);
  uint16_t rxpacc = (uint16_t)((rxpacc_reg >> 20) & 0x0FFF);
  if (rxpacc >= 8) rxpacc = (uint16_t)(rxpacc - 8);
  g_rx_pacc = rxpacc;
  if (sender_idx < MAX_NODES) {
    cir_rx_pacc[sender_idx] = rxpacc;
  }

  dwt_readaccdata(cir_buf, sizeof(cir_buf), CIR_OFFSET);
  compress_cir_to_mag16(cir_buf, cir_mag16[sender_idx], CIR_SAMPLES);

  g_cir_valid_mask |= (uint8_t)(1U << sender_idx);
}

// -------------------- Global ranging state --------------------
Preferences prefs;

// Node identity
uint8_t g_node_idx = 0;   // 0 = A, 1 = B, ...
uint16_t g_round_id = 0;  // round/superframe id (16-bit in UWB/UDP)
uint8_t frame_seq_nb = 0;

// Local UDP frame counter (monotonic per node; increments on each new poll tx/rx)
static uint32_t g_frame_no = 0;

// Timestamps: ts_rx[src][dst], ts_tx[node]
static uint32_t ts_rx[MAX_NODES][MAX_NODES];
static uint32_t ts_tx[MAX_NODES];

// Distances
static double dist_table[MAX_NODES][MAX_NODES];
static uint8_t g_dist_valid_mask = 0; // bit i => distance to i is valid this round (as computed on this node)

// IRQ flag
static volatile bool dw_irq_flag = false;
void IRAM_ATTR dw3000_gpio_isr(void*) { dw_irq_flag = true; }

// Round state
enum {
  ROUND_IDLE,
  ROUND_WAITING_FRAMES
};
uint8_t  g_round_state = ROUND_IDLE;
uint32_t g_round_start_us = 0;
uint8_t  g_max_sender_seen = 0;
static bool g_poll_seen_this_round = false;  // gates UDP on non-node-0
static bool g_rx_armed = false;
static uint32_t g_rx_arm_us = 0;


// Node0 schedule
static bool     g_sched_init = false;
static uint32_t g_next_frame_us = 0;

// Error flags (bitmask)
static uint32_t g_error_flags = 0;

// Error flags table:
// bit 0  ERR_ROUND_TIMEOUT
// bit 1  ERR_POLL_TX_START_FAIL
// bit 2  ERR_POLL_TXFRS_TIMEOUT
// bit 3  ERR_NODE_TX_START_FAIL
// bit 4  ERR_NODE_TXFRS_TIMEOUT
// bit 5  ERR_RX_FRAME_TOO_LONG
// bit 6  ERR_RX_APP_CODE_MISMATCH
// bit 7  ERR_RX_STALE_ROUND
// bit 8  ERR_RX_UNKNOWN_MSG_TYPE
// bit 9  ERR_IRQ_UNEXPECTED_STATUS
// bit 10 ERR_WIFI_NOT_CONNECTED
// bit 11 ERR_UDP_SEND_FAIL

#define ERR_ROUND_TIMEOUT          (1UL << 0)
#define ERR_POLL_TX_START_FAIL     (1UL << 1)
#define ERR_POLL_TXFRS_TIMEOUT     (1UL << 2)
#define ERR_NODE_TX_START_FAIL     (1UL << 3)
#define ERR_NODE_TXFRS_TIMEOUT     (1UL << 4)
#define ERR_RX_FRAME_TOO_LONG      (1UL << 5)
#define ERR_RX_APP_CODE_MISMATCH   (1UL << 6)
#define ERR_RX_STALE_ROUND         (1UL << 7)
#define ERR_RX_UNKNOWN_MSG_TYPE    (1UL << 8)
#define ERR_IRQ_UNEXPECTED_STATUS  (1UL << 9)
#define ERR_WIFI_NOT_CONNECTED     (1UL << 10)
#define ERR_UDP_SEND_FAIL          (1UL << 11)

// -------------------- Helpers: timestamp pack/unpack --------------------
static uint32_t get_ts32(const uint8_t *p)
{
  uint32_t v = 0;
  v |= (uint32_t)p[0];
  v |= (uint32_t)p[1] << 8;
  v |= (uint32_t)p[2] << 16;
  v |= (uint32_t)p[3] << 24;
  return v;
}

static uint16_t get_u16le(const uint8_t *p)
{
  uint16_t v = 0;
  v |= (uint16_t)p[0];
  v |= (uint16_t)p[1] << 8;
  return v;
}

static void put_ts32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void put_u16le(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

// -------------------- Distance calculation --------------------
static void try_compute_range_with(uint8_t other)
{
  uint8_t self = g_node_idx;
  if (self == other) return;

  if (!ts_tx[self] || !ts_tx[other] ||
      !ts_rx[self][other] || !ts_rx[other][self]) {
    return;
  }

  int32_t rtd_init = (int32_t)(ts_rx[other][self] - ts_tx[self]);
  int32_t rtd_resp = (int32_t)(ts_tx[other]       - ts_rx[self][other]);

  float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

  double tof = ((double)rtd_init -
               (double)rtd_resp * (1.0 - clockOffsetRatio)) *
               0.5 * DWT_TIME_UNITS;
  double distance = tof * SPEED_OF_LIGHT;

  dist_table[self][other] = dist_table[other][self] = distance;
  if (other < 8) {
    g_dist_valid_mask |= (uint8_t)(1U << other);
  }
}

// -------------------- TX frame builders --------------------
static uint16_t build_poll_frame()
{
  memcpy(tx_buffer, uwb_header, sizeof(uwb_header));

  tx_buffer[ALL_MSG_SN_IDX] = frame_seq_nb++;
  tx_buffer[MSG_TYPE_IDX]   = MSG_TYPE_POLL;
  put_u16le(&tx_buffer[ROUND_ID_L_IDX], g_round_id);
  tx_buffer[NODE_ID_IDX]    = 0; // A

  return ALL_MSG_COMMON_LEN + FCS_LEN; // no payload
}

// For node > 0: payload = [rx_0_i][rx_1_i]...[rx_(i-1)_i][tx_i]
static uint16_t build_node_tx_frame()
{
  uint8_t i = g_node_idx;

  memcpy(tx_buffer, uwb_header, sizeof(uwb_header));

  tx_buffer[ALL_MSG_SN_IDX] = frame_seq_nb++;
  tx_buffer[MSG_TYPE_IDX]   = MSG_TYPE_NODE_TX;
  put_u16le(&tx_buffer[ROUND_ID_L_IDX], g_round_id);
  tx_buffer[NODE_ID_IDX]    = i;

  uint8_t *p = &tx_buffer[PAYLOAD_IDX];

  // previous nodes 0..i-1
  for (uint8_t prev = 0; prev < i; prev++) {
    put_ts32(p, ts_rx[prev][i]); // rx_prev_i
    p += TS_SIZE;
  }

  // our own tx timestamp
  put_ts32(p, ts_tx[i]);
  p += TS_SIZE;

  return (uint16_t)(p - tx_buffer) + FCS_LEN;
}

// -------------------- UDP packet format --------------------
#define UDP_MAGIC  ((uint32_t)('U') | ((uint32_t)('W') << 8) | ((uint32_t)('B') << 16) | ((uint32_t)('M') << 24))
#define IMU_MAGIC  ((uint32_t)('I') | ((uint32_t)('M') << 8) | ((uint32_t)('U') << 16) | ((uint32_t)('!') << 24))

typedef struct __attribute__((packed)) {
  uint32_t magic;           // UDP_MAGIC
  uint64_t timestamp_us;    // esp_timer_get_time()
  uint32_t frame_no;        // monotonic local counter
  uint16_t round_id;        // 16-bit UWB round id
  uint8_t  node_idx;
  uint8_t  num_nodes;
  uint8_t  max_sender_seen; // diagnostic: where chain reached on this node
  uint32_t error_flags;     // bitmask
  uint8_t  dist_count;      // = num_nodes - node_idx - 1
  uint8_t  cir_count;       // = node_idx
  uint8_t  dist_valid_mask; // bit i => distance to i valid
  uint8_t  cir_valid_mask;  // bit i => CIR from i valid
} uwb_udp_hdr_t;

typedef struct __attribute__((packed)) {
  uint32_t imu_magic;        // IMU_MAGIC (delimiter)
  uint32_t imu_error_flags;  // bitmask for IMU
  uint16_t i2c_ms_f16;       // float16 milliseconds
  uint16_t reserved;         // 0
} imu_tail_hdr_t;

// UDP buffer (keep under MTU; worst-case here is well below 1500)
static uint8_t udp_buf[1500];

// ---------- float32 -> IEEE754 float16 ----------
static inline uint16_t float_to_half_ieee754(float f)
{
  union { float f; uint32_t u; } v;
  v.f = f;

  uint32_t sign = (v.u >> 31) & 0x1;
  int32_t  exp  = (int32_t)((v.u >> 23) & 0xFF) - 127;
  uint32_t mant = v.u & 0x7FFFFF;

  uint16_t hs = (uint16_t)(sign << 15);

  if ((v.u & 0x7FFFFFFF) == 0) {
    return hs; // +/-0
  }

  if (((v.u >> 23) & 0xFF) == 0xFF) {
    // Inf/NaN
    uint16_t he = 0x1F << 10;
    uint16_t hm = (mant != 0) ? 0x200 : 0; // quiet NaN minimal
    return (uint16_t)(hs | he | hm);
  }

  int32_t half_exp = exp + 15;

  if (half_exp >= 31) {
    // overflow -> Inf
    return (uint16_t)(hs | (0x1F << 10));
  } else if (half_exp <= 0) {
    // subnormal or underflow to zero
    if (half_exp < -10) {
      return hs; // too small -> 0
    }
    // subnormal: implicit leading 1
    uint32_t m = mant | 0x800000;
    int32_t  shift = 1 - half_exp;
    uint32_t half_mant = m >> (shift + 13);
    return (uint16_t)(hs | (uint16_t)half_mant);
  } else {
    // normal
    uint16_t he = (uint16_t)(half_exp << 10);
    uint16_t hm = (uint16_t)(mant >> 13);
    return (uint16_t)(hs | he | hm);
  }
}

static void send_udp_frame_with_imu_tail()
{
  imu_latest_t imu_latest;
  memset(&imu_latest, 0, sizeof(imu_latest));
#if ENABLE_IMU
  imu_get_latest_bno085(&imu_latest);
#else
  imu_latest.error_flags = IMU_ERR_NOT_READY;
#endif

  // Build UWB header + payload into udp_buf
  uwb_udp_hdr_t hdr;
  hdr.magic         = UDP_MAGIC;
  hdr.timestamp_us  = (uint64_t)esp_timer_get_time();
  hdr.frame_no      = g_frame_no;
  hdr.round_id      = g_round_id;
  hdr.node_idx      = g_node_idx;
  hdr.num_nodes     = g_num_nodes;
  hdr.max_sender_seen = g_max_sender_seen;
  hdr.error_flags   = g_error_flags;

  uint8_t dist_count = 0;
  if (g_node_idx < g_num_nodes) dist_count = (uint8_t)(g_num_nodes - g_node_idx - 1);
  uint8_t cir_count  = 0;
  if (g_node_idx < g_num_nodes) cir_count  = g_node_idx;

  hdr.dist_count       = dist_count;
  hdr.cir_count        = cir_count;
  hdr.dist_valid_mask  = g_dist_valid_mask;
  hdr.cir_valid_mask   = g_cir_valid_mask;

  uint8_t *p = udp_buf;
  memcpy(p, &hdr, sizeof(hdr));
  p += sizeof(hdr);

  // distances: float, for other = node_idx+1 .. num_nodes-1
  for (uint8_t other = (uint8_t)(g_node_idx + 1); other < g_num_nodes; other++) {
    float d = (float)dist_table[g_node_idx][other];
    memcpy(p, &d, sizeof(d));
    p += sizeof(d);
  }

  // CIR blocks: sender 0 .. node_idx-1, each 96*uint16_t
  for (uint8_t sender = 0; sender < g_node_idx; sender++) {
    memcpy(p, cir_mag16[sender], CIR_SAMPLES * sizeof(uint16_t));
    p += (CIR_SAMPLES * sizeof(uint16_t));
  }

  // ---------------- IMU tail appended here ----------------
  imu_tail_hdr_t imuh;
  imuh.imu_magic = IMU_MAGIC;
  imuh.imu_error_flags = imu_latest.error_flags;
  imuh.reserved = 0;

  float i2c_ms = ((float)imu_latest.i2c_read_us) / 1000.0f;
  imuh.i2c_ms_f16 = float_to_half_ieee754(i2c_ms);

  // Append IMU header
  memcpy(p, &imuh, sizeof(imuh));
  p += sizeof(imuh);

  // Append IMU data (44 bytes): accel + gyro + quat + quat accuracy
  memcpy(p, &imu_latest.ax, 4); p += 4;
  memcpy(p, &imu_latest.ay, 4); p += 4;
  memcpy(p, &imu_latest.az, 4); p += 4;

  memcpy(p, &imu_latest.gx, 4); p += 4;
  memcpy(p, &imu_latest.gy, 4); p += 4;
  memcpy(p, &imu_latest.gz, 4); p += 4;

  memcpy(p, &imu_latest.qx, 4); p += 4;
  memcpy(p, &imu_latest.qy, 4); p += 4;
  memcpy(p, &imu_latest.qz, 4); p += 4;
  memcpy(p, &imu_latest.qw, 4); p += 4;

  memcpy(p, &imu_latest.quat_accuracy_rad, 4); p += 4;

  size_t pkt_len = (size_t)(p - udp_buf);

  if (!g_wifi_connected) {
    g_error_flags |= ERR_WIFI_NOT_CONNECTED;
    return;
  }

  if (!g_udp_ready) {
    g_error_flags |= ERR_UDP_SEND_FAIL;
    return;
  }

  size_t sent = udp.writeTo(udp_buf, pkt_len, UDP_SERVER_IP, g_udp_remote_port);
  if (sent != pkt_len) {
    g_error_flags |= ERR_UDP_SEND_FAIL;
  }
}

// -------------------- Round init/finalize + Part 3 hook --------------------
static void clear_round_outputs()
{
  memset(ts_rx, 0, sizeof(ts_rx));
  memset(ts_tx, 0, sizeof(ts_tx));

  for (uint8_t i = 0; i < MAX_NODES; i++) {
    dist_table[g_node_idx][i] = NAN;
    dist_table[i][g_node_idx] = NAN;
  }

  memset(cir_mag16, 0, sizeof(cir_mag16));
  memset(cir_rx_pacc, 0, sizeof(cir_rx_pacc));
  g_cir_valid_mask = 0;
  g_dist_valid_mask = 0;
}

static void start_round_common(uint16_t new_round_id)
{
  g_round_id = new_round_id;

  g_frame_no++;
  g_error_flags = 0;
  g_round_state = ROUND_WAITING_FRAMES;
  g_round_start_us = micros();
  g_max_sender_seen = 0;

  g_poll_seen_this_round = false;   // <-- add this

  clear_round_outputs();
}

// Part 3 placeholder: after last node heard OR timeout
static void post_round_tasks()
{
  // ---- PLACEHOLDER (Part 3) ----
  // Add extra processing here later.
  // -----------------------------
  if (g_node_idx != 0 && !g_poll_seen_this_round) {
    return;
  }

  // Send UDP packet (UWB payload + IMU tail)
  send_udp_frame_with_imu_tail();
}

static void finalize_round()
{
  if (g_round_state != ROUND_WAITING_FRAMES) return;
  Serial.print(ts_tx[g_node_idx]);
  Serial.print("  |  ");
  Serial.print(ts_rx[0][g_node_idx]);
  Serial.print("  |  ");
  Serial.print(ts_rx[1][g_node_idx]);
  Serial.print("  |  ");
  Serial.print(ts_rx[2][g_node_idx]);
  Serial.print("  |  ");
  Serial.print(ts_rx[3][g_node_idx]);
  Serial.print("  |  ");
  Serial.print(ts_rx[4][g_node_idx]);
  Serial.print("  |  ");
  Serial.println(ts_rx[5][g_node_idx]);
  post_round_tasks();
  g_round_state = ROUND_IDLE;
}

// -------------------- Chained slot scheduling --------------------
static void maybe_schedule_own_tx(uint8_t sender, uint32_t rx_ts)
{
  if (g_node_idx != sender + 1) return;
  if (g_node_idx >= g_num_nodes) return;

  uint8_t i = g_node_idx;

  uint64_t delay_uus = RESP_GAP_UUS;
  uint32_t delay_dtu = (uint32_t)((delay_uus * UUS_TO_DWT_TIME) >> 8);

  uint32_t sys_hi = dwt_readsystimestamphi32();
  uint32_t tx_time = sys_hi + delay_dtu;

  uint64_t tx_ts64 = (((uint64_t)(tx_time & 0xFFFFFFFEUL)) << 8) + ANT_DELAY;
  uint32_t tx_ts32 = (uint32_t)tx_ts64;

  ts_tx[i] = tx_ts32;

  uint16_t frame_len = build_node_tx_frame();

  dwt_setdelayedtrxtime(tx_time);
  dwt_writetxdata(frame_len - FCS_LEN, tx_buffer, 0);
  dwt_writetxfctrl(frame_len, 0, 1);

  digitalWrite(PIN_LED, HIGH);

  int ret = dwt_starttx(DWT_START_TX_DELAYED);
  if (ret == DWT_SUCCESS) {
    uint32_t t0 = micros();
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
      if ((micros() - t0) > 5000) {
        g_error_flags |= ERR_NODE_TXFRS_TIMEOUT;
        break;
      }
    }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  } else {
    g_error_flags |= ERR_NODE_TX_START_FAIL;
  }

  digitalWrite(PIN_LED, LOW);

  if (i == (uint8_t)(g_num_nodes - 1)) {
    g_max_sender_seen = (uint8_t)(g_num_nodes - 1);
    finalize_round();
  }
}

// -------------------- RX handlers --------------------
static void on_poll_from_A(uint16_t round_id, uint32_t rx_ts)
{
  start_round_common(round_id);
  g_poll_seen_this_round = true;    // <-- add this (POLL received)

  ts_rx[0][g_node_idx] = rx_ts;

  if (g_node_idx > 0) {
    capture_cir_from_sender(0);
  }

  maybe_schedule_own_tx(0, rx_ts);
}

static void on_node_frame(uint8_t sender, uint16_t round_id,
                          uint32_t frame_len, uint32_t rx_ts)
{
  ts_rx[sender][g_node_idx] = rx_ts;

  if (sender > g_max_sender_seen) g_max_sender_seen = sender;

  uint8_t *p = &rx_buffer[PAYLOAD_IDX];
  uint8_t num_prev = sender;

  for (uint8_t prev = 0; prev < num_prev; prev++) {
    uint32_t rx_prev_at_sender = get_ts32(p);
    ts_rx[prev][sender] = rx_prev_at_sender;
    p += TS_SIZE;
  }

  uint32_t tx_sender = get_ts32(p);
  ts_tx[sender] = tx_sender;

  if (sender < g_node_idx) {
    capture_cir_from_sender(sender);
  }

  maybe_schedule_own_tx(sender, rx_ts);

  try_compute_range_with(sender);

  if (sender == (uint8_t)(g_num_nodes - 1)) {
    finalize_round();
  }
}

static void handle_good_rx()
{
  uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
  if (frame_len > sizeof(rx_buffer)) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    g_error_flags |= ERR_RX_FRAME_TOO_LONG;
    return;
  }

  dwt_readrxdata(rx_buffer, frame_len - FCS_LEN, 0);

  uint8_t app_code = rx_buffer[APP_CODE_IDX];
  if (app_code != 0xE0){
    g_error_flags |= ERR_RX_APP_CODE_MISMATCH;
    return;
  }

  uint8_t msg_type = rx_buffer[MSG_TYPE_IDX];
  uint16_t round_id = get_u16le(&rx_buffer[ROUND_ID_L_IDX]);
  uint8_t sender   = rx_buffer[NODE_ID_IDX];
  Serial.print("sender: ");
  Serial.println(sender);

  uint32_t rx_ts = dwt_readrxtimestamplo32();

  if (msg_type == MSG_TYPE_POLL && sender == 0) {
    on_poll_from_A(round_id, rx_ts);
    return;
  }

  if (msg_type == MSG_TYPE_NODE_TX) {
    if (round_id != g_round_id) {
      g_error_flags |= ERR_RX_STALE_ROUND;
      return;
    }
    on_node_frame(sender, round_id, frame_len, rx_ts);
    return;
  }

  
  g_error_flags |= ERR_RX_UNKNOWN_MSG_TYPE;
}

// -------------------- Round supervision (Node 0 only) --------------------
static void start_new_round()
{
  if (g_node_idx != 0) return;

  dwt_forcetrxoff();

  g_round_id++;
  if (g_round_id == 0) g_round_id = 1;

  start_round_common(g_round_id);
  g_poll_seen_this_round = true; 

  uint16_t frame_len = build_poll_frame();

  dwt_writetxdata(frame_len - FCS_LEN, tx_buffer, 0);
  dwt_writetxfctrl(frame_len, 0, 1);

  digitalWrite(PIN_LED, HIGH);

  int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
  if (ret != DWT_SUCCESS) {
    g_error_flags |= ERR_POLL_TX_START_FAIL;
    digitalWrite(PIN_LED, LOW);
    finalize_round();
    return;
  }

  uint32_t t0 = micros();
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
    if ((micros() - t0) > 5000) {
      g_error_flags |= ERR_POLL_TXFRS_TIMEOUT;
      break;
    }
  }

  dwt_write32bitreg(SYS_STATUS_ID,
                    SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_TXFRB_BIT_MASK |
                    SYS_STATUS_TXPRS_BIT_MASK | SYS_STATUS_TXPHS_BIT_MASK);

  digitalWrite(PIN_LED, LOW);

  ts_tx[0] = dwt_readtxtimestamplo32();
}

static void supervise_round()
{
  if (g_node_idx != 0) return;

  uint32_t now = micros();
  if (!g_sched_init) {
    g_next_frame_us = now;
    g_sched_init = true;
  }

  if (g_round_state == ROUND_WAITING_FRAMES) {
    if ((uint32_t)(now - g_round_start_us) > RANGING_TIMEOUT_US) {
      g_error_flags |= ERR_ROUND_TIMEOUT;
      finalize_round();
    }
  }

  if ((int32_t)(now - g_next_frame_us) >= 0) {
    if (g_round_state == ROUND_WAITING_FRAMES) {
      g_error_flags |= ERR_ROUND_TIMEOUT;
      finalize_round();
    }

    start_new_round();
    g_rx_armed = false;

    g_next_frame_us += FRAME_PERIOD_US;
    while ((int32_t)(now - g_next_frame_us) >= 0) {
      g_next_frame_us += FRAME_PERIOD_US;
    }
  }
}

static void check_round_timeout_all_nodes()
{
  if (g_round_state != ROUND_WAITING_FRAMES) return;

  uint32_t now = micros();
  if ((uint32_t)(now - g_round_start_us) > RANGING_TIMEOUT_US) {
    g_error_flags |= ERR_ROUND_TIMEOUT;
    finalize_round();
  }
}

// -------------------- WiFi helpers --------------------
static void blink_node_idx_pattern_once()
{
  for (uint32_t i = 0; i < g_node_idx; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(150);
    digitalWrite(PIN_LED, LOW);
    delay(150);
  }
}

static void connectToWiFi(const char *ssid, const char *pwd)
{
  WiFi.disconnect(true);
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pwd);
}

void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      g_wifi_connected = true;
      udp.close();
      g_udp_ready = udp.listen(g_udp_local_port);
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      g_wifi_connected = false;
      g_udp_ready = false;
      udp.close();
      break;

    default: break;
  }
}

// -------------------- Setup & main loop --------------------
void setup()
{
  UART_init();
  test_run_info((unsigned char*)APP_NAME);

  Serial.begin(921600);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(200);
  while (!dwt_checkidlerc()) {
    UART_puts("IDLE FAILED\r\n");
    while (1) {}
  }

  dwt_softreset();
  delay(200);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    UART_puts("INIT FAILED\r\n");
    while (1) {}
  }

  if (dwt_configure(&config)) {
    UART_puts("CONFIG FAILED\r\n");
    while (1) {}
  }

  while ((dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_CP_LOCK_BIT_MASK) == 0) {}
  dwt_write32bitreg(SYS_STATUS_ID,
                    SYS_STATUS_CP_LOCK_BIT_MASK | SYS_STATUS_RXFCG_BIT_MASK);

  dwt_configuretxrf(&txconfig_options);
  dwt_setrxantennadelay(ANT_DELAY);
  dwt_settxantennadelay(ANT_DELAY);
  dwt_setrxtimeout(0);
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  pinMode(PIN_IRQ, INPUT_PULLUP);
  attachInterruptArg(PIN_IRQ, dw3000_gpio_isr, nullptr, RISING);

  dwt_setinterrupt(SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK, 0, DWT_ENABLE_INT_ONLY);

  port_set_spi_hz(20000000);

  prefs.begin("config", false);
  g_node_idx  = prefs.getUInt("node_idx", 10);

  if (g_node_idx == 10) {
    while(1) delay(100);
  }

  // Blink node_idx times on LED for quick visual ID (as original)
  blink_node_idx_pattern_once();

  memset(ts_rx, 0, sizeof(ts_rx));
  memset(ts_tx, 0, sizeof(ts_tx));
  memset(dist_table, 0, sizeof(dist_table));
  memset(cir_mag16, 0, sizeof(cir_mag16));
  memset(cir_rx_pacc, 0, sizeof(cir_rx_pacc));

  g_cir_valid_mask = 0;
  g_dist_valid_mask = 0;

  g_udp_local_port  = (uint16_t)(UDP_PORT_BASE + g_node_idx);
  g_udp_remote_port = (uint16_t)(UDP_PORT_BASE + g_node_idx);

#if ENABLE_IMU
  bool imu_ok = imu_init_bno085(
      IMU_IRQ_PIN,
      IMU_I2C_ADDR,
      I2C_CLOCK_HZ,
      IMU_REPORT_INTERVAL_MS);
  if (!imu_ok) {
#if REQUIRE_IMU
    while (1) {
      digitalWrite(PIN_LED, HIGH);
      delay(100);
      digitalWrite(PIN_LED, LOW);
      delay(100);
    }
#endif
  }
#endif

  connectToWiFi(WIFI_SSID, WIFI_PSK);

#if REQUIRE_WIFI_CONNECTED
  while (!g_wifi_connected) {
    blink_node_idx_pattern_once();
    delay(250);
  }
#endif
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void loop()
{
#if ENABLE_IMU
  // Keep IMU update path running continuously; UDP uses latest cached values.
  uint32_t imu_tb = micros();
  imu_poll_bno085();
  uint32_t imu_te = micros();
  uint32_t imu_time = imu_te - imu_tb;
  if (imu_time > 10){
    Serial.print("imu time: ");
    Serial.println(imu_te - imu_tb);
  }
    
#endif

#if REQUIRE_WIFI_CONNECTED
  if (!g_wifi_connected) {
    return;
  }
#endif

  supervise_round();
  check_round_timeout_all_nodes();

  if (!g_rx_armed) {
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    g_rx_armed = true;
    g_rx_arm_us = micros();
  }

  // // a watchdog to re-arm RX occasionally
  // // without spamming it (e.g. if something gets stuck)
  if (g_rx_armed && (uint32_t)(micros() - g_rx_arm_us) > 50000) { // 50 ms window
    // Re-arm RX, but only at low rate
    dwt_forcetrxoff();
    g_rx_armed = false;
    // dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    // dwt_rxenable(DWT_START_RX_IMMEDIATE);
    // g_rx_arm_us = micros();
    return;
  }

  if (!dw_irq_flag) {
    return;
  }

  dw_irq_flag = false;
  uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);

  if (status & SYS_STATUS_RXFCG_BIT_MASK) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    // unsigned long ts_now = micros();
    // Serial.println(ts_now-last_ts);
    // last_ts = ts_now;
    handle_good_rx();
  } else {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    g_error_flags |= ERR_IRQ_UNEXPECTED_STATUS;
  }
  dwt_forcetrxoff();
  g_rx_armed = false;
}
