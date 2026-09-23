#include "dw3000.h"
#include "esp_pm.h"

#define APP_NAME "SIMPLE RX v1.1"

#define CIR_SAMPLES 128
static uint8_t cir_buf[1 + CIR_SAMPLES * 6];
static uint16_t cir_mag16[CIR_SAMPLES];

static uint8_t rx_poll_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E'};


static esp_pm_lock_handle_t apb_lock;
static esp_pm_lock_handle_t cpu_lock;

// connection pins
const uint8_t PIN_RST = D2; // reset pin
const uint8_t PIN_IRQ = D0; // irq pin
const uint8_t PIN_SS = D1;   // spi select pin

static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (128 + 1 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64, /* STS length, see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0     /* PDOA mode off */
};


/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
uint32_t status_reg;
/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
uint16_t frame_len;

static volatile bool dw_irq_flag = false;

void IRAM_ATTR dw3000_gpio_isr(void*) {
  dw_irq_flag = true;
}

static void send_cir_mag16_packet() {
  const uint8_t magic[4] = {'D','W','3','M'};            // marker for mag16 frames
  const uint16_t payload_len = CIR_SAMPLES * 2;          // 256 bytes
  Serial.write(magic, 4);
  Serial.write((const uint8_t*)&payload_len, 2);         // little-endian length
  Serial.write((const uint8_t*)cir_mag16, payload_len);  // send 16-bit mags (LE on AVR/ARM)
}

static void send_cir_packet() {
  const uint8_t magic[4] = {'D','W','3','C'};      // frame marker
  const uint16_t payload_len = CIR_SAMPLES * 6;     // 1536 bytes (skip dummy)
  Serial.write(magic, 4);
  Serial.write((uint8_t*)&payload_len, 2);          // little-endian length
  Serial.write(&cir_buf[1], payload_len);           // send I/Q only
}

static inline int32_t sign_extend_24(uint32_t x) {
  if (x & 0x00800000UL) x |= 0xFF000000UL;
  return (int32_t)x;
}

static inline uint32_t uabs32(int32_t v) {
  return (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
}

void compress_cir_to_mag16(const uint8_t *buf, uint16_t *out16, size_t nSamples)
{
  const uint8_t *p = buf + 1;
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
    // uint16_t c16 = (uint16_t)((mag >> 1) & 0xFFFFU);
    uint16_t c16 = (uint16_t)(mag & 0xFFFFU);
    out16[k] = c16;
  }
}


void setup()
{

  esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "apb80", &apb_lock);
  esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "cpu160", &cpu_lock);

  esp_pm_lock_acquire(apb_lock);
  esp_pm_lock_acquire(cpu_lock);

  
  UART_init();
  test_run_info((unsigned char *)APP_NAME);

  /* Configure SPI rate, DW3000 supports up to 38 MHz */
  /* Reset DW IC */
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(200); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ;
  }

  dwt_softreset();
  delay(200);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  // dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  // Configure DW IC. See NOTE 5 below.
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1);
  }

  while ((dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_CP_LOCK_BIT_MASK) == 0) { }
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_CP_LOCK_BIT_MASK | SYS_STATUS_RXFCG_BIT_MASK);
  pinMode(PIN_IRQ, INPUT_PULLUP);
  attachInterruptArg(PIN_IRQ, dw3000_gpio_isr, nullptr, RISING);
  dwt_setinterrupt(SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK, 0, DWT_ENABLE_INT_ONLY);
  port_set_spi_hz(20000000);

  Serial.begin(115200);
  
  // Give the serial connection a moment to stabilize
  delay(1000);
}

void loop()
{
  memset(rx_buffer, 0, sizeof(rx_buffer));
  dw_irq_flag = false;

  // Arm RX
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_RXFCG_BIT_MASK);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  // Wait for IRQ
  while (!dw_irq_flag) { /* spin or light sleep */ }
  dw_irq_flag = false;

  // Check what fired
  status_reg = dwt_read32bitreg(SYS_STATUS_ID);
  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;    
    
    while ( (dwt_read32bitoffsetreg(0x01, 0x24) & (1u<<2)) == 0 ) ;

    Serial.println("RX Frame received");
    uint16_t ip_fp = dwt_read16bitoffsetreg(0x0C, 0x48);
    uint16_t fp_int = ip_fp >> 6;
    uint16_t start  = (fp_int > 64) ? fp_int - 64 : 0;

    unsigned long t0 = micros();
    dwt_readaccdata(cir_buf, sizeof(cir_buf), 700);
    // compress_cir_to_mag16(cir_buf, cir_mag16, CIR_SAMPLES);
    // send_cir_mag16_packet();
    send_cir_packet();
    unsigned long t1 = micros();
    Serial.println(t1-t0);

    delay(100);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_CIADONE_BIT_MASK);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
  } else {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }
}