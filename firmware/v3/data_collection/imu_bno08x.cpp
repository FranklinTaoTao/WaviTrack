#include "imu_bno08x.h"

#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

static const uint8_t IMU_REPORTS_ALL_MASK = 0x07; // quat + accel + gyro
static const uint32_t IMU_I2C_TIMEOUT_LATCH_US = 12000;

static BNO08x g_imu;
static volatile bool g_imu_irq_fired = false;
static uint8_t g_imu_irq_pin = 0;
static bool g_imu_started = false;
static uint8_t g_seen_reports_mask = 0;
static imu_latest_t g_latest = {
  0.0f, 0.0f, 0.0f, // accel
  0.0f, 0.0f, 0.0f, // gyro
  0.0f, 0.0f, 0.0f, 1.0f, // quat
  0.0f, // quat accuracy
  0,    // i2c read us
  IMU_ERR_NOT_READY
};

void IRAM_ATTR imu_irq_isr_bno085()
{
  g_imu_irq_fired = true;
}

bool imu_init_bno085(uint8_t irq_pin,
                     uint8_t i2c_addr,
                     uint32_t i2c_clock_hz,
                     uint16_t report_interval_ms)
{
  g_imu_irq_pin = irq_pin;
  g_imu_irq_fired = false;
  g_imu_started = false;
  g_seen_reports_mask = 0;
  g_latest.error_flags = IMU_ERR_NOT_READY;
  g_latest.i2c_read_us = 0;

  Wire.begin();
  Wire.setClock(i2c_clock_hz);

  pinMode(g_imu_irq_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(g_imu_irq_pin), imu_irq_isr_bno085, FALLING);

  // Use manual IRQ handling on ESP32 by passing -1 for INT/RST.
  if (!g_imu.begin(i2c_addr, Wire, -1, -1)) {
    g_latest.error_flags |= IMU_ERR_INIT_FAIL;
    return false;
  }

  bool ok = true;
  ok &= g_imu.enableRotationVector(report_interval_ms);
  ok &= g_imu.enableAccelerometer(report_interval_ms);
  ok &= g_imu.enableGyro(report_interval_ms);

  if (!ok) {
    g_latest.error_flags |= IMU_ERR_REPORT_CFG;
    return false;
  }

  g_imu_started = true;
  return true;
}

void imu_poll_bno085()
{
  if (!g_imu_started) return;

  if (!g_imu_irq_fired && digitalRead(g_imu_irq_pin) != LOW) {
    return;
  }

  noInterrupts();
  g_imu_irq_fired = false;
  interrupts();

  uint32_t t0 = micros();
  uint8_t report_count = 0;

  while (g_imu.getSensorEvent()) {
    report_count++;
    uint8_t id = g_imu.getSensorEventID();

    if (id == SENSOR_REPORTID_ROTATION_VECTOR) {
      g_latest.qx = g_imu.getQuatI();
      g_latest.qy = g_imu.getQuatJ();
      g_latest.qz = g_imu.getQuatK();
      g_latest.qw = g_imu.getQuatReal();
      g_latest.quat_accuracy_rad = g_imu.getQuatRadianAccuracy();
      g_seen_reports_mask |= (1U << 0);
    } else if (id == SENSOR_REPORTID_ACCELEROMETER) {
      g_latest.ax = g_imu.getAccelX();
      g_latest.ay = g_imu.getAccelY();
      g_latest.az = g_imu.getAccelZ();
      g_seen_reports_mask |= (1U << 1);
    } else if (id == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
      g_latest.gx = g_imu.getGyroX();
      g_latest.gy = g_imu.getGyroY();
      g_latest.gz = g_imu.getGyroZ();
      g_seen_reports_mask |= (1U << 2);
    }

    if (report_count >= 32) {
      break;
    }
  }

  g_latest.i2c_read_us = (uint32_t)(micros() - t0);

  if (g_latest.i2c_read_us >= IMU_I2C_TIMEOUT_LATCH_US) {
    g_latest.error_flags |= IMU_ERR_I2C_TIMEOUT;
  }

  if ((g_seen_reports_mask & IMU_REPORTS_ALL_MASK) == IMU_REPORTS_ALL_MASK) {
    g_latest.error_flags &= ~IMU_ERR_NOT_READY;
  } else {
    g_latest.error_flags |= IMU_ERR_NOT_READY;
  }
}

void imu_get_latest_bno085(imu_latest_t *out)
{
  if (out == nullptr) return;
  *out = g_latest;
}

uint32_t imu_get_error_flags_bno085()
{
  return g_latest.error_flags;
}

bool imu_is_ready_bno085()
{
  return ((g_latest.error_flags & IMU_ERR_NOT_READY) == 0);
}

