#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

// ESP32-C6 + BNO085 (I2C)
static const uint8_t IMU_I2C_ADDR = 0x4A;
static const uint8_t IMU_IRQ_PIN  = 16;
static const uint8_t PIN_LED = D3;
static const uint32_t I2C_CLOCK_HZ = 400000;
static const uint16_t REPORT_INTERVAL_MS = 10; // 100 Hz target
static const uint32_t I2C_TIMEOUT_LATCH_US = 12000; // timeout-like long transaction

BNO08x imu;

volatile bool imu_irq_fired = false;
bool g_imu_error = false;

void IRAM_ATTR imu_irq_isr()
{
  imu_irq_fired = true;
}

// Latest values
float qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f;
float q_acc_rad = 0.0f;
float ax = 0.0f, ay = 0.0f, az = 0.0f;
float gx = 0.0f, gy = 0.0f, gz = 0.0f;

bool have_quat = false;
bool have_accel = false;
bool have_gyro = false;

static void update_error_led_5hz()
{
  // 5 Hz square wave => 100 ms HIGH + 100 ms LOW
  static uint32_t last_toggle_ms = 0;
  static bool led_on = false;
  uint32_t now_ms = millis();
  if ((uint32_t)(now_ms - last_toggle_ms) >= 100) {
    led_on = !led_on;
    digitalWrite(PIN_LED, led_on ? HIGH : LOW);
    last_toggle_ms = now_ms;
  }
}

static bool init_reports()
{
  bool ok = true;
  ok &= imu.enableRotationVector(REPORT_INTERVAL_MS);
  ok &= imu.enableAccelerometer(REPORT_INTERVAL_MS);
  ok &= imu.enableGyro(REPORT_INTERVAL_MS);
  return ok;
}

void setup()
{
  Serial.begin(115200);
  delay(300);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  pinMode(IMU_IRQ_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IMU_IRQ_PIN), imu_irq_isr, FALLING);

  // Pass -1 for INT/RST so we can handle IRQ manually on ESP32.
  if (!imu.begin(IMU_I2C_ADDR, Wire, -1, -1)) {
    Serial.println("BNO085 init failed at 0x4A");
    while (1) {
      delay(1000);
    }
  }

  if (!init_reports()) {
    Serial.println("Failed to enable one or more reports");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("BNO085 IRQ test started");
  Serial.println("Columns: i2c_us,reports,qx,qy,qz,qw,q_acc_rad,ax,ay,az,gx,gy,gz");
}

void loop()
{
  if (g_imu_error) {
    update_error_led_5hz();
    return;
  }

  if (!imu_irq_fired && digitalRead(IMU_IRQ_PIN) != LOW) {
    return;
  }

  noInterrupts();
  imu_irq_fired = false;
  interrupts();

  digitalWrite(PIN_LED, HIGH); // reading window start
  uint32_t t0 = micros();
  uint8_t report_count = 0;

  // Drain all currently pending sensor events.
  while (imu.getSensorEvent()) {
    report_count++;
    uint8_t id = imu.getSensorEventID();

    if (id == SENSOR_REPORTID_ROTATION_VECTOR) {
      qx = imu.getQuatI();
      qy = imu.getQuatJ();
      qz = imu.getQuatK();
      qw = imu.getQuatReal();
      q_acc_rad = imu.getQuatRadianAccuracy();
      have_quat = true;
    } else if (id == SENSOR_REPORTID_ACCELEROMETER) {
      ax = imu.getAccelX();
      ay = imu.getAccelY();
      az = imu.getAccelZ();
      have_accel = true;
    } else if (id == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
      gx = imu.getGyroX();
      gy = imu.getGyroY();
      gz = imu.getGyroZ();
      have_gyro = true;
    }

    if (report_count >= 32) {
      break;
    }
  }

  uint32_t i2c_us = micros() - t0;
  digitalWrite(PIN_LED, LOW); // reading window end

  // Latch IMU error only on timeout-like I2C duration, not on empty report cycles.
  if (i2c_us >= I2C_TIMEOUT_LATCH_US) {
    g_imu_error = true;
    return;
  }

  if (!(have_quat && have_accel && have_gyro)) {
    return;
  }

  Serial.printf(
      "%lu,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
      (unsigned long)i2c_us,
      (unsigned int)report_count,
      qx, qy, qz, qw, q_acc_rad,
      ax, ay, az,
      gx, gy, gz);
}
