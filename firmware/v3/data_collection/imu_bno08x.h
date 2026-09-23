#pragma once

#include <Arduino.h>
#include <stdint.h>

typedef struct {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float qx;
  float qy;
  float qz;
  float qw;
  float quat_accuracy_rad;
  uint32_t i2c_read_us;
  uint32_t error_flags;
} imu_latest_t;

// IMU error flags (bitmask)
#define IMU_ERR_NOT_READY     (1UL << 0)
#define IMU_ERR_I2C_TIMEOUT   (1UL << 1)
#define IMU_ERR_INIT_FAIL     (1UL << 2)
#define IMU_ERR_REPORT_CFG    (1UL << 3)

bool imu_init_bno085(uint8_t irq_pin,
                     uint8_t i2c_addr,
                     uint32_t i2c_clock_hz,
                     uint16_t report_interval_ms);

void imu_poll_bno085();
void imu_get_latest_bno085(imu_latest_t *out);
uint32_t imu_get_error_flags_bno085();
bool imu_is_ready_bno085();

