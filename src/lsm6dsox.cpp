// Copyright 2022 Pascal Massimino
//
// MIT License
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////
// LSM6DSOX gyro/accel
// https://www.st.com/content/st_com/en/products/mems-and-sensors/inemo-inertial-modules/lsm6dsox.html
//
// Author: skal (pascal.massimino@gmail.com)
////////////////////////////////////////////////////////////////////////////////

#include "sklmpu9255.h"

#include "internal.h"

#include <unistd.h>
#include <initializer_list>

namespace skl {

////////////////////////////////////////////////////////////////////////////////
// definitions of sensors addresses

#define LSM_ADDRESS             0x6a    // I2C: default LSMxxx device address
#define LSM_WHO_AM_I            0x0f    // self-identify: 0x6c or 0x69
// control registers
#define CTRL1_XL                0x10    // Accel: bit2-3: full-scale, bit 4-7: ODR
#define CTRL2_G                 0x11    // Gyro:  bit1-3: full_scale, bit 4-7: ODR
#define CTRL3_C                 0x12
#define CTRL5_C                 0x14    // bit7: ultra-low power, bit0-3:self-tests
#define CTRL7_G                 0x16
#define CTRL8_XL                0x17
#define STATUS_REG              0X1e

#define TEMPERATURE_OUT         0x20   // 0x20 / 0x21

// accel / gyro
#define ACCEL_OUT               0x28   // 0x22 -> 0x2d : linear accel
#define GYRO_OUT                0x22   // 0x22 -> 0x27 : angular rate

////////////////////////////////////////////////////////////////////////////////

bool CheckStatusBit(uint8_t bit) {
  uint8_t v;
  return I2C_read_bytes(LSM_ADDRESS, STATUS_REG, &v, 1) ? !!(v & bit) : false;
}

bool LSM6DSOX::init() {
  CHECK_OK(I2C_is_connected(LSM_ADDRESS));
  uint8_t id;
  CHECK_READ(LSM_ADDRESS, LSM_WHO_AM_I, &id, 1);
  CHECK_OK(id == WAI());
  LOG_MSG("WHO AM I: imu = 0x%.2x [%s]\n", id,
          id == 0x6c ? "LSM6DSOX" :
          id == 0x69 ? "LSM6DSOX" : "MPU????");

  // accel : 104Hz, 4G, bypass, low-pass filtering
  CHECK_WRITE(LSM_ADDRESS, CTRL1_XL, 0x4a);
  // gyro : 104Hz, 2000dps
  CHECK_WRITE(LSM_ADDRESS, CTRL2_G, 0x4c);
  // gyro is 'high-performance', 16MHz bandwidth
  CHECK_WRITE(LSM_ADDRESS, CTRL7_G, 0x00);
  // ODR config: ODR/4
  CHECK_WRITE(LSM_ADDRESS, CTRL8_XL, 0x09);

  CHECK_OK(set_gyro_scale(GYRO_FULL_SCALE_250DPS));
  CHECK_OK(set_accel_scale(ACCEL_FULL_SCALE_2G));

  return true;
}

bool LSM6DSOX::close() {
  CHECK_WRITE(LSM_ADDRESS, CTRL1_XL, 0x00);
  CHECK_WRITE(LSM_ADDRESS, CTRL2_G, 0x00);
  return true;
}

float LSM6DSOX::temperature() const {
  if (!CheckStatusBit(4)) return 0.;
  uint8_t tmp[2];
  if (!I2C_read_bytes(LSM_ADDRESS, TEMPERATURE_OUT, tmp, 2)) return -273.;
  return get_16s_le(tmp) / 256.0f + 25.0f;
}

bool LSM6DSOX::set_gyro_scale(gyro_full_scale_t scale) {
  uint8_t v;
  CHECK_READ(LSM_ADDRESS, CTRL2_G, &v, 1);
  v &= ~(7 << 1);  // clear bit 1-3
  float dps;   // degree per seconds
  switch (scale) {
    case GYRO_FULL_SCALE_125DPS:  v |= 1 << 1; dps =  125.f; break;
    case GYRO_FULL_SCALE_250DPS:  v |= 0 << 1; dps =  250.f; break;
    case GYRO_FULL_SCALE_500DPS:  v |= 2 << 1; dps =  500.f; break;
    case GYRO_FULL_SCALE_1000DPS: v |= 4 << 1; dps = 1000.f; break;
    default:
    case GYRO_FULL_SCALE_2000DPS: v |= 6 << 1; dps = 2000.f; break;
  }
  CHECK_WRITE(LSM_ADDRESS, CTRL2_G, v);
  gyro_scale_ = dps * 35. / 1000000.;
  return true;
}

bool LSM6DSOX::set_accel_scale(accel_full_scale_t scale) {
  uint8_t v;
  CHECK_READ(LSM_ADDRESS, CTRL1_XL, &v, 1);
  v &= ~(3 << 2);  // clear bit2-3
  float g;
  switch (scale) {
    case ACCEL_FULL_SCALE_2G:  v |= 0 << 2; g =  2.f; break;  // 2g
    case ACCEL_FULL_SCALE_4G:  v |= 2 << 2; g =  4.f; break;  // 4g
    case ACCEL_FULL_SCALE_8G:  v |= 3 << 2; g =  8.f; break;  // 8g
    default:
    case ACCEL_FULL_SCALE_16G: v |= 1 << 2; g = 16.f; break;  // 16g
  }
  CHECK_WRITE(LSM_ADDRESS, CTRL1_XL, v);
  accel_scale_ = g / 32768.0;
  return true;
}

bool LSM6DSOX::accel(float values[3]) {
  if (!CheckStatusBit(1)) return 0.;
  if (!get_3f_le(LSM_ADDRESS, ACCEL_OUT, accel_scale_, values)) return false;
  for (int i : {0, 1, 2}) values[i] -= accel_scale_ * accel_bias_[i];
  return true;
}

bool LSM6DSOX::gyro(float values[3]) {
  if (!CheckStatusBit(2)) return 0.;
  if (!get_3f_le(LSM_ADDRESS, GYRO_OUT, gyro_scale_, values)) return false;
  for (int i : {0, 1, 2}) values[i] -= gyro_bias_[i];
  return true;
}

void LSM6DSOX::reset_bias() {
  for (int i : {0, 1, 2}) accel_bias_[i] = gyro_bias_[i] = 0.f;
}

void LSM6DSOX::set_bias(const float accel_bias[3], const float gyro_bias[3]) {
  for (int i : {0, 1, 2}) accel_bias_[i] = accel_bias[i];
  for (int i : {0, 1, 2}) gyro_bias_[i] = gyro_bias[i];
}

bool LSM6DSOX::calibrate(float accel_bias[3], float gyro_bias[3]) {
  return true;
}

////////////////////////////////////////////////////////////////////////////////

void LSM6DSOX::print() const {
  print3f("Gyro bias:  ", gyro_bias_);
  print3f("Accel bias: ", accel_bias_);
}

////////////////////////////////////////////////////////////////////////////////

}  // namespace skl
