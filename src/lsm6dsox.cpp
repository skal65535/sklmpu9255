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
//  https://pdf1.alldatasheet.com/datasheet-pdf/view/1447356/STMICROELECTRONICS/LSM6DSOX.html
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
#define CTRL1_XL                0x10
#define CTRL2_G                 0x11
#define CTRL3_C                 0x12
#define CTRL7_G                 0x16
#define CTRL8_XL                0x17
#define STATUS_REG              0X1e

#define TEMPERATURE_OUT         0x20   // 0x20 / 0x21

// accel / gyro
#define ACCEL_OUT               0x28   // 0x22 -> 0x2d : linear accel
#define GYRO_OUT                0x22   // 0x22 -> 0x27 : angular rate

////////////////////////////////////////////////////////////////////////////////

bool CheckStatusBit(uint8_t bit) {
  return true;
//  return ((I2C_read_byte(LSM_ADDRESS, STATUS_REG) & bit) != 0);
}


uint8_t LSM6DSOX::id() { return I2C_read_byte(LSM_ADDRESS, LSM_WHO_AM_I); }

bool LSM6DSOX::init() {
  // accel : 104Hz, 4G, bypass, ODR/4 low-pass filtering
  I2C_write_byte(LSM_ADDRESS, CTRL1_XL, 0x4a);
  // gyro : 104Hz, 2000dps, bypass
  I2C_write_byte(LSM_ADDRESS, CTRL2_G, 0x4c);
  // gyro is 'high-performance', 16MHz bandwidth
  I2C_write_byte(LSM_ADDRESS, CTRL7_G, 0x00);
  // ODR config: ODR/4
  I2C_write_byte(LSM_ADDRESS, CTRL8_XL, 0x09);

  return true;
}

void LSM6DSOX::close() {
  I2C_write_byte(LSM_ADDRESS, CTRL1_XL, 0x00);
  I2C_write_byte(LSM_ADDRESS, CTRL2_G, 0x00);
}

float LSM6DSOX::temperature() const {
  uint8_t tmp[2];
  if (!CheckStatusBit(4)) return 0.;
  if (!I2C_read_bytes(LSM_ADDRESS, TEMPERATURE_OUT, tmp, 2)) return 0.;
  return get_16s(tmp) / 256.0f + 25.0f;
}

void LSM6DSOX::set_gyro_scale(gyro_full_scale_t scale) {
  float dps;   // degree per seconds
  uint8_t v = I2C_read_byte(LSM_ADDRESS, CTRL2_G);
  v &= ~(7 << 1);  // clear bit 1,2,3
  switch (scale) {
    case GYRO_FULL_SCALE_125DPS:  v |= 1 << 1; dps =  125.f; break;
    case GYRO_FULL_SCALE_250DPS:  v |= 0 << 1; dps =  250.f; break;
    case GYRO_FULL_SCALE_500DPS:  v |= 2 << 1; dps =  500.f; break;
    case GYRO_FULL_SCALE_1000DPS: v |= 4 << 1; dps = 1000.f; break;
    case GYRO_FULL_SCALE_2000DPS: v |= 6 << 1; dps = 2000.f; break;
  }
  I2C_write_byte(LSM_ADDRESS, CTRL2_G, v);
  gyro_scale_ = dps / 32768.;
}

void LSM6DSOX::set_accel_scale(accel_full_scale_t scale) {
  float g;
  uint8_t v = I2C_read_byte(LSM_ADDRESS, CTRL1_XL);
  v &= ~(3 << 2);  // clear bit 2, 3
  switch (scale) {
    case ACCEL_FULL_SCALE_2G:  v |= 0 << 2; g =  2.f; break;  // 2g
    case ACCEL_FULL_SCALE_4G:  v |= 2 << 2; g =  4.f; break;  // 4g
    case ACCEL_FULL_SCALE_8G:  v |= 3 << 2; g =  8.f; break;  // 8g
    case ACCEL_FULL_SCALE_16G: v |= 1 << 2; g = 16.f; break;  // 16g
  }
  I2C_write_byte(LSM_ADDRESS, CTRL1_XL, v);
  accel_scale_ = g / 32768.0;
}

bool LSM6DSOX::accel(float values[3]) {
  if (!CheckStatusBit(1)) return 0.;
  if (!get_3f(LSM_ADDRESS, ACCEL_OUT, accel_scale_, values)) return false;
  for (int i : {0, 1, 2}) values[i] -= accel_scale_ * accel_bias_[i];
  return true;
}

bool LSM6DSOX::gyro(float values[3]) {
  if (!CheckStatusBit(2)) return 0.;
  if (!get_3f(LSM_ADDRESS, GYRO_OUT, gyro_scale_, values)) return false;
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
#if 0
  // We use hardware calibration
  I2C_write_byte(LSM_ADDRESS, PWR_MGMT_1, 0x80);  // reset device (bit 7);
  usleep(100 * 1000);
  // select stable source time
  I2C_write_byte(LSM_ADDRESS, PWR_MGMT_1, 0x01);
  I2C_write_byte(LSM_ADDRESS, PWR_MGMT_2, 0x00);
  usleep(200 * 1000);
  // setup bias calculation
  I2C_write_byte(LSM_ADDRESS, INT_ENABLE, 0x00);    // disable all interrupts
  I2C_write_byte(LSM_ADDRESS, FIFO_EN,    0x00);    // disable FIFO
  I2C_write_byte(LSM_ADDRESS, PWR_MGMT_1, 0x00);    // turn clock on
  I2C_write_byte(LSM_ADDRESS, I2C_MST_CTRL, 0x00);  // disable I2C master
  I2C_write_byte(LSM_ADDRESS, USER_CTRL, 0x00);     // disable FIFO and I2C
  I2C_write_byte(LSM_ADDRESS, USER_CTRL, 0x0c);     // reset FIFO and DMP
  usleep(15 * 1000);
  // configure gyro and accel bias recording
  I2C_write_byte(LSM_ADDRESS, MPU_CONFIG,   0x01);  // low-pass filter
  I2C_write_byte(LSM_ADDRESS, SMPLRT_DIV,   0x00);  // 1kHz sampling
  I2C_write_byte(LSM_ADDRESS, GYRO_CONFIG,  0x00);  // full scale 250dps
  I2C_write_byte(LSM_ADDRESS, ACCEL_CONFIG, 0x00);  // full scale 2G
  // configure FIFO
  I2C_write_byte(LSM_ADDRESS, USER_CTRL,    0x40);  // enable FIFO
  I2C_write_byte(LSM_ADDRESS, FIFO_EN,      0x78);  // enable gyro & accel
  usleep(40 * 1000);   // collect ~40 samples

  // extract bias now
  uint8_t tmp[12];
  I2C_write_byte(LSM_ADDRESS, FIFO_EN, 0x00);       // disable gyro & accel
  I2C_read_bytes(LSM_ADDRESS, FIFO_COUNT, tmp, 2);  // read FIFO count
  const int nb_samples = get_16s(tmp) / 12;
  for (int i : {0, 1, 2}) accel_bias[i] = gyro_bias[i] = 0.f;
  if (nb_samples > 0) {
    const float norm_g = 250.f / 32768. / nb_samples;  // 250dps scaling
    const float norm_a = 2.f / 32768. / nb_samples;    // 2G scaling
    for (int n = 0; n < nb_samples; ++n) {
      I2C_read_bytes(LSM_ADDRESS, FIFO_R_W, tmp, 12);
      float ab[3], gb[3];
      get_3f(tmp + 0, norm_a, ab);
      get_3f(tmp + 6, norm_g, gb);
      for (int i : {0, 1, 2}) accel_bias[i] += ab[i];
      for (int i : {0, 1, 2}) gyro_bias[i] += gb[i];
    }
    accel_bias[2] -= (accel_bias[2] > 0.) ? 1.f : -1.f;   // subtract 1G
  }
  usleep(100 * 1000);
#endif

  init();    // and reset all!
  usleep(500 * 1000);
  return true;
}

////////////////////////////////////////////////////////////////////////////////

void LSM6DSOX::print() const {
  print3f("Gyro bias:  ", gyro_bias_);
  print3f("Accel bias: ", accel_bias_);
}

////////////////////////////////////////////////////////////////////////////////

}  // namespace skl
