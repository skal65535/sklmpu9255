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
// MPU 925x gyro/accel
//  https://stanford.edu/class/ee267/misc/MPU-9255-Datasheet.pdf
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

#define MPU_ADDRESS             0x68    // I2C: default MPUxxx device address
#define MPU_WHO_AM_I            0x75    // self-identify: 0x71, 0x73, 0x70
// control registers
#define USER_CTRL               0x6a    // DMP: bit 7: enable, bit 3: reset
#define PWR_MGMT_1              0x6b
#define PWR_MGMT_2              0x6c
#define INT_PIN_CFG             0x37
#define ACCEL_CONFIG1           0x1c
#define ACCEL_CONFIG2           0x1d
#define MPU_CONFIG              0x1a
#define SMPLRT_DIV              0x19    // sample rate = 1000 / (1 + value) Hz
#define GYRO_CONFIG             0x1b
#define ACCEL_CONFIG            0x1c
#define FIFO_EN                 0x23
#define FIFO_COUNT              0x72
#define FIFO_R_W                0x74
#define I2C_MST_CTRL            0x24
#define INT_PIN_CFG             0x37
#define INT_ENABLE              0x38
#define TEMPERATURE_OUT         0x41   // 0x41/0x42: OUT_H/L

// accel
#define ACCEL_OUT               0x3b   // 0x3b -> 0x40 : XOUT_H/L YOUT_H/L ZOUT_H/L

// gyro
#define GYRO_OUT                0x43   // 0x43 -> 0x48 : XOUT_H/L YOUT_H/L ZOUT_H/L

////////////////////////////////////////////////////////////////////////////////

uint8_t MPU925x::id() { return I2C_read_byte(MPU_ADDRESS, MPU_WHO_AM_I); }

bool MPU925x::init() {
  I2C_write_byte(MPU_ADDRESS, PWR_MGMT_1, 0x80);  // reset device (bit 7)
  usleep(100 * 1000);
  I2C_write_byte(MPU_ADDRESS, PWR_MGMT_1, 0x00);  // enable all sensors
  usleep(100 * 1000);
  I2C_write_byte(MPU_ADDRESS, PWR_MGMT_1, 0x01);  // auto-select clock source
  usleep(100 * 1000);

  I2C_write_byte(MPU_ADDRESS, MPU_CONFIG, 3);  // DLPF: 41Hz
  I2C_write_byte(MPU_ADDRESS, SMPLRT_DIV, 4);  // Sample rate: 200Hz

  uint8_t c = I2C_read_byte(MPU_ADDRESS, GYRO_CONFIG);
  c = (c & 0x04) | 0x00;   // fchoice = 3
  I2C_write_byte(MPU_ADDRESS, GYRO_CONFIG, c);

  uint8_t d1 = I2C_read_byte(MPU_ADDRESS, ACCEL_CONFIG);
  d1 = (d1 & 0xf8);
  I2C_write_byte(MPU_ADDRESS, ACCEL_CONFIG, d1);

  uint8_t d2 = I2C_read_byte(MPU_ADDRESS, ACCEL_CONFIG2);
  d2 = (d2 & 0xf0) | (0x01 << 3) | 0x03;  // fchoice + DLPF 41Hz (3)
  I2C_write_byte(MPU_ADDRESS, ACCEL_CONFIG2, d2);

  I2C_write_byte(MPU_ADDRESS, INT_PIN_CFG, 0x22);// BYPASS ENABLE, LATCH_INT_EN
  I2C_write_byte(MPU_ADDRESS, INT_ENABLE, 0x01); // bit0: Enable data ready
  usleep(100 * 1000);

  // set default full scale range for gyro and accel
  set_gyro_scale(GYRO_FULL_SCALE_250DPS);
  set_accel_scale(ACCEL_FULL_SCALE_8G);
  return true;
}

void MPU925x::close() {
}

float MPU925x::temperature() const {
  uint8_t tmp[2];
  if (!I2C_read_bytes(MPU_ADDRESS, TEMPERATURE_OUT, tmp, 2)) return 0.;
  return get_16s(tmp) / 333.87 + 21.0;
}

void MPU925x::set_gyro_scale(gyro_full_scale_t scale) {
  float dps;   // degree per seconds
  uint8_t v = I2C_read_byte(MPU_ADDRESS, GYRO_CONFIG);
  v &= ~(3 << 3);  // clear bit 3 and 4
  switch (scale) {
    case GYRO_FULL_SCALE_125DPS:
    case GYRO_FULL_SCALE_250DPS:  v |= 0 << 3; dps =  250.f; break;
    case GYRO_FULL_SCALE_500DPS:  v |= 1 << 3; dps =  500.f; break;
    case GYRO_FULL_SCALE_1000DPS: v |= 2 << 3; dps = 1000.f; break;
    case GYRO_FULL_SCALE_2000DPS: v |= 3 << 3; dps = 2000.f; break;
  }
  I2C_write_byte(MPU_ADDRESS, GYRO_CONFIG, v);
  gyro_scale_ = dps / 32768.;
}

void MPU925x::set_accel_scale(accel_full_scale_t scale) {
  float g;
  uint8_t v = I2C_read_byte(MPU_ADDRESS, ACCEL_CONFIG1);
  v &= ~(3 << 3);  // clear bit 3 and 4
  switch (scale) {
    case ACCEL_FULL_SCALE_2G:  v |= 0 << 3; g =  2.f; break;  // 2g
    case ACCEL_FULL_SCALE_4G:  v |= 1 << 3; g =  4.f; break;  // 4g
    case ACCEL_FULL_SCALE_8G:  v |= 2 << 3; g =  8.f; break;  // 8g
    case ACCEL_FULL_SCALE_16G: v |= 3 << 3; g = 16.f; break;  // 16g
  }
  I2C_write_byte(MPU_ADDRESS, ACCEL_CONFIG1, v);
  accel_scale_ = g / 32768.0;
}

bool MPU925x::accel(float values[3]) {
  if (!get_3f(MPU_ADDRESS, ACCEL_OUT, accel_scale_, values)) return false;
  for (int i : {0, 1, 2}) values[i] -= accel_scale_ * accel_bias_[i];
  return true;
}

bool MPU925x::gyro(float values[3]) {
  if (!get_3f(MPU_ADDRESS, GYRO_OUT, gyro_scale_, values)) return false;
  for (int i : {0, 1, 2}) values[i] -= gyro_bias_[i];
  return true;
}

void MPU925x::reset_bias() {
  for (int i : {0, 1, 2}) accel_bias_[i] = gyro_bias_[i] = 0.f;
}

void MPU925x::set_bias(const float accel_bias[3], const float gyro_bias[3]) {
  for (int i : {0, 1, 2}) accel_bias_[i] = accel_bias[i];
  for (int i : {0, 1, 2}) gyro_bias_[i] = gyro_bias[i];
}

bool MPU925x::calibrate(float accel_bias[3], float gyro_bias[3]) {
  // We use hardware calibration
  I2C_write_byte(MPU_ADDRESS, PWR_MGMT_1, 0x80);  // reset device (bit 7);
  usleep(100 * 1000);
  // select stable source time
  I2C_write_byte(MPU_ADDRESS, PWR_MGMT_1, 0x01);
  I2C_write_byte(MPU_ADDRESS, PWR_MGMT_2, 0x00);
  usleep(200 * 1000);
  // setup bias calculation
  I2C_write_byte(MPU_ADDRESS, INT_ENABLE, 0x00);    // disable all interrupts
  I2C_write_byte(MPU_ADDRESS, FIFO_EN,    0x00);    // disable FIFO
  I2C_write_byte(MPU_ADDRESS, PWR_MGMT_1, 0x00);    // turn clock on
  I2C_write_byte(MPU_ADDRESS, I2C_MST_CTRL, 0x00);  // disable I2C master
  I2C_write_byte(MPU_ADDRESS, USER_CTRL, 0x00);     // disable FIFO and I2C
  I2C_write_byte(MPU_ADDRESS, USER_CTRL, 0x0c);     // reset FIFO and DMP
  usleep(15 * 1000);
  // configure gyro and accel bias recording
  I2C_write_byte(MPU_ADDRESS, MPU_CONFIG,   0x01);  // low-pass filter
  I2C_write_byte(MPU_ADDRESS, SMPLRT_DIV,   0x00);  // 1kHz sampling
  I2C_write_byte(MPU_ADDRESS, GYRO_CONFIG,  0x00);  // full scale 250dps
  I2C_write_byte(MPU_ADDRESS, ACCEL_CONFIG, 0x00);  // full scale 2G
  // configure FIFO
  I2C_write_byte(MPU_ADDRESS, USER_CTRL,    0x40);  // enable FIFO
  I2C_write_byte(MPU_ADDRESS, FIFO_EN,      0x78);  // enable gyro & accel
  usleep(40 * 1000);   // collect ~40 samples

  // extract bias now
  uint8_t tmp[12];
  I2C_write_byte(MPU_ADDRESS, FIFO_EN, 0x00);       // disable gyro & accel
  I2C_read_bytes(MPU_ADDRESS, FIFO_COUNT, tmp, 2);  // read FIFO count
  const int nb_samples = get_16s(tmp) / 12;
  for (int i : {0, 1, 2}) accel_bias[i] = gyro_bias[i] = 0.f;
  if (nb_samples > 0) {
    const float norm_g = 250.f / 32768. / nb_samples;  // 250dps scaling
    const float norm_a = 2.f / 32768. / nb_samples;    // 2G scaling
    for (int n = 0; n < nb_samples; ++n) {
      I2C_read_bytes(MPU_ADDRESS, FIFO_R_W, tmp, 12);
      float ab[3], gb[3];
      get_3f(tmp + 0, norm_a, ab);
      get_3f(tmp + 6, norm_g, gb);
      for (int i : {0, 1, 2}) accel_bias[i] += ab[i];
      for (int i : {0, 1, 2}) gyro_bias[i] += gb[i];
    }
    accel_bias[2] -= (accel_bias[2] > 0.) ? 1.f : -1.f;   // subtract 1G
  }
  usleep(100 * 1000);

  init();    // and reset all!
  usleep(500 * 1000);

  return true;
}

////////////////////////////////////////////////////////////////////////////////

void MPU925x::print() const {
  print3f("Gyro bias:  ", gyro_bias_);
  print3f("Accel bias: ", accel_bias_);
}

////////////////////////////////////////////////////////////////////////////////

}  // namespace skl
