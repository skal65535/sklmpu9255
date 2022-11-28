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
// Simple I2C Interface for MPU 9255 gyro and AK8963 magnetometer found
// in Waveshare products
//
//  https://stanford.edu/class/ee267/misc/MPU-9255-Datasheet.pdf
//  https://download.mikroe.com/documents/datasheets/ak8963c-datasheet.pdf
//
// Author: skal (pascal.massimino@gmail.com)
////////////////////////////////////////////////////////////////////////////////

#if !defined(SKLMPU9255_H_)
#define SKLMPU9255_H_

#include <cstdint>
#include <cassert>
#include <cstdio>

namespace skl {

////////////////////////////////////////////////////////////////////////////////
// I2C interface

extern bool I2C_init();
extern void I2C_close();
extern uint8_t I2C_read_byte(uint8_t dev_address, uint8_t reg_address);
extern bool I2C_read_bytes(uint8_t dev_address, uint8_t reg_address,
                           uint8_t values[], uint32_t len);
extern bool I2C_write_byte(uint8_t dev_address, uint8_t reg_address,
                           uint8_t value);

////////////////////////////////////////////////////////////////////////////////
// Gyro accel -> RPY filter

class QFilter {
 public:
  QFilter() {}
  void reset();
  void update(const float gyro[3], float dt);
  void updateAHRS(const float gyro[3], const float a[3], const float m[3],
                  float dt);
  void get_rpy(float rpy[3]) const;

 private:
  float q_[4] { 1.f, 0.f, 0.f, 0.f};   // quaternion
};

////////////////////////////////////////////////////////////////////////////////
// IMU interface

typedef enum { GYRO_FULL_SCALE_250DPS = 1,
               GYRO_FULL_SCALE_500DPS = 2,
               GYRO_FULL_SCALE_1000DPS = 3,
               GYRO_FULL_SCALE_2000DPS = 4
} gyro_full_scale_t;

typedef enum { ACCEL_FULL_SCALE_2G = 1,
               ACCEL_FULL_SCALE_4G = 2,
               ACCEL_FULL_SCALE_8G = 3,
               ACCEL_FULL_SCALE_16G = 4
} accel_full_scale_t;

class MPU {
 public:
  ~MPU() { I2C_close(); }

  bool init(bool ahrs = false, int average_size = 0 /*off*/);

  bool accel(float values[3]);  // in m/s^2
  bool gyro(float values[3]);   // in deg/s
  bool mag(float values[3]);    // in milliGauss
  float temperature();          // T in C

  void set_mag_scale(bool use_14bits);
  bool calibrate_mag(float bias[3], float scale[3], float nb_secs = 15.f);
  bool calibrate_mag(float nb_secs = 15.f) {
    return calibrate_mag(mag_bias_, mag_scales_, nb_secs);
  }
  void set_mag_bias_scale(float bias[3], float scale[3]);

  void set_gyro_scale(gyro_full_scale_t scale);
  void set_accel_scale(accel_full_scale_t scale);
  bool calibrate_gyro(float accel_bias[3],float gyro_bias[3]);
  bool calibrate_gyro() { return calibrate_gyro(accel_bias_, gyro_bias_); }
  void set_gyro_bias(float accel_bias[3], float gyro_bias[3]);

  bool get_rpy(float rpy[3]);
  void operator++();   // advance internal averaging counter

  void print() const;

 private:
  static const int kMaxAvg = 32;
  // store instant value v[] and compute average
  void get_average(float buf[][kMaxAvg], float v[3]);
  // just compute the average value so far
  void compute_average(const float buf[][kMaxAvg], float v[3]) const;

  bool self_test() const;
  bool init_accel_gyro();
  bool init_mag();

  float gyro_scale_;
  float gyro_bias_[3] = { 0.f, 0.f, 0.f };

  float accel_scale_;
  float accel_bias_[3] = { 0.f, 0.f, 0.f };

  float mag_scale_;
  float mag_bias_f_[3];   // magnetometer factory bias
  float mag_bias_[3] = { 0.f, 0.f, 0.f };     // in 16bits precision
  float mag_scales_[3] = { 1.f, 1.f, 1.f };

  QFilter q_filter_;

  int avg_size_ = 0;
  int avg_pos_ = 0;
  float avg_rpy_[9][kMaxAvg];   // gyro, accel, mag
  bool ahrs_ = false;
};

}  // namespace skl

////////////////////////////////////////////////////////////////////////////////

#endif  // SKLMPU9255_H_
