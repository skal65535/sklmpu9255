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
// I2C Interface for MPU 9255 gyro and AK8963 magnetometer
//
// Author: skal (pascal.massimino@gmail.com)
////////////////////////////////////////////////////////////////////////////////

#include "sklmpu9255.h"
#include "internal.h"

#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>
#include <cassert>
#include <ctime>
#include <cstring>
#include <algorithm>

#define TEMPERATURE_OUT         0x41   // 0x41/0x42: OUT_H/L

namespace skl {

////////////////////////////////////////////////////////////////////////////////

bool MPU::init(float calibration_secs, bool ahrs, int averaging_size) {
  if (!I2C_init()) return false;

  // self tests
  if (!self_test()) return false;

  if (!mpu925x_.init()) return false;
  if (!ak8963_.init()) return false;

  if (calibration_secs > 0.f) {
    if (!mpu925x_.calibrate() ||
        !ak8963_.calibrate(calibration_secs)) {
      return false;
    }
  }

  ahrs_ = ahrs;
  avg_mag_.init(averaging_size, 0.f);
  avg_accel_.init(averaging_size, 0.f);
  avg_gyro_.init(averaging_size, 0.f);
  q_filter_.reset();

  return true;
}

void MPU::set_full_scales(MPU925x::accel_full_scale_t accel_scale,
                          MPU925x::gyro_full_scale_t gyro_scale) {
  mpu925x_.set_accel_scale(accel_scale);
  mpu925x_.set_gyro_scale(gyro_scale);
}

void MPU::print() const {
  I2C_print();
  mpu925x_.print();
  ak8963_.print();
}

bool MPU::self_test() const {
  const uint8_t mpu_id = mpu925x_.id();
  const uint8_t mag_id = ak8963_.id();
  fprintf(stderr, "WHO AM I: mpu 0x%.2x [%s], mag = 0x%.2x [%s]\n",
          mpu_id, mpu_id == 0x71 ? "MPU9250" : mpu_id == 0x73 ? "MPU9255" :
                  mpu_id == 0x70 ? "MPU6500" : "MPU????",
          mag_id, mag_id == 0x48 ? "AK8963" : "??????");
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool MPU::accel(float values[3]) {
  return (mpu925x_.accel(values) && avg_accel_.store(values));
}
bool MPU::gyro(float values[3]) {
  return (mpu925x_.gyro(values) && avg_gyro_.store(values));
}
bool MPU::mag(float values[3]) {
  return (ak8963_.mag(values) && avg_mag_.store(values));
}

float MPU::temperature() {
  uint8_t tmp[2];
  if (!I2C_read_bytes(MPU_ADDRESS, TEMPERATURE_OUT, tmp, 2)) return 0.;
  return get_16s(tmp) / 333.87 + 21.0;
}

////////////////////////////////////////////////////////////////////////////////

bool MPU::get_rpy(float rpy[3]) {
  const float dt = 0.005f;
  float v[3], a[3], m[3];
  if (!gyro(v)) return false;
  if (ahrs_ && accel(a) && mag(m)) {
    q_filter_.updateAHRS(v, a, m, dt);
  } else {
    q_filter_.update(v, dt);
  }
  q_filter_.get_rpy(rpy);
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// AveragingBuffer

void AveragingBuffer::init(int averaging_size, float v0) {
  pos_ = 0;
  max_pos_ = std::max(1, std::min(averaging_size, kMaxAvg));
  for (int i = 0; i < kMaxAvg; ++i) buf_[0][i] = buf_[1][i] = buf_[2][i] = v0;
}

bool AveragingBuffer::store(float v[3]) {
  if (max_pos_ == 0) return true;
  for (int i : {0, 1, 2}) buf_[i][pos_] = v[i];
  if (max_pos_ > 0 && ++pos_ == max_pos_) pos_ = 0;
  get(v);
  return true;
}

void AveragingBuffer::get(float v[3]) const {
  assert(max_pos_ > 0);
  for (int i : {0, 1, 2}) {
    float acc = 0.;
    for (int j = 0; j < max_pos_; ++j) acc += buf_[i][j];
    v[i] = acc / max_pos_;
  }
}

////////////////////////////////////////////////////////////////////////////////

}  // namespace skl
