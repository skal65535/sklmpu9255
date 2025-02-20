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

namespace skl {

////////////////////////////////////////////////////////////////////////////////

bool MPU::init(float calibration_secs, bool ahrs, int averaging_size) {
  ahrs_ = ahrs;
  avg_mag_.init(averaging_size, 0.f);
  avg_accel_.init(averaging_size, 0.f);
  avg_gyro_.init(averaging_size, 0.f);
  q_filter_.reset();

  if (!I2C_init()) return false;

  imu_ok_ = mpu925x_.init();
  mag_ok_ = ak8963_.init();
  LOG_MSG("IMU OK: %d     MAG OK: %d\n", imu_ok_, mag_ok_);

  if (calibration_secs > 0.f) {
    if (imu_ok_) {
      imu_ok_ = mpu925x_.calibrate();
      if (!imu_ok_) return false;
    }
    if (mag_ok_) {
      mag_ok_ = ak8963_.calibrate(calibration_secs);
      if (!mag_ok_) return false;
    }
  }

  return true;
}

bool MPU::close() {
  if (imu_ok_) {
    mpu925x_.close();
    imu_ok_ = false;
  }
  if (mag_ok_) {
    ak8963_.close();
    mag_ok_ = false;
  }
  return I2C_close();
}

bool MPU::set_full_scales(accel_full_scale_t accel_scale,
                          gyro_full_scale_t gyro_scale) {
  if (imu_ok_) {
    CHECK_OK(mpu925x_.set_accel_scale(accel_scale));
    CHECK_OK(mpu925x_.set_gyro_scale(gyro_scale));
  }
  return true;
}

void MPU::print() const {
  I2C_print();
  if (imu_ok_) mpu925x_.print();
  if (mag_ok_) ak8963_.print();
}

////////////////////////////////////////////////////////////////////////////////

bool MPU::accel(float values[3]) {
  return imu_ok_ && mpu925x_.accel(values) && avg_accel_.store(values);
}
bool MPU::gyro(float values[3]) {
  return imu_ok_ && mpu925x_.gyro(values) && avg_gyro_.store(values);
}
bool MPU::mag(float values[3]) {
  return mag_ok_ && ak8963_.mag(values) && avg_mag_.store(values);
}
float MPU::temperature() {
  return imu_ok_ ? mpu925x_.temperature() : -273.f;
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
  const int kMaxAvg = (int)AveragingBuffer::kMaxAvg;
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
