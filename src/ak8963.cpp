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
// AK8963 magnetometer
//  https://stanford.edu/class/ee267/misc/MPU-9255-Datasheet.pdf
//
// Author: skal (pascal.massimino@gmail.com)
////////////////////////////////////////////////////////////////////////////////

#include "sklmpu9255.h"

#include "internal.h"

#include <unistd.h>
#include <algorithm>

namespace skl {

////////////////////////////////////////////////////////////////////////////////
// register map

// AK8963 magnetometer  (cf table 8.1)
#define MAG_WHO_AM_I            0x00   // WIA should return 0x48
#define MAG_ST1                 0x02   // data ready status bit 0
#define MAG_OUT                 0x03   // 0x03 -> 0x08 : XOUT_L/H YOUT_L/H ZOUT_L/H
#define MAG_ST2                 0x09   // bit3: data overflow, bit2: error stat
#define MAG_CNTL1               0x0a   // bit 0-3: MAG_MODE (0x02=8Hz, 0x06=100Hz continuous read)
#define MAG_ASTC                0x0c   // self-test
#define MAG_ASA                 0x10   // fuse ROM axis sensitivity adjustment (0x10->0x12)

#define MAG_ADDRESS             0x0c    // AK8963 magnetometer

#define MAG_MODE                0x06    // continuous read @ 100Hz

////////////////////////////////////////////////////////////////////////////////

uint8_t AK8963::address() { return MAG_ADDRESS; }
uint8_t AK8963::id() { return I2C_read_byte(MAG_ADDRESS, MAG_WHO_AM_I); }

bool AK8963::init() {
  if (!I2C_is_connected(address())) return false;
  const uint8_t my_id = id();
  if (my_id != WAI()) return false;
  LOG_MSG("WHO AM I: mag = 0x%.2x [%s]\n", my_id,
          my_id == 0x48 ? "AK8963" : "??????");

  I2C_write_byte(MAG_ADDRESS, MAG_CNTL1, 0x00);  // power down
  usleep(10 * 1000);
  I2C_write_byte(MAG_ADDRESS, MAG_CNTL1, 0x0f);  // enter fuse ROM access mode
  usleep(10 * 1000);
  uint8_t values[3];
  I2C_read_bytes(MAG_ADDRESS, MAG_ASA, values, 3);
  mag_asa_f_[0] = 1.f + (values[0] - 128.) / 256.;
  mag_asa_f_[1] = 1.f + (values[1] - 128.) / 256.;
  mag_asa_f_[2] = 1.f + (values[2] - 128.) / 256.;
  I2C_write_byte(MAG_ADDRESS, MAG_CNTL1, 0x00);  // power down
  usleep(10 * 1000);
  // bit 0-3: MAG_MODE -> 0x02=8Hz, 0x06=100Hz continuous read
  // bit4: 0 = M14BITS, 1 = M16BITS
  const bool use_14b = false;
  I2C_write_byte(MAG_ADDRESS, MAG_CNTL1, (use_14b ? 0x00 : 0x10) | MAG_MODE);
  usleep(10 * 1000);
  // 1 milliGauss = 10 micro-Tesla.
  mag_scale_ = 10. * 4912. / (use_14b ? 8190. : 32760.);
  return true;
}

void AK8963::close() {
  I2C_write_byte(MAG_ADDRESS, MAG_CNTL1, 0x00);  // power down
}

bool AK8963::mag(float values[3]) {
  const uint8_t st1 = I2C_read_byte(MAG_ADDRESS, MAG_ST1);
  if (!(st1 & 1)) return false;  // DRDY: data not ready
  uint8_t tmp[7];
  // this also read ST2 as tmp[7] at the end:
  if (!I2C_read_bytes(MAG_ADDRESS, MAG_OUT, tmp, 7)) return false;
  if (MAG_MODE == 0x02 || MAG_MODE == 0x04 || MAG_MODE == 0x06) {
    // if (st1 & 2) return false;   // DOR: Data overrun
  }
  const uint8_t st2 = tmp[6];
  if (st2 & 8) return false;  // HOFL: measurement overflow
  for (int i : {0, 1, 2}) {
    const float v = get_16s_le(tmp + 2 * i) * mag_scale_ * mag_asa_f_[i];
    values[i] = (v  - mag_bias_[i]) * mag_scales_[i];
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////

void AK8963::reset_bias_scale() {
  for (int i : {0, 1, 2}) mag_bias_[i] = 0.;
  for (int i : {0, 1, 2}) mag_scales_[i] = 1.;
}

void AK8963::set_bias_scale(const float bias[3], const float scale[3]) {
  for (int i : {0, 1, 2}) mag_bias_[i] = bias[i];
  for (int i : {0, 1, 2}) mag_scales_[i] = scale[i];
}

bool AK8963::calibrate(float bias[3], float scale[3], float nb_secs) {
  if (nb_secs <= 0.f) {
    reset_bias_scale();
    return true;
  }
  float btmp[3] = {0., 0., 0.}, stmp[3] = {0., 0., 0.};
  float m_max[3] = {-1e38f, -1e38f, -1e38f};
  float m_min[3] = { 1e38f,  1e38f,  1e38f};
  const int size = (int)(nb_secs * 100);  // @100Hz, that's ~15s of measurement
  for (int i = 0; i < size; ++i) {
    float tmp[3];
    if (mag(tmp)) {
      for (int j : {0, 1, 2}) {
        m_max[j] = std::max(m_max[j], tmp[j]);
        m_min[j] = std::min(m_min[j], tmp[j]);
      }
      usleep(12 * 1000);
    }
  }
  for (int j : {0, 1, 2}) {
    bias[j]  = (m_max[j] + m_min[j]) * 0.5;
    scale[j] = (m_max[j] - m_min[j]) * 0.5;
  }
  const float avg_scale = (scale[0] + scale[1] + scale[2]) / 3.f;
  for (int j : {0, 1, 2}) scale[j] = avg_scale / scale[j];

  return true;
}

////////////////////////////////////////////////////////////////////////////////

void AK8963::print() const {
  print3f("Mag bias:   ", mag_bias_);
  print3f("Mag scales: ", mag_scales_);
  print3f("Mag axis sensity adjustment factors: ", mag_asa_f_);
}

////////////////////////////////////////////////////////////////////////////////

}  // namespace skl
