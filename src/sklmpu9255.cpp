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
// Simple I2C Interface for MPU 9255 gyro and AK8963 magnetometer
//
// Author: skal (pascal.massimino@gmail.com)
////////////////////////////////////////////////////////////////////////////////

#include "sklmpu9255.h"

#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>
#include <ctime>
#include <cmath>
#include <cstring>
#include <algorithm>

#if !defined(FAKE_I2C)
#include <linux/i2c-dev.h>
#else
#define I2C_SLAVE 0
#endif

namespace skl {

////////////////////////////////////////////////////////////////////////////////
// I2C I/O

static const uint8_t MAG_MODE = 0x06;   // continuous 100Hz

static const char kDevName[] = "/dev/i2c-1";
static int fd = -1;

static inline int16_t get_16s(const uint8_t buf[2]) {
  return (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

bool I2C_init(void) {
  if (fd < 0) fd = open(kDevName, O_RDWR);
  if (fd < 0) {
    fprintf(stderr, "Failed to open the i2c bus [%s].\n", kDevName);
    return false;
  }
  return true;
}

void I2C_close() {
  if (fd >= 0) {
    close(fd);
    fd = -1;
  }
}

uint8_t I2C_read_byte(uint8_t dev_address, uint8_t reg_address) {
  assert(fd >= 0);
  if (ioctl(fd, I2C_SLAVE, dev_address) < 0) {
    fprintf(stderr, "bus: failed to get READ access [dev:0x%.2x].\n",
            dev_address);
    return 0;
  }
  uint8_t value = 0;
  if (write(fd, &reg_address, 1) != 1 || read(fd, &value, 1) != 1) {
    fprintf(stderr, "Failed to read bus value [reg: 0x%.2x].\n", reg_address);
  }
  return value;
}

bool I2C_read_bytes(uint8_t dev_address, uint8_t reg_address,
                    uint8_t values[], uint32_t len) {
  assert(fd >= 0);
  if (ioctl(fd, I2C_SLAVE, dev_address) < 0) {
    fprintf(stderr, "bus: failed to get READ access [dev:0x%.2x].\n",
            dev_address);
    return false;
  }
  if (write(fd, &reg_address, 1) != 1 || read(fd, values, len) != len) {
    fprintf(stderr, "Failed to read %d values [reg: 0x%.2x].\n",
            len, reg_address);
    return false;
  }
  return true;
}

bool I2C_write_byte(uint8_t dev_address, uint8_t reg_address, uint8_t value) {
  assert(fd >= 0);
  if (ioctl(fd, I2C_SLAVE, dev_address) < 0) {
    fprintf(stderr, "bus: failed to get WRITE access [dev:0x%.2x].\n",
            dev_address);
    return false;
  }
  const uint8_t buf[2] = { reg_address, value };
  if (write(fd, buf, 2) != 2) {
    fprintf(stderr, "Failed to write bus value %u [reg:0x%.2x].\n",
            value, reg_address);
    return false;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// definitions of sensors adresses
#define MPU_ADDRESS             0x68    // I2C: default device adress
#define MPU_WHO_AM_I            0x75    // self-identify: 0x71, 0x73, 0x70
#define MAG_ADDRESS             0x0c    // AK8963 magnetometer
#define GYRO_ADDRESS            0xd0    // gyro address
#define ACCEL_ADDRESS           0xd0    // accel address
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
// AK8963 magnetometer
#define MAG_WHO_AM_I            0x00   // WIA should return 0x48
#define MAG_ST1                 0x02   // data ready status bit 0
#define MAG_OUT                 0x03   // 0x03 -> 0x08 : XOUT_L/H YOUT_L/H ZOUT_L/H
#define MAG_ST2                 0x09   // bit3: data overflow, bit2: error stat
#define MAG_CNTL                0x0a   // bit 0-3: MAG_MODE (0x02=8Hz, 0x06=100Hz continuous read)
#define MAG_ASAX                0x10   // fuse ROM axis sensitivity adjustment
#define MAG_ASAY                0x11
#define MAG_ASAZ                0x12

// accel
#define ACCEL_OUT               0x3b   // 0x3b -> 0x40 : XOUT_H/L YOUT_H/L ZOUT_H/L
#define TEMPERATURE_OUT         0x41   // 0x41/0x42: OUT_H/L
// gyro
#define GYRO_OUT                0x43   // 0x43 -> 0x48 : XOUT_H/L YOUT_H/L ZOUT_H/L

////////////////////////////////////////////////////////////////////////////////

bool MPU::init_accel_gyro() {    // Accel init
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
  d2 = (d2 & 0xf0) | (0x01 << 3) | 0x03;  // fchoise + dlpf 41Hz (3)
  I2C_write_byte(MPU_ADDRESS, ACCEL_CONFIG2, d2);

  I2C_write_byte(MPU_ADDRESS, INT_PIN_CFG, 0x22);// BYPASS ENABLE, LATCH_INT_EN
  I2C_write_byte(MPU_ADDRESS, INT_ENABLE, 0x01); // bit0: Enable data ready
  usleep(100 * 1000);
  return true;
}

bool MPU::init_mag() {  // Magnetometer init
  I2C_write_byte(MAG_ADDRESS, MAG_CNTL, 0x00);  // power down
  usleep(10 * 1000);
  I2C_write_byte(MAG_ADDRESS, MAG_CNTL, 0x0f);  // enter fuse ROM access mode
  usleep(10 * 1000);
  uint8_t values[3];
  I2C_read_bytes(MAG_ADDRESS, MAG_ASAX, values, 3);
  mag_bias_f_[0] = 1.f + (values[0] - 128.) / 256.;
  mag_bias_f_[1] = 1.f + (values[1] - 128.) / 256.;
  mag_bias_f_[2] = 1.f + (values[2] - 128.) / 256.;
  I2C_write_byte(MAG_ADDRESS, MAG_CNTL, 0x00);  // power down
  usleep(10 * 1000);
  // bit4: 0 = M14BITS, 1 = M16BITS
  // bit 0-3: MAG_MODE -> 0x02=8Hz, 0x06=100Hz continuous read

  I2C_write_byte(MAG_ADDRESS, MAG_CNTL, 0x10 | MAG_MODE);  // 16bit prec, 100hz
  usleep(10 * 1000);
  return true;
}

bool MPU::init(bool ahrs, int average_size) {
  if (!I2C_init()) return false;

  // self tests
  if (!self_test()) return false;

  if (!init_accel_gyro()) return false;
  if (!init_mag()) return false;

  // set default full scale range for gyro and accel
  set_gyro_scale(GYRO_FULL_SCALE_250DPS);
  set_accel_scale(ACCEL_FULL_SCALE_8G);

  // use 16bit prec for magnetometer
  set_mag_scale(false);

  ahrs_ = ahrs;
  avg_size_ = std::max(1, std::min(average_size, kMaxAvg));
  avg_pos_ = 0;
  memset(avg_rpy_, 0, sizeof(avg_rpy_));
  q_filter_.reset();

  return true;
}

static void print3f(const char msg[], const float v[3]) {
  fprintf(stderr, "%s%.3f %.3f %.3f\n", msg, v[0], v[1], v[2]);
}

void MPU::print() const {
  fprintf(stderr, "I2C OK [fd: %d  dev='%s'].\n", fd, kDevName);
  print3f("Gyro bias:  ", gyro_bias_);
  print3f("Accel bias: ", accel_bias_);
  print3f("Mag bias:   ", mag_bias_);
  print3f("Mag scales: ", mag_scales_);
  print3f("Mag factory bias: ", mag_bias_f_);
}

bool MPU::self_test() const {
  const uint8_t mpu_id = I2C_read_byte(MPU_ADDRESS, MPU_WHO_AM_I);
  const uint8_t mag_id = I2C_read_byte(MAG_ADDRESS, MAG_WHO_AM_I);
  fprintf(stderr, "WHO AM I: mpu 0x%.2x [%s], mag = 0x%.2x [%s]\n",
          mpu_id, mpu_id == 0x71 ? "MPU9250" : mpu_id == 0x73 ? "MPU9255" :
                  mpu_id == 0x70 ? "MPU6500" : "MPU????",
          mag_id, mag_id == 0x48 ? "AK8963" : "??????");
  return true;
}

void MPU::set_gyro_scale(gyro_full_scale_t scale) {
  float dps;   // degree per seconds
  uint8_t v = I2C_read_byte(MPU_ADDRESS, GYRO_CONFIG);
  v &= ~(3 << 3);  // clear bit 3 and 4
  switch (scale) {
    case GYRO_FULL_SCALE_250DPS:  v |= 0 << 3; dps =  250.f; break;
    case GYRO_FULL_SCALE_500DPS:  v |= 1 << 3; dps =  500.f; break;
    case GYRO_FULL_SCALE_1000DPS: v |= 2 << 3; dps = 1000.f; break;
    case GYRO_FULL_SCALE_2000DPS: v |= 3 << 3; dps = 2000.f; break;
  }
  I2C_write_byte(MPU_ADDRESS, GYRO_CONFIG, v);
  gyro_scale_ = dps / 32768.;
}

void MPU::set_accel_scale(accel_full_scale_t scale) {
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

void MPU::set_mag_scale(bool use_14bits) {
  mag_scale_ = 10. * 4912. / (use_14bits ? 8192. : 32768.);
}

static void get_3f(const uint8_t buf[6], float scale, float values[3]) {
  values[0] = scale * get_16s(buf + 0);
  values[1] = scale * get_16s(buf + 2);
  values[2] = scale * get_16s(buf + 4);
}

static bool get_3f(uint8_t reg, float scale, float values[3]) {
  uint8_t tmp[6];
  if (!I2C_read_bytes(MPU_ADDRESS, reg, tmp, 6)) return false;
  get_3f(tmp, scale, values);
  return true;
}

bool MPU::accel(float values[3]) {
  if (!get_3f(ACCEL_OUT, accel_scale_, values)) return false;
  if (avg_size_ > 0) get_average(avg_rpy_ + 3, values);
  for (int i : {0, 1, 2}) values[i] -= accel_scale_ * accel_bias_[i];
  return true;
}

bool MPU::gyro(float values[3]) {
  if (!get_3f(GYRO_OUT, gyro_scale_, values)) return false;
  if (avg_size_ > 0) get_average(avg_rpy_ + 0, values);
  for (int i : {0, 1, 2}) values[i] -= gyro_bias_[i];
  return true;
}

void MPU::set_gyro_bias(float accel_bias[3], float gyro_bias[3]) {
  for (int i : {0, 1, 2}) accel_bias_[i] = accel_bias[i];
  for (int i : {0, 1, 2}) gyro_bias_[i] = gyro_bias[i];
}

bool MPU::calibrate_gyro(float accel_bias[3], float gyro_bias[3]) {
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
    accel_bias[2] -= (accel_bias[2] > 0.) ? 1.f : 1.f;   // subtract 1G
  }
  
  usleep(100 * 1000);
  // and reset all!
  init_accel_gyro();
  usleep(500 * 1000);

  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool MPU::mag(float values[3]) {
  const uint8_t st1 = I2C_read_byte(MAG_ADDRESS, MAG_ST1);
  if (!(st1 & 1)) return false;  // data not ready (DRDY)
  uint8_t tmp[7];
  // this also read ST2 as tmp[7] at the end:
  if (!I2C_read_bytes(MAG_ADDRESS, MAG_OUT, tmp, 7)) return false;
  if (MAG_MODE == 0x02 || MAG_MODE == 0x04 || MAG_MODE == 0x06) {
    // if (st1 & 2) return false;   // DOR
  }
  const uint8_t st2 = tmp[6];
  if (st2 & 8) return false;  // data not ready
  get_3f(tmp, mag_scale_, values);
  if (avg_size_ > 0) get_average(avg_rpy_ + 6, values);
  for (int i : {0, 1, 2}) {
    values[i] = (values[i] * mag_bias_f_[i] - mag_bias_[i]) * mag_scales_[i];
  }
  return true;
}

void MPU::set_mag_bias_scale(float bias[3], float scale[3]) {
  for (int i : {0, 1, 2}) mag_bias_[i] = bias[i];
  for (int i : {0, 1, 2}) mag_scales_[i] = scale[i];
}

bool MPU::calibrate_mag(float bias[3], float scale[3], float nb_secs) {
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

void MPU::operator++() {
  if (avg_size_ > 0 && ++avg_pos_ == avg_size_) avg_pos_ = 0;
}

void MPU::get_average(float buf[][kMaxAvg], float v[3]) {
  if (avg_size_ == 0) return;
  for (int i : {0, 1, 2}) buf[i][avg_pos_] = v[i];
  compute_average(buf, v);
}

void MPU::compute_average(const float buf[][kMaxAvg], float v[3]) const {
  assert(avg_size_ > 0);
  for (int i : {0, 1, 2}) {
    float acc = 0.;
    for (int j = 0; j < avg_size_; ++j) acc += buf[i][j];
    v[i] = acc / avg_size_;
  }
}

////////////////////////////////////////////////////////////////////////////////
// QFilter

static const float kScale = 180. / 3.1415926535;
static float fmod360(float v) {
  return fmod(v * kScale + 360., 360.);
}

static void normalize4(float q[4]) {
  double n = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  if (n != 0.) {
    n = 1. / sqrt(n);
    q[0] *= n;
    q[1] *= n;
    q[2] *= n;
    q[3] *= n;
  }
}

static void normalize3(const float in[3], float out[3]) {
  double n = in[0] * in[0] + in[1] * in[1] + in[2] * in[2];
  if (n != 0.) {
    n = 1. / sqrt(n);
    out[0] = in[0] * n;
    out[1] = in[1] * n;
    out[2] = in[2] * n;
  }
}
static void cross(const float a[3], const float b[3], float c[3]) {
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

void QFilter::reset() {
  q_[0] = 1.f;
  q_[1] = q_[2] = q_[3] = 0.f;
}

void QFilter::update(const float gyro[3], float dt) {
  dt *= 0.5;
  const float gx = dt * gyro[0], gy = dt * gyro[1], gz = dt * gyro[2];
  const float qw = q_[0], qx = q_[1], qy = q_[2], qz = q_[3];
  q_[0] += -qx * gx - qy * gy - qz * gz;
  q_[1] +=  qw * gx - qz * gy + qy * gz;
  q_[2] +=  qz * gx + qw * gy - qx * gz;
  q_[3] += -qy * gx + qx * gy + qw * gz;
  normalize4(q_);
}

void QFilter::updateAHRS(const float gyro[3],
                         const float A[3], const float M[3], float dt) {
  const float qw = q_[0], qx = q_[1], qy = q_[2], qz = q_[3];
  float a[3], m[3];
  normalize3(A, a);
  normalize3(M, m);
  // reference flux
  float h[3];
  h[0] = m[0] + 2. * (m[0] * (-qy * qy - qz * qz) + m[1] * ( qx * qy - qw * qz) + m[2] * ( qx * qz + qw * qy));
  h[1] = m[1] + 2. * (m[0] * ( qx * qy + qw * qz) + m[1] * (-qx * qx - qz * qz) + m[2] * ( qy * qz - qw * qx));
  h[2] = m[2] + 2. * (m[0] * ( qx * qz - qw * qy) + m[1] * ( qy * qz + qw * qx) + m[2] * (-qx * qx - qy * qy));
  const float bx = sqrt(h[0] * h[0] + h[1] * h[1]);
  const float bz = h[2];

  // estimate gravity
  const float v[3] = { 2.f * (qx * qz - qw * qy),
                       2.f * (qw * qx + qy * qz),
                       qw * qw - qx * qx - qy * qy - qz * qz };
  // estimate flux
  const float w[3] = { bx + 2.f * (bx * (qy * qy + qz * qz) + bz * (qx * qy - qw * qy)),
                            2.f * (bx * (qx * qy - qw * qz) + bz * (qw * qx + qy * qz)),
                       bz + 2.f * (bx * (qw * qy + qx * qz) - bz * (qx * qx + qy * qy)) };

  // estimate error
  float err1[3], err2[3];
  cross(a, v, err1);
  cross(m, w, err2);
  err1[0] += err2[0];
  err1[1] += err2[1];
  err1[2] += err2[2];
  float g[3] = { gyro[0], gyro[1], gyro[2] };
  if (err1[0] != 0. && err1[1] != 0. && err1[2]) {  // correct gyro
    const float beta = .1;
    g[0] -= err1[0] * beta;
    g[1] -= err1[1] * beta;
    g[2] -= err1[2] * beta;
  }
  update(g, dt);
}

void QFilter::get_rpy(float rpy[3]) const {
  // Tait-Bryan (z-axis down)
  const float qw = q_[0], qx = q_[1], qy = q_[2], qz = q_[3];

  const float a31 = 2.f * (qw * qx + qy * qz);
  const float a33 = qw * qw - qx * qx - qy * qy + qz * qz;
  rpy[0] = fmod360(atan2f(a31, a33));

  const float a32 = 2.f * (qw * qy - qx * qz);
  rpy[1] = fmod360(asinf(a32));

  const float a12 = 2.f * (qx * qy + qw * qz);
  const float a22 = qw * qw - qx * qx - qy * qy + qz * qz;
  rpy[2] = fmod360(atan2f(a12, a22));   // magnetic declination?
}

////////////////////////////////////////////////////////////////////////////////

}  // namespace skl
