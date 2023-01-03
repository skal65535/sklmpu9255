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
// Quaternion filter
//
// Author: skal (pascal.massimino@gmail.com)
////////////////////////////////////////////////////////////////////////////////

#include "sklmpu9255.h"

#include <math.h>
#include <unistd.h>
#include <algorithm>

namespace skl {

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
  const float a22 = qw * qw + qx * qx - qy * qy - qz * qz;
  rpy[2] = fmod360(atan2f(a12, a22));   // magnetic declination?
}

}  // namespace skl
