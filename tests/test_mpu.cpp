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
// simple tests for the API
//
// Author: skal (pascal.massimino@gmail.com)
////////////////////////////////////////////////////////////////////////////////

#include "sklmpu9255.h"
#include "gtest/gtest.h"

////////////////////////////////////////////////////////////////////////////////

TEST(Basic, TestI2C) {
  EXPECT_TRUE(skl::I2C_init());
  skl::I2C_close();
}

TEST(Basic, TestMPU) {
  const int cnt = 3;
  skl::MPU mpu;
  ASSERT_TRUE(mpu.init(true, 16)) << "Init failed.";

  for (uint32_t n = 0; n < cnt; ++n) {
    int16_t v[3];
    float vf[3];

    EXPECT_TRUE(mpu.accel(vf));
    printf("Acc:  \t%.3f \t%.3f \t%.3f\n", vf[0], vf[1], vf[2]);

    EXPECT_TRUE(mpu.gyro(vf));
    printf("gyro: \t%.3f \t%.3f \t%.3f\n", vf[0], vf[1], vf[2]);

    EXPECT_TRUE(mpu.mag(vf));
    printf("mag:  \t%.3f \t%.3f \t%.3f\n", vf[0], vf[1], vf[2]);

    const float T = mpu.temperature();
    printf("Temperature: %.2f\n", T);

    if (n == 20) mpu.set_full_scales(skl::MPU925x::ACCEL_FULL_SCALE_4G,
                                     skl::MPU925x::GYRO_FULL_SCALE_500DPS);
    if (n == 40) mpu.set_full_scales(skl::MPU925x::ACCEL_FULL_SCALE_8G,
                                     skl::MPU925x::GYRO_FULL_SCALE_2000DPS);
  }
}

TEST(Filter1, TestMPU) {
  const int cnt = 10;
  skl::MPU mpu;
  ASSERT_TRUE(mpu.init());
  for (uint32_t n = 0; n < cnt; ++n) {
    float rpy[3];
    ASSERT_TRUE(mpu.get_rpy(rpy));
    printf("%d %.3f  %.3f  %.3f\n", n, rpy[0], rpy[1], rpy[2]);
  }
}

TEST(Filter2, TestMPU) {
  const int cnt = 10;
  skl::MPU mpu;
  ASSERT_TRUE(mpu.init());
  for (uint32_t n = 0; n < cnt; ++n) {
    float gyro[3];
    ASSERT_TRUE(mpu.gyro(gyro));
    printf("%d %.3f  %.3f  %.3f\n", n, gyro[0], gyro[1], gyro[2]);
  }
}

////////////////////////////////////////////////////////////////////////////////
