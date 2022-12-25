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
// simple tests of the I2C interface
//
// Author: skal (pascal.massimino@gmail.com)
////////////////////////////////////////////////////////////////////////////////

#include "sklmpu9255.h"
#include "gtest/gtest.h"

using namespace skl;

////////////////////////////////////////////////////////////////////////////////

TEST(Map, TestI2C) {
  EXPECT_TRUE(I2C_init());
  EXPECT_TRUE(I2C_reset_device());
  EXPECT_TRUE(I2C_init());
  printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
  for (uint32_t address = 0x00; address <= 0x77; ++address) {
    if ((address & 0xf) == 0) printf("\n%02x:", address);
    if (I2C_is_connected(address)) printf(" %02x", address);
    else printf(" --");
  }
  printf("\n");
  I2C_close();
}

////////////////////////////////////////////////////////////////////////////////

TEST(WAI, TestI2C) {
  EXPECT_TRUE(I2C_init());
  I2C_print();
  uint8_t WAI_1, WAI_2, WAI_3;
  EXPECT_TRUE(I2C_read_bytes(0x68 /* MPU_ADDRESS */, 0x75, &WAI_1, 1));
  printf("WAI_1 = 0x%.2x\n", WAI_1);
  EXPECT_TRUE(WAI_1 == 0x70 || WAI_1 == 0x71 || WAI_1 == 0x73);

  EXPECT_TRUE(I2C_read_bytes(0x0c /* MAG_ADDRESS */, 0x00, &WAI_2, 1));
  printf("WAI_2 = 0x%.2x\n", WAI_2);
  EXPECT_EQ(WAI_2, 0x48);

  EXPECT_TRUE(I2C_read_bytes(0x6a /* LSM_ADDRESS */, 0x0f, &WAI_3, 1));
  printf("WAI_3 = 0x%.2x\n", WAI_3);
  EXPECT_EQ(WAI_3, 0x6c);

  I2C_close();
  I2C_print();
}

////////////////////////////////////////////////////////////////////////////////
