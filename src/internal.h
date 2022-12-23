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
// some common internal functions
//
// Author: skal (pascal.massimino@gmail.com)
////////////////////////////////////////////////////////////////////////////////

#if !defined(SKL_MPU9255_INTERNAL_H_)
#define SKL_MPU9255_INTERNAL_H_

#include <stdint.h>
#include <stdio.h>

#define DEBUG_INTERNAL

namespace skl {

static inline int16_t get_16s(const uint8_t buf[2]) {
  return (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}
static inline int16_t get_16s_le(const uint8_t buf[2]) {
  return (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
}

static inline void get_3f(const uint8_t buf[6], float scale, float values[3]) {
  values[0] = scale * get_16s(buf + 0);
  values[1] = scale * get_16s(buf + 2);
  values[2] = scale * get_16s(buf + 4);
}

static inline bool get_3f(uint32_t device, uint8_t reg, float scale,
                          float values[3]) {
  uint8_t tmp[6];
  if (!I2C_read_bytes(device, reg, tmp, 6)) return false;
  get_3f(tmp, scale, values);
  return true;
}

static inline void print3f(const char msg[], const float v[3]) {
  fprintf(stderr, "%s%.3f %.3f %.3f\n", msg, v[0], v[1], v[2]);
}

#if defined(DEBUG_INTERNAL)
#define LOG_MSG(fmt, ...) do {    \
  fprintf(stderr, "[%s : %d] ", __FILE__, __LINE__); \
  fprintf(stderr, fmt, ## __VA_ARGS__); } while (false)
#else
#define LOG_MSG(fmt, ...) ((void*)fmt)
#endif

}  // namespace skl

////////////////////////////////////////////////////////////////////////////////

#endif  // SKL_MPU9255_INTERNAL_H_
