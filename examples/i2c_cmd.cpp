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
// simple tool to issue I2C commands
//
// Author: skal (pascal.massimino@gmail.com)
////////////////////////////////////////////////////////////////////////////////

#include "sklmpu9255.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

using namespace skl;

////////////////////////////////////////////////////////////////////////////////

#define LOG(...) fprintf(stderr, ## __VA_ARGS__)

#define CHECK_CALL(CALL, ERR) do {              \
  const bool ok = (CALL);                       \
  if (!ok) {                                    \
    LOG(#CALL " call failed.");                 \
    I2C_close();                                \
    return (ERR);                               \
  }                                             \
  LOG("Call to " #CALL " was successful\n");    \
} while (false)

#define CHECK_TRUE(CALL, ERR) do {              \
  const bool ok = (CALL);                       \
  if (!ok) {                                    \
    LOG("Invalid check: " #CALL "\n");          \
    I2C_close();                                \
    return (ERR);                               \
  }                                             \
} while (false)

static uint8_t ParseByte(const char str[]) {
  const long v = strtol(str, NULL, 0);
  if (v < 0 || v > 255) {
    LOG("Hex string [%s] can't be parsed as a byte.\n", str);
    exit(1);
  }
  return (uint8_t)v;
}

static const char* kBool[2] = {"false", "true"};

////////////////////////////////////////////////////////////////////////////////

int main(int argc, const char* argv[]) {
  CHECK_CALL(I2C_init(), 1);
  LOG("-------------------------------------------------------\n\n");
  for (int c = 1; c < argc; ++c) {
    if (!strcmp(argv[c], "-reset")) {
      CHECK_CALL(I2C_reset_device(), 1);
    } else if (!strcmp(argv[c], "-r") || !strcmp(argv[c], "-read")) {
      CHECK_TRUE(c + 2 < argc, 2);
      const uint8_t dev = ParseByte(argv[++c]);
      const uint8_t reg = ParseByte(argv[++c]);
      const uint8_t v = I2C_read_byte(dev, reg);
      LOG("I2C_read_byte(0x%.2x, 0x%.2x) = 0x%.2x\n", dev, reg, v);
    } else if (!strcmp(argv[c], "-p") || !strcmp(argv[c], "-print")) {
      I2C_print();
      LOG("-------------------------------------------------------\n\n");
    } else if (!strcmp(argv[c], "-w") || !strcmp(argv[c], "-write")) {
      CHECK_TRUE(c + 3 < argc, 3);
      const uint8_t dev = ParseByte(argv[++c]);
      const uint8_t reg = ParseByte(argv[++c]);
      const uint8_t val = ParseByte(argv[++c]);
      const bool ok = I2C_write_byte(dev, reg, val);
      LOG("I2C_write_byte(0x%.2x, 0x%.2x, 0x%.2x) = %s\n",
          dev, reg, val, kBool[ok]);
    } else if (!strcmp(argv[c], "-c") || !strcmp(argv[c], "-connected")) {
      CHECK_TRUE(c + 1 < argc, 2);
      const uint8_t dev = ParseByte(argv[++c]);
      const bool ok = I2C_is_connected(dev);
      LOG("I2C_is_connected(0x%.2x) = %s\n", dev, kBool[ok]);
    } else if (!strcmp(argv[c], "-map")) {
      LOG("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
      for (uint32_t address = 0x00; address <= 0x77; ++address) {
        if ((address & 0xf) == 0) LOG("\n%02x:", address);
        if (I2C_is_connected(address)) LOG(" %02x", address);
        else LOG(" --");
      }
      LOG("\n");
    } else if (!strcmp(argv[c], "-h") || !strcmp(argv[c], "-help")) {
      LOG("i2c_cmd: debugging tool to issue multiple I2C commands.\n");
      LOG("options:\n");
      LOG(" -reset .............. Reset the i2c device\n");
      LOG(" -r dev reg .......... Read register #reg on device #dev\n");
      LOG(" -w dev reg value .... "
          "Write #value on register #reg of device #dev\n");
      LOG(" -c dev .............. Calls 'is_connected' on device #dev\n");
      LOG(" -map ................ map the addresses, detecting devices\n");
      LOG(" -p .................. print I2C device info\n");
      I2C_close();
      return 0;
    } else {
      LOG("Unknown option [%s]\n\n", argv[c]);
      I2C_close();
      return 1;
    }
  }
  LOG("\n");
  I2C_close();
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
