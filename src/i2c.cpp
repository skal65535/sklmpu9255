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
// Simple I2C Interface (w/ or w/o MCP2221a)
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

#if defined(USE_HID)
#include "hidapi.h"
typedef struct hid_device_info hid_device_info;
#include <vector>
#include <string>
using std::vector;
using std::string;
#elif defined(FAKE_I2C)
#define I2C_SLAVE 0
#else
#include <linux/i2c-dev.h>
#endif

namespace skl {

////////////////////////////////////////////////////////////////////////////////
// I2C I/O

#if defined(USE_HID)

// I2C interface using MCP2221 USB<->I2C microcontroller
//   http://ww1.microchip.com/downloads/en/devicedoc/20005292c.pdf

struct I2CDevice;
struct Report;

enum {
  STATUS_SET    = 0x10,   // Table 3-1 / 3-2
  READ_FLASH    = 0xB0,   // Table 3-3
  I2C_WRITE     = 0x90,   // Table 3-20 / 3-21
  I2C_WRITE_RPT = 0x92,   // Table 3-22 / 3-23
  I2C_READ      = 0x91,   // launch the 'read' command
  I2C_READ_RPT  = 0x93,   // read w/ repeated start. Table 3-28 / 3-29
  I2C_READ_GET  = 0x40,   // retrieve the read data once command is completed
  SET_SRAM  = 0x60,       // table 3-36 / 3-37
  GET_SRAM  = 0x61,       // table 3-38 / 3-39  
  RESET     = 0x70,       // table 3-40
};

enum {   // table 3-3
  // Unused values -> 0x00: read chip settings, 0x01: read GPIO settings
  FLASH_MANUFACTURER   = 0x02,
  FLASH_PRODUCT        = 0x03,
  FLASH_SERIAL         = 0x04,
  FLASH_FACTORY_SERIAL = 0x05
};

constexpr uint16_t kDefaultVID = 0x04d8;
constexpr uint16_t kDefaultPID = 0x00dd;
constexpr wchar_t kDefaultManufacturer[] = L"Microchip Technology Inc.";
constexpr wchar_t kDefaultProduct[] = L"MCP2221 USB-I2C/UART Combo";



// I2C device communicating to the USB Host with HID packets ('Report')
struct I2CDevice {
 protected:
  static const size_t kMsgLen = 32 + 1;  // = report_size / 2 + 1

  int id_ = 0;
  string path_;
  wchar_t manufacturer_[kMsgLen];
  wchar_t product_[kMsgLen];
  wchar_t serial_[kMsgLen];
  char factory_serial_[60 + 1];   // = (report_size - 4) + 1
  uint8_t hardware_[2], firmware_[2];
  uint16_t vid_ = 0, pid_ = 0;
  bool self_powered_ = false;
  bool remote_wakeup_ = false;
  uint16_t milliamps_ = 0;
  uint8_t divider_ = 0x00;
  uint8_t duty_ = 0x00;
  hid_device*  hid_ = nullptr;

  bool OnError(const char msg[], uint32_t reg, const Report& r);
  bool wait_state(uint8_t expected = 0 /* IDLE */, int nb_try = 10);

 public:
  static bool init();  // to be called once at begining
  static bool exit();  // closes everything
  static bool store_device(const hid_device_info& info);

  bool open();   // must be called after store_device()
  bool reset();
  bool set_clock(uint8_t divider, uint8_t duty);
  bool set_divider(uint8_t divider);
  bool is_connected();
  bool write(uint32_t slave, uint32_t reg, uint8_t data);  // write 1 byte
  bool read(uint32_t slave, uint32_t reg, uint8_t* data, size_t len);
  bool cancel();  // cancel any pending transfer, if any. Always returns false.
  // states: 0=IDLE, 37=addr not found,  85=Data ready,  98=timeout when STOP
  uint8_t get_state();
  void close();
  void print() const;
  void print_status(const char msg[] = nullptr);
  ~I2CDevice() { close(); }
};

// HID's 64-bytes report structure
struct Report {
  static const size_t kSize = 64;
  uint8_t data_w_[kSize + 1];  // +1 for ID (always 0 on MCP2221)
  uint8_t* const data_ = &data_w_[1];

  Report(uint8_t type) {
    reset();
    data_[0] = type;
    data_w_[0] = 0x00;  // constant, Report-ID
  }
  uint8_t operator[](int i) const { return data_[i]; }
  void reset() { memset(data_, 0, kSize); }
  bool USBread(hid_device* const hid) {
    memset(data_, 0, kSize);   // TODO(skal): needed?
    return (hid != NULL) && 
           (hid_read(hid, data_, kSize) == kSize) &&
           (data_[1] == 0x00);
  }
  bool USBwrite(hid_device* const hid) const {
    return (hid != NULL) && (hid_write(hid, data_w_, kSize + 1) >= 0);
  }
  // transaction
  bool USBio(hid_device* const hid) { return USBwrite(hid) && USBread(hid); }
  // transaction w/o payload
  bool USBcmd(hid_device* const hid, uint8_t type, uint8_t cmd = 0) {
    reset();
    data_[0] = type;
    data_[1] = cmd;
    return USBwrite(hid) && USBread(hid);
  }
  // get wchar description. Table 3-7 to 3-9.
  bool get_descriptor(hid_device* const hid, uint8_t dsc, wchar_t dst[],
                      size_t max_len) {
    if (!USBcmd(hid, READ_FLASH, dsc)) return false;
    if (data_[3] != 0x03) return false;
    const size_t len = std::min((size_t)(data_[2] / 2 - 1), max_len - 1);
    for (uint32_t i = 0; i < len; ++i) {
      dst[i] = (wchar_t)get_16s_le(&data_[4 + 2 * i]);
    }
    dst[len] = L'\0';
    return true;
  }
  void print(int len = 0) const {
    if (len == 0) len = kSize;
    for (int i = 0; i <= len; ++i) {
      if ((i % 16) == 0) fprintf(stderr, "\n#%2d  ", i);
      fprintf(stderr, "%.2x ", data_w_[i]);
    }
    fprintf(stderr, "\n");
  }
};

static vector<I2CDevice> devices_;
static I2CDevice* dev_ = nullptr;

bool I2CDevice::init() {
  devices_.clear();
  const int res = hid_init();
  return (res >= 0);
}

bool I2CDevice::exit() {
  devices_.clear();  // will close() all handles
  const int res = hid_exit();
  return (res >= 0);
}

bool I2CDevice::store_device(const hid_device_info& info) {
  I2CDevice dev;
  dev.id_ = devices_.size();
  dev.path_ = string(info.path, strlen(info.path));
  // TODO: serial_
  devices_.push_back(dev);
  return true;
}

bool I2CDevice::open() {
  if (path_.size() == 0) return false;
  hid_ = hid_open_path(path_.c_str());
  if (hid_ == NULL) return false;

  set_clock(5 /* 1.5 MHz */, 0x18 /* duty < 75%*/);
  set_divider(26);

  // Store USB info
  Report r(READ_FLASH);
  if (!r.get_descriptor(hid_, FLASH_MANUFACTURER, manufacturer_, kMsgLen) ||
      !r.get_descriptor(hid_, FLASH_PRODUCT, product_, kMsgLen) ||
      !r.get_descriptor(hid_, FLASH_SERIAL, serial_, kMsgLen)) return false;

  // Table 3-10
  if (!r.USBcmd(hid_, READ_FLASH, FLASH_FACTORY_SERIAL)) return false;
  const size_t len = std::min(r[2], (uint8_t)60);
  memcpy(factory_serial_, &r.data_[4], len);
  factory_serial_[len] = 0;

  if (!r.USBcmd(hid_, STATUS_SET)) return false;
  hardware_[0] = r[46];  // 'A'
  hardware_[1] = r[47];  // '6'
  firmware_[0] = r[48];  // '1'
  firmware_[1] = r[49];  // '2'

  if (!r.USBcmd(hid_, GET_SRAM)) return false;
  vid_ = get_16s_le(r.data_ +  8);
  pid_ = get_16s_le(r.data_ + 10);
  self_powered_  = (r[12] & 0x40) ? true : false;
  remote_wakeup_ = (r[12] & 0x20) ? true : false;
  milliamps_ = r[13] * 2;
  divider_   = r[5] & 0x07;
  duty_      = r[5] & 0x18;
  return true;
}

void I2CDevice::close() {
  if (hid_ != NULL) {
    LOG_MSG("I2C: close(%p)\n", hid_);
    hid_close(hid_);
    hid_ = NULL;
  }
}

void I2CDevice::print() const {
  fprintf(stderr, "Device id #%d: hardware:%c%c, firmware:%c%c\n",
          id_, hardware_[0], hardware_[1], firmware_[0], firmware_[1]);
  fprintf(stderr, "              self-pwr: %d, wakeup:%d  [%d milliamps] "
                  "(divider:0x%x [%.2f MHz] duty:0x%x)\n",
          self_powered_, remote_wakeup_, milliamps_,
          divider_, 48. / (1 << divider_), duty_);
  fprintf(stderr, " manufacturer:   [%ls]\n", manufacturer_);
  fprintf(stderr, " product:        [%ls]\n", product_);
  fprintf(stderr, " serial:         [%ls]\n", serial_);
  fprintf(stderr, " factory serial: [%s]\n", factory_serial_);
}

void I2CDevice::print_status(const char msg[]) {
  Report r(STATUS_SET);
  if (r.USBio(hid_)) {  // Table 3-2
    if (msg != nullptr) fprintf(stderr, "%s:\n", msg);
    fprintf(stderr, " [ 1] success = 0x%.2x (success = 0x00)\n", r[1]);
    fprintf(stderr, " [ 2] cancel state = 0x%.2x (!= 0x00 ?)\n", r[2]);
    fprintf(stderr, " [ 3] speed state = 0x%.2x\n", r[3]);
    if (r[4]) fprintf(stderr, " [ 4] new speed divider = %d\n", r[4]);
    fprintf(stderr, " [ 8] chip comm state = %d\n", r[8]);
    fprintf(stderr, " [ 9] length requested = %d\n", get_16s_le(r.data_ + 9));
    fprintf(stderr, " [11] length transfered = %d\n", get_16s_le(r.data_ + 11));
    fprintf(stderr, " [14] speed divider = %d\n", r[14]);
    fprintf(stderr, " [16] slave address = 0x%x\n", get_16s_le(r.data_ + 16));
    fprintf(stderr, " [20] ACK = %d\n", !(r[20] & 0x40));
    fprintf(stderr, " [22-23] pin values: SCL=%d SDA=%d\n", r[22], r[23]);
    fprintf(stderr, " [25] pending value? %d\n", r[25]);
    if (msg != nullptr) fprintf(stderr, "----------------------\n");
  }
}

bool I2CDevice::reset() {
  Report r(RESET);
  r.data_[1] = 0xab;
  r.data_[2] = 0xcd;
  r.data_[3] = 0xef;
  return r.USBio(hid_);
}

bool I2CDevice::set_clock(uint8_t divider, uint8_t duty) {
  Report r(SET_SRAM);
  if (divider == 0 || divider > 7) divider = 4;   // 3MHz
  r.data_[2] = 0x80 | (duty & 0x18) | (divider & 7);  // table 3-37
  return r.USBio(hid_);  // table 3-38
}

bool I2CDevice::set_divider(uint8_t divider) {
  Report r(STATUS_SET);
  r.data_[3] = 0x20;   // Table 3-1
  r.data_[4] = divider;
  return r.USBio(hid_);  // table 3-2
}

bool I2CDevice::is_connected() {
  Report r(STATUS_SET);
  return r.USBio(hid_) && (r[0] == STATUS_SET);
}

bool I2CDevice::write(uint32_t slave, uint32_t reg, uint8_t data) {
  if (reg >= 128) return false;
  Report r(I2C_WRITE);
  r.data_[1] = 2;
  r.data_[2] = 0;
  r.data_[3] = slave * 2;
  r.data_[4] = reg;
  r.data_[5] = data;
  return r.USBio(hid_);
}

bool I2CDevice::OnError(const char msg[], uint32_t reg, const Report& r) {
  print_status(msg);
  if (reg <= 0xff) {
    r.print();
    LOG_MSG("Reg=%d: code=%d (state=%d)\n", reg, r[1], r[2]);
  }
  return cancel();
}

bool I2CDevice::read(uint32_t slave, uint32_t reg, uint8_t* data, size_t len) {
  if (data == nullptr || reg >= 128 || len >= 60) return false;
  cancel();

  // Table 3-26:
  Report rw(I2C_WRITE);
  rw.data_[1] = 1;
  rw.data_[2] = 0;
  rw.data_[3] = slave * 2;
  rw.data_[4] = reg;
  if (!rw.USBio(hid_)) return OnError("I2C_WRITE", reg, rw);

  Report r(I2C_READ_RPT);
  r.data_[1] = len;
  r.data_[2] = 0;
  r.data_[3] = slave * 2 + 1;
  if (!r.USBio(hid_)) return OnError("I2C_READ", reg, r);

  if (!wait_state(85)) return OnError("wait_state", 0xffff, r);

  // Table 3-30 / 3-31:
  if (!r.USBcmd(hid_, I2C_READ_GET) || r[3] != len) {
    return OnError("READ_GET", reg, r);
  }
  memcpy(data, r.data_ + 4, len);
  return true;
}

bool I2CDevice::cancel() {
  Report r(STATUS_SET);   // Table 3-1
  r.data_[2] = 0x10;
  return r.USBio(hid_) && false;
}

uint8_t I2CDevice::get_state() {
  Report r(STATUS_SET);
  return r.USBio(hid_) ? r[8] : 0;
}

bool I2CDevice::wait_state(uint8_t expected, int nb_try) {
  while (nb_try-- > 0) {
    const uint8_t state = get_state();
    if (state == expected) return true;
    LOG_MSG("State: %u instead of %u\n", state, expected);
    usleep(50 * 1000);
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////

void I2C_print(void) {
  if (dev_ != nullptr) {
    fprintf(stderr, "I2C: OK [dev: %p].\n", dev_);
    dev_->print();
  } else {
    fprintf(stderr, "I2C: not initialized\n");
  }
}

bool I2C_init(void) {
  if (dev_ != nullptr) return true;
  if (!I2CDevice::init()) return false;
  hid_device_info* const all = hid_enumerate(kDefaultVID, kDefaultPID);
  for (const hid_device_info* cur = all; cur != NULL; cur = cur->next) {
    LOG_MSG("Device:  type: %04hx %04hx\n  path: %s\n  serial_number: %ls\n",
            cur->vendor_id, cur->product_id, cur->path,
            cur->serial_number ? cur->serial_number :  L"(NONE)");
    I2CDevice::store_device(*cur);
  }
  hid_free_enumeration(all);
  if (devices_.size() == 0) return false;
  if (!devices_[0].open()) return false;
  dev_ = &devices_[0];
  return true;
}

void I2C_close() {
  if (dev_ != nullptr) {
    (void)I2CDevice::exit();
    dev_ = nullptr;
  }
}

bool I2C_read_bytes(uint8_t dev_address, uint8_t reg_address,
                    uint8_t values[], uint32_t len) {
  return (dev_ != nullptr) && dev_->read(dev_address, reg_address, values, len);
}

bool I2C_write_byte(uint8_t dev_address, uint8_t reg_address, uint8_t value) {
  return (dev_ != nullptr) && dev_->write(dev_address, reg_address, value);
}

////////////////////////////////////////////////////////////////////////////////
// Unix I2C interface

#else

static const char kDevName[] = "/dev/i2c-1";
static int fd = -1;

void I2C_print(void) {
  if (fd >= 0) {
    fprintf(stderr, "I2C OK [fd: %d  dev='%s'].\n", fd, kDevName);
  } else {
    fprintf(stderr, "I2C not initialized\n");
  }
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

#endif

uint8_t I2C_read_byte(uint8_t dev_address, uint8_t reg_address) {
  uint8_t value = 0;
  if (!I2C_read_bytes(dev_address, reg_address, &value, 1)) return 0;
  return value;
}

////////////////////////////////////////////////////////////////////////////////

}  // namespace skl
