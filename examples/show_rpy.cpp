// Copyright 2022 Pascal 'skal' Massimino
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
// simple measurement tool
//  Displays the R/P/Y and other measurements using SDL
//
// Author: skal (pascal.massimino@gmail.com)
////////////////////////////////////////////////////////////////////////////////

#include "sklmpu9255.h"

#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
using std::vector;

#if defined(USE_THREAD)
#include <pthread.h>
#endif  // USE_THREAD

#if defined(USE_SDL)
#include "SDL.h"
#endif  // USE_SDL

int what = 0;   // 0=raw RPY, 1: angles, 2:accel, 3:mag
bool mag = false;   // show magneto field
int delay_ms = 10;

struct RPYData { float v[3]; };
vector<RPYData> buf;
int buf_pos = 0;
bool run_thread = true;

skl::MPU mpu;

#if defined(USE_THREAD)
void* runThread(void* ptr) {
  const int buf_len = *(int*)ptr;
  buf_pos = 0;
  while (run_thread) {
    const bool ok =
        (what == 0) ? mpu.gyro(buf[buf_pos].v) :
        (what == 1) ? mpu.get_rpy(buf[buf_pos].v) :
        (what == 2) ? mpu.mag(buf[buf_pos].v) :
        (what == 3) ? mpu.accel(buf[buf_pos].v) : false;
    if (ok) {
      if (++buf_pos == buf_len) buf_pos = 0;
      usleep(delay_ms * 1000);
    }
  }
 end:
  pthread_exit(NULL);
  return NULL;  // not reached
}
#else
#error "need THREAD for show_rpy"
#endif

////////////////////////////////////////////////////////////////////////////////

#if defined(USE_SDL)
template<typename T> T clamp(T v, T m, T M) { return std::min(std::max(v, m), M); }
class Window {
 public:
  Window(int W, int H, bool fullscreen = false) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) return;
    screen_ = SDL_CreateWindow(
        "sklmpu9255 Visu",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        W, H, fullscreen ? SDL_WINDOW_FULLSCREEN : SDL_WINDOW_RESIZABLE);
    if (screen_ == NULL) return;
    renderer_ = SDL_CreateRenderer(screen_, -1, SDL_RENDERER_ACCELERATED);
    if (renderer_ == NULL) return;
    SDL_RenderSetLogicalSize(renderer_, W, H);
    SDL_SetRenderDrawColor(renderer_, 0x00, 0x00, 0x00, 0xff);
    SDL_RenderClear(renderer_);
    surface_ = SDL_CreateTexture(renderer_,
                                 SDL_PIXELFORMAT_ARGB8888,
                                 SDL_TEXTUREACCESS_STREAMING, W, H);
    if (surface_ == NULL) return;
    w_ = W;
    h_ = H;
    buf_ = new uint32_t[w_ * h_];
    if (buf_ != NULL) flush_events();
  }
  uint32_t* lock() const {
    if (buf_ != nullptr) memset(buf_, 0, w_ * h_ * sizeof(*buf_));
    return buf_;
  }
  bool unlock() {
    if (renderer_ == NULL) return false;
    if (surface_ == NULL) return false;
    if (buf_ == NULL) return false;
    SDL_UpdateTexture(surface_, NULL, buf_, w_ * sizeof(uint32_t));
    SDL_RenderCopy(renderer_, surface_, NULL, NULL);
    SDL_RenderPresent(renderer_);
    flush_events();
    return true;
  }
  ~Window() {
    if (screen_) SDL_SetWindowFullscreen(screen_, 0);
    delete[] buf_;
    flush_events();
    if (surface_ != NULL) SDL_DestroyTexture(surface_);
    if (renderer_ != NULL) SDL_DestroyRenderer(renderer_);
    if (screen_ != NULL) SDL_DestroyWindow(screen_);
    SDL_Quit();
  }
  void flush_events() {
    key_ = -1;
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_KEYDOWN) key_ = event.key.keysym.sym;
    }
  }
  uint32_t* row(int y) { return buf_ + y * w_; }
  void draw_v(int y0, int y1, int x, uint32_t c) {
    if (y1 < y0) std::swap(y0, y1);
    for (int y = y0; y <= y1; ++y) row(y)[x] = c;
  }
  int w() const { return w_; }
  int h() const { return h_; }
  int key() const { return key_; }

 private:
  int w_ = 0, h_ = 0;
  uint32_t* buf_ = nullptr;
  SDL_Texture* surface_ = NULL;
  SDL_Renderer* renderer_ = NULL;
  SDL_Window* screen_ = NULL;
  int key_ = -1;
};
#endif  // USE_SDL

////////////////////////////////////////////////////////////////////////////////

static const char* const kWhat[4] = { "gyro", "rpy", "mag", "accel" };
static const char* const kOff[2] = { "off", "on" };

static void print_help() {
  printf("Options:\n");
  printf("  -gyro ............ show gyro measurement\n");
  printf("  -rpy ............. show angle\n");
  printf("  -accel ........... show accel\n");
  printf("  -mag ............. show magnetic field\n");
  printf("  -ahrs ............ use AHRS correction\n");
  printf("  -noshow .......... just print values\n");
  printf("  -avg <int> ....... average window size\n");
  printf("  -cnt <int> ....... number of iterations\n");
  printf("  -ms <int> ........ measurement delay in ms\n");
  printf("  -s <int> <int> ... window size\n");
  printf("  -cal <float> ..... calibrate time for mag in secs\n");
  printf("\nkeys:\n");
  printf("r :     toggle ahrs\n");
  printf("g :     toggle gyro / rpy / accel / gyro\n");
  printf("a/s/d : toggle x/y/z measurement\n");
  printf("1 :     change accel scale\n");
  printf("2 :     change gyro scale\n");
  printf("h :     this help\n");
  printf("q :     quit\n");
}

int main(int argc, char **argv) {
  bool show = true;
  int cnt = 300000;
  int W = 0, H = 0;
  bool ahrs = false;
  int avg_size = 8;   /* 0=off */
  bool show_v0 = true, show_v1 = true, show_v2 = true;
  bool show_mag = false;
  int a_scale = 0, g_scale = 0;
  float calibrate = 1.f;
  static const skl::accel_full_scale_t kAScales[] = {
    skl::ACCEL_FULL_SCALE_2G,
    skl::ACCEL_FULL_SCALE_4G,
    skl::ACCEL_FULL_SCALE_8G,
    skl::ACCEL_FULL_SCALE_16G
  };
  static const skl::gyro_full_scale_t kGScales[] = {
    skl::GYRO_FULL_SCALE_250DPS,
    skl::GYRO_FULL_SCALE_500DPS,
    skl::GYRO_FULL_SCALE_1000DPS,
    skl::GYRO_FULL_SCALE_2000DPS
  };
  for (int c = 1; c < argc; ++c) {
    if (!strcmp(argv[c], "-noshow")) {
      show = false;
    } else if (!strcmp(argv[c], "-gyro")) {
      what = 0;
    } else if (!strcmp(argv[c], "-rpy")) {
      what = 1;
    } else if (!strcmp(argv[c], "-mag")) {
      what = 2;
    } else if (!strcmp(argv[c], "-accel")) {
      what = 3;
    } else if (!strcmp(argv[c], "-ahrs")) {
      ahrs = true;
    } else if (c + 1 < argc && !strcmp(argv[c], "-cal")) {
      calibrate = atof(argv[++c]);
    } else if (c + 1 < argc && !strcmp(argv[c], "-avg")) {
      avg_size = atoi(argv[++c]);
    } else if (c + 1 < argc && !strcmp(argv[c], "-cnt")) {
      cnt = atoi(argv[++c]);
    } else if (c + 1 < argc && !strcmp(argv[c], "-ms")) {
      delay_ms = atoi(argv[++c]);
    } else if (c + 2 < argc && !strcmp(argv[c], "-s")) {
      W = atoi(argv[++c]);
      H = atoi(argv[++c]);
    } else if (!strcmp(argv[c], "-h")) {
      print_help();
      return 0;
    }
  }
  if (W == 0) W = 1280;
  if (H == 0) H = 1 + (W / 3);

  if (!mpu.init(calibrate, ahrs, avg_size)) {
    fprintf(stderr, "init/calibration failed.\n");
    return 1;
  }
  mpu.set_full_scales(kAScales[a_scale], kGScales[g_scale]);

#if defined(USE_SDL)
  Window* window = nullptr;
  if (show) {
    window = new Window(W, H);
    if (window == nullptr) {
      fprintf(stderr, "Can't init video mode\n");
      show = false;
    }
  }
#else
  show = false;
  uint32_t* window = nullptr;  // decoy
#endif  // USE_SDL

  const int buf_len = W;
  buf.resize(buf_len);

#if defined(USE_THREAD)
  pthread_t t_id;
  if (pthread_create(&t_id, NULL, &runThread, (void*)&buf_len) == 0) {
    run_thread = true;   // 'started'
    fprintf(stderr, "\nThread CREATED. len=%d\n", buf_len);
  }
#endif

  fprintf(stderr, "SHOWING %s\n", kWhat[what]);
  mpu.print();
  while (cnt > 0) {
    if (show) {
#if defined(USE_SDL)
      switch (window->key()) {
        default: case -1: break;
        case 'q': cnt = 0; break;
        case 'a': show_v0 = !show_v0; break;
        case 's': show_v1 = !show_v1; break;
        case 'd': show_v2 = !show_v2; break;
        case 'g':
          what = (what + 1) % 4;
          printf("showing %s\n", kWhat[what]);
        break;
        case 'r':
          ahrs = !ahrs;
          printf("AHRS: %s\n", kOff[ahrs]);
        break;
        case 'h': print_help(); break;
        case '1':
          a_scale = (a_scale + 1) % 4;
          mpu.set_full_scales(kAScales[a_scale],
                              kGScales[g_scale]);
          printf("a_scale=%d\n", a_scale);
        break;
        case '2':
          g_scale = (g_scale + 1) % 4;
          mpu.set_full_scales(kAScales[a_scale],
                              kGScales[g_scale]);
          printf("g_scale=%d\n", g_scale);
        break;
      }
      if (!window->lock()) continue;
      const int H = window->h() - 1;
      int idx = buf_pos;
      static float kUnits[4] = { 0.01f, 1. / 360., 1. / 200., 30. / 200. };
      const float scale = H * kUnits[what];
      const float mid = (what == 1) ? H : H / 2;
      int last_v0 = 0, last_v1 = 0, last_v2 = 0;
      for (int x = 0; x < buf_len; ++x) {
        const int v0 = clamp((int)(mid - buf[idx].v[0] * scale), 0, H);
        const int v1 = clamp((int)(mid - buf[idx].v[1] * scale), 0, H);
        const int v2 = clamp((int)(mid - buf[idx].v[2] * scale), 0, H);
        if (show_v0) window->draw_v(last_v0, v0, x, 0xffff00ffu);
        if (show_v1) window->draw_v(last_v1, v1, x, 0xffffff00u);
        if (show_v2) window->draw_v(last_v2, v2, x, 0xff00ffffu);
        last_v0 = v0;
        last_v1 = v1;
        last_v2 = v2;
        if (what != 1) window->row(H >> 1)[x] = 0xffffffffu;
        if (++idx >= buf_len) idx = 0;
      }
      window->unlock();
#endif  // USE_SDL
    } else {
      static int last_idx = 0;
      const int idx = buf_pos;
      if (idx == last_idx) continue;
      printf("#%d: %.3f %.3f %.3f\n",
             last_idx, buf[last_idx].v[0],  buf[last_idx].v[1], buf[last_idx].v[2]);
      last_idx = idx;
      usleep(1000);
    }
    if (cnt) --cnt;
  }
  run_thread = false;
#if defined(USE_THREAD)
  pthread_join(t_id, NULL);
  fprintf(stderr, "Thread STOPPING\n");
#endif
  mpu.close();
  delete window;
  return 0;
}
