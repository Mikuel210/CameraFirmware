#pragma once
#include <cstdint>

struct ColorData {
  uint8_t r, g, b;

  ColorData(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_) {}
  ColorData() {}
};
