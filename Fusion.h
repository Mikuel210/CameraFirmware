#pragma once
#include "Color.h"
#include "ColorData.h"
#include <cmath>

class Fusion {
public:
  // Pixels with saturation below this threshold are classified as WHITE
  // regardless of hue or value (catches both white and black inputs).
  static constexpr float SAT_THRESHOLD = 0.2f;

  // Reference hues (degrees, 0–360) for each chromatic color,
  // derived from the calibration samples in Fusion.cpp.
  static constexpr float HUE_YELLOW = 60.0f;
  static constexpr float HUE_GREEN  = 125.0f;
  static constexpr float HUE_BLUE   = 220.0f;

  static Color rgbToColor(ColorData cd) {
    // --- RGB → HSV ---
    float r = cd.r / 255.0f;
    float g = cd.g / 255.0f;
    float b = cd.b / 255.0f;

    float cmax  = r > g ? (r > b ? r : b) : (g > b ? g : b);
    float cmin  = r < g ? (r < b ? r : b) : (g < b ? g : b);
    float delta = cmax - cmin;

    float s = (cmax > 0.0f) ? (delta / cmax) : 0.0f;

    // Low saturation → achromatic (white, grey, or black) → WHITE
    if (s < SAT_THRESHOLD) return WHITE;

    // Hue (0–360°)
    float h;
    if (cmax == r)      h = 60.0f * std::fmod((g - b) / delta, 6.0f);
    else if (cmax == g) h = 60.0f * ((b - r) / delta + 2.0f);
    else                h = 60.0f * ((r - g) / delta + 4.0f);
    if (h < 0.0f) h += 360.0f;

    // Find chromatic color with smallest angular hue distance
    const float refs[]   = { HUE_YELLOW, HUE_GREEN, HUE_BLUE };
    const Color colors[] = { YELLOW,     GREEN,      BLUE     };

    float minDist = 361.0f;
    Color result  = WHITE;  // fallback (should never stay WHITE here)

    for (int i = 0; i < 3; i++) {
      float dist = std::abs(h - refs[i]);
      if (dist > 180.0f) dist = 360.0f - dist;  // wrap around 0°/360°
      if (dist < minDist) {
        minDist = dist;
        result  = colors[i];
      }
    }

    return result;
  }
};
