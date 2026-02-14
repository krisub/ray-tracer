#pragma once

#include <algorithm>
#include <glm/vec3.hpp>

// interpolation helpers shared by texture sampling and fxaa
namespace interp {

inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

inline double clamp01(double v) { return clamp(v, 0.0, 1.0); }

// linear interpolation: t=0 -> a, t=1 -> b
template <typename T> inline T lerp(const T &a, const T &b, double t) {
  return a * (1.0 - t) + b * t;
}

// bilinear interpolation from four samples:
// c00 = (x0,y0), c10 = (x1,y0), c01 = (x0,y1), c11 = (x1,y1)
// tx in [0,1] across x, ty in [0,1] across y.
template <typename T>
inline T bilerp(const T &c00, const T &c10, const T &c01, const T &c11,
                double tx, double ty) {
  const T c0 = lerp(c00, c10, tx);
  const T c1 = lerp(c01, c11, tx);
  return lerp(c0, c1, ty);
}

// BT.601 luma weights
inline double luminance601(const glm::dvec3 &c) {
  return 0.299 * c.r + 0.587 * c.g + 0.114 * c.b;
}

} // namespace interp

