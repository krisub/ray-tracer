#include "ReactionDiffusionTexture.h"
#include "../interp.h"
#include <algorithm>
#include <cmath>
#include <random>

#define _USE_MATH_DEFINES
#include <cmath>

namespace {
inline int wrap(int i, int n) {
  int r = i % n;
  return r < 0 ? r + n : r;
}
} // namespace

ReactionDiffusionTexture::ReactionDiffusionTexture(int width, int height,
                                                   Pattern pattern,
                                                   int iterations)
    : width_(width), height_(height), pattern_(pattern), iterations_(iterations),
      concentration_(static_cast<size_t>(width) * height, 0.0) {
  simulate();
}

void ReactionDiffusionTexture::simulate() {
  switch (pattern_) {
  case Pattern::Spots:
    simulateSpots();
    break;
  case Pattern::Stripes:
    simulateStripes();
    break;
  case Pattern::Wood:
    simulateWood();
    break;
  case Pattern::Marble:
    simulateMarble();
    break;
  }
}

void ReactionDiffusionTexture::simulateSpots() {
  // isotropic two-morphogen RD, threshold reaction (Witkin & Kass section 6.1)
  // giraffe like cellular pattern
  const double a_plus = 1.0;   // diffusion rate for C+
  const double a_minus = 0.5;  // diffusion rate for C- (slower = activator)
  const double b = 1.0;        // dissipation
  const double k = 1.0;        // reaction strength
  const double h = 1.0;        // grid spacing
  const double dt = 0.02;

  std::vector<double> C_plus(static_cast<size_t>(width_) * height_, 0.0);
  std::vector<double> C_minus(static_cast<size_t>(width_) * height_, 0.0);

  // jittered diamond grid initial conditions (Witkin & Kass)
  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> jitter(-0.3, 0.3);
  const int step = std::max(4, std::min(width_, height_) / 16);
  for (int j = 0; j < height_; j += step) {
    for (int i = (j / step) % 2; i < width_; i += step) {
      int ii = std::min(width_ - 1, std::max(0, i + static_cast<int>(jitter(rng) * step)));
      int jj = std::min(height_ - 1, std::max(0, j + static_cast<int>(jitter(rng) * step)));
      size_t idx = static_cast<size_t>(jj) * width_ + ii;
      C_plus[idx] = 1.0 + jitter(rng) * 0.5;
      C_minus[idx] = 0.5 + jitter(rng) * 0.3;
    }
  }

  // noise
  std::uniform_real_distribution<double> noise(-0.1, 0.1);
  for (size_t i = 0; i < C_plus.size(); ++i) {
    C_plus[i] += noise(rng);
    C_minus[i] += noise(rng);
    C_plus[i] = std::max(0.0, C_plus[i]);
    C_minus[i] = std::max(0.0, C_minus[i]);
  }

  const double h2 = h * h;
  const double lap_coeff_plus = a_plus * a_plus / h2;
  const double lap_coeff_minus = a_minus * a_minus / h2;

  for (int iter = 0; iter < iterations_; ++iter) {
    std::vector<double> C_plus_new = C_plus;
    std::vector<double> C_minus_new = C_minus;

    for (int j = 0; j < height_; ++j) {
      for (int i = 0; i < width_; ++i) {
        size_t idx = static_cast<size_t>(j) * width_ + i;

        // discrete Laplacian (5-point stencil, cyclic BC)
        int i1 = wrap(i + 1, width_), i0 = wrap(i - 1, width_);
        int j1 = wrap(j + 1, height_), j0 = wrap(j - 1, height_);

        double lap_plus =
            (C_plus[static_cast<size_t>(j) * width_ + i1] +
             C_plus[static_cast<size_t>(j) * width_ + i0] +
             C_plus[static_cast<size_t>(j1) * width_ + i] +
             C_plus[static_cast<size_t>(j0) * width_ + i] - 4.0 * C_plus[idx]);
        double lap_minus =
            (C_minus[static_cast<size_t>(j) * width_ + i1] +
             C_minus[static_cast<size_t>(j) * width_ + i0] +
             C_minus[static_cast<size_t>(j1) * width_ + i] +
             C_minus[static_cast<size_t>(j0) * width_ + i] - 4.0 * C_minus[idx]);

        // reaction: R = k if C+ > C-, else 0 (both morphogens)
        double R = (C_plus[idx] > C_minus[idx]) ? k : 0.0;

        C_plus_new[idx] =
            C_plus[idx] +
            dt * (lap_coeff_plus * lap_plus - b * C_plus[idx] + R);
        C_minus_new[idx] =
            C_minus[idx] +
            dt * (lap_coeff_minus * lap_minus - b * C_minus[idx] + R);

        C_plus_new[idx] = std::max(0.0, C_plus_new[idx]);
        C_minus_new[idx] = std::max(0.0, C_minus_new[idx]);
      }
    }
    C_plus = std::move(C_plus_new);
    C_minus = std::move(C_minus_new);
  }

  // store difference C+ - C- map to [0,1] for coloring
  double lo = 1e9, hi = -1e9;
  for (size_t i = 0; i < C_plus.size(); ++i) {
    double d = C_plus[i] - C_minus[i];
    concentration_[i] = d;
    lo = std::min(lo, d);
    hi = std::max(hi, d);
  }
  double range = hi - lo;
  if (range < 1e-9)
    range = 1.0;
  for (size_t i = 0; i < concentration_.size(); ++i) {
    concentration_[i] = (concentration_[i] - lo) / range;
  }
}

void ReactionDiffusionTexture::simulateStripes() {
  // anisotropic RD for zebra-like stripes (Witkin & Kass section 6.2)
  const double s = 1.0, e = 2.0, d = 1.5;
  const double a1_plus = e * s, a2_plus = s;   // P+ = [0, es, s]
  const double a1_minus = e * s, a2_minus = d * s; // P- = [0, es, ds]
  const double b = 1.0, k = 1.0;
  const double h = 1.0, dt = 0.02;

  std::vector<double> C_plus(static_cast<size_t>(width_) * height_, 0.0);
  std::vector<double> C_minus(static_cast<size_t>(width_) * height_, 0.0);

  std::mt19937 rng(54321);
  std::uniform_real_distribution<double> init(-1.0, 1.0);
  for (size_t i = 0; i < C_plus.size(); ++i) {
    C_plus[i] = init(rng);
    C_minus[i] = init(rng);
  }

  const double h2 = h * h;
  // anisotropic diffusion: faster in x (a1) than y (a2) for horizontal stripes
  const double Dxx_plus = a1_plus * a1_plus, Dyy_plus = a2_plus * a2_plus;
  const double Dxx_minus = a1_minus * a1_minus, Dyy_minus = a2_minus * a2_minus;

  for (int iter = 0; iter < iterations_; ++iter) {
    std::vector<double> C_plus_new = C_plus;
    std::vector<double> C_minus_new = C_minus;

    for (int j = 0; j < height_; ++j) {
      for (int i = 0; i < width_; ++i) {
        size_t idx = static_cast<size_t>(j) * width_ + i;
        int i1 = wrap(i + 1, width_), i0 = wrap(i - 1, width_);
        int j1 = wrap(j + 1, height_), j0 = wrap(j - 1, height_);

        double d2x_plus = (C_plus[static_cast<size_t>(j) * width_ + i1] +
                           C_plus[static_cast<size_t>(j) * width_ + i0] -
                           2.0 * C_plus[idx]);
        double d2y_plus = (C_plus[static_cast<size_t>(j1) * width_ + i] +
                           C_plus[static_cast<size_t>(j0) * width_ + i] -
                           2.0 * C_plus[idx]);
        double lap_plus = (Dxx_plus * d2x_plus + Dyy_plus * d2y_plus) / h2;

        double d2x_minus = (C_minus[static_cast<size_t>(j) * width_ + i1] +
                            C_minus[static_cast<size_t>(j) * width_ + i0] -
                            2.0 * C_minus[idx]);
        double d2y_minus = (C_minus[static_cast<size_t>(j1) * width_ + i] +
                            C_minus[static_cast<size_t>(j0) * width_ + i] -
                            2.0 * C_minus[idx]);
        double lap_minus = (Dxx_minus * d2x_minus + Dyy_minus * d2y_minus) / h2;

        double R = (C_plus[idx] > C_minus[idx]) ? k : 0.0;

        C_plus_new[idx] = C_plus[idx] + dt * (lap_plus - b * C_plus[idx] + R);
        C_minus_new[idx] =
            C_minus[idx] + dt * (lap_minus - b * C_minus[idx] + R);
      }
    }
    C_plus = std::move(C_plus_new);
    C_minus = std::move(C_minus_new);
  }

  double lo = 1e9, hi = -1e9;
  for (size_t i = 0; i < C_plus.size(); ++i) {
    concentration_[i] = C_plus[i] - C_minus[i];
    lo = std::min(lo, concentration_[i]);
    hi = std::max(hi, concentration_[i]);
  }
  double range = hi - lo;
  if (range < 1e-9)
    range = 1.0;
  for (size_t i = 0; i < concentration_.size(); ++i) {
    concentration_[i] = (concentration_[i] - lo) / range;
  }
}

namespace {
// noise
inline double hash2(double x, double y) {
  double d = x * 12.9898 + y * 78.233;
  return std::fmod(std::sin(d) * 43758.5453, 1.0);
}
double noise2(double x, double y) {
  int xi = static_cast<int>(std::floor(x)) & 255;
  int yi = static_cast<int>(std::floor(y)) & 255;
  double xf = x - std::floor(x), yf = y - std::floor(y);
  double u = xf * xf * (3.0 - 2.0 * xf), v = yf * yf * (3.0 - 2.0 * yf);
  double a = hash2(xi, yi);
  double b = hash2(xi + 1, yi);
  double c = hash2(xi, yi + 1);
  double d = hash2(xi + 1, yi + 1);
  return a * (1 - u) * (1 - v) + b * u * (1 - v) + c * (1 - u) * v + d * u * v;
}
double turbulence(double x, double y, int octaves) {
  double v = 0, f = 1, a = 1;
  for (int i = 0; i < octaves; ++i) {
    v += a * std::fabs(noise2(x * f, y * f));
    f *= 2;
    a *= 0.5;
  }
  return v;
}
} // namespace

void ReactionDiffusionTexture::simulateWood() {
  for (int j = 0; j < height_; ++j) {
    for (int i = 0; i < width_; ++i) {
      double u = static_cast<double>(i) / (width_ - 1);
      double v = static_cast<double>(j) / (height_ - 1);
      double rings = 20.0 * v + turbulence(u * 4, v * 4, 4) * 3.0;
      double grain = std::sin(rings * 3.14159 * 2.0) * 0.5 + 0.5;
      concentration_[static_cast<size_t>(j) * width_ + i] = grain;
    }
  }
}

void ReactionDiffusionTexture::simulateMarble() {
  for (int j = 0; j < height_; ++j) {
    for (int i = 0; i < width_; ++i) {
      double u = static_cast<double>(i) / (width_ - 1);
      double v = static_cast<double>(j) / (height_ - 1);
      double turb = turbulence(u * 5, v * 5, 5);
      double vein = std::sin(u * 8.0 + turb * 12.0) * 0.5 + 0.5;
      concentration_[static_cast<size_t>(j) * width_ + i] = vein;
    }
  }
}

double ReactionDiffusionTexture::sampleConcentration(int x, int y) const {
  x = std::max(0, std::min(x, width_ - 1));
  y = std::max(0, std::min(y, height_ - 1));
  return concentration_[static_cast<size_t>(y) * width_ + x];
}

glm::dvec3 ReactionDiffusionTexture::getMappedValue(const glm::dvec2 &coord) const {
  if (concentration_.empty())
    return glm::dvec3(1.0, 1.0, 1.0);

  double u = interp::clamp01(coord.x);
  double v = interp::clamp01(coord.y);
  double x = u * (width_ - 1);
  double y = v * (height_ - 1);

  int x0 = static_cast<int>(std::floor(x));
  int y0 = static_cast<int>(std::floor(y));
  int x1 = std::min(x0 + 1, width_ - 1);
  int y1 = std::min(y0 + 1, height_ - 1);

  double dx = x - x0;
  double dy = y - y0;

  double c00 = sampleConcentration(x0, y0);
  double c10 = sampleConcentration(x1, y0);
  double c01 = sampleConcentration(x0, y1);
  double c11 = sampleConcentration(x1, y1);

  double c = interp::bilerp(c00, c10, c01, c11, dx, dy);

  glm::dvec3 dark, light;
  switch (pattern_) {
  case Pattern::Wood:
    dark = glm::dvec3(0.25, 0.12, 0.05);
    light = glm::dvec3(0.75, 0.55, 0.35);
    break;
  case Pattern::Marble:
    dark = glm::dvec3(0.6, 0.6, 0.65);
    light = glm::dvec3(0.95, 0.93, 0.9);
    break;
  default:
    dark = glm::dvec3(0.2, 0.15, 0.1);
    light = glm::dvec3(0.9, 0.85, 0.7);
    break;
  }
  return interp::lerp(light, dark, c);
}
