#include "CameraEffects.h"
#include <cmath>

glm::dvec2 LensAperture::sampleAperturePoint() {
  if (!enabled_)
    return glm::dvec2(0.0, 0.0);
  double u = random();
  double v = random();
  glm::dvec2 disk = concentricDiskSample(u, v);

  return aperture_radius_ * disk;
}

glm::dvec2 LensAperture::sampleAperturePointGaussian() {
  if (!enabled_)
    return glm::dvec2(0.0, 0.0);

  std::normal_distribution<double> gaussian(0.0, aperture_radius_ / 2.0);
  double x = gaussian(generator_);
  double y = gaussian(generator_);

  double r = std::sqrt(x * x + y * y);
  if (r > aperture_radius_) {
    double scale = aperture_radius_ / r;
    x *= scale;
    y *= scale;
  }

  return glm::dvec2(x, y);
}

double TemporalSampler::sampleTime() {
  if (!enabled_)
    return shutter_open_;

  double t = random();
  return shutter_open_ + t * (shutter_close_ - shutter_open_);
}
