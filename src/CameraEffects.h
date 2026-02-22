#ifndef __CAMERA_EFFECTS_H__
#define __CAMERA_EFFECTS_H__

#include <glm/glm.hpp>
#include <random>

/**
 * - depth of field via lens aperture sampling
 * - motion blur via temporal sampling
 * - lens distortion utilities
 */

/**
 * depth of field is achieved by:
 * sampling random points on the lens aperture
 * casting rays from these points through focus point
 * varying the focal point distance to control DoF strength
 *
 * formula for depth of field:
 * - focal length: distance from lens to focus plane
 * - f-number (f-stop): focal_length / aperture_diameter
 * - lower f-number = larger aperture = stronger DoF
 * - circle of confusion: how blurry out-of-focus objects appear
 */
class LensAperture {
public:
  /**
   * @brief create lens aperture with given parameters
   *
   * @param focal_distance  distance from camera to focus plane (in world units)
   * @param f_number        f-stop value lower = more dif
   * @param focal_length    approximate focal length parameter (for realism)
   * @param seed            random seed
   */
  LensAperture(double focal_distance = 1.0, double f_number = 8.0,
               double focal_length = 50.0, unsigned int seed = std::random_device{}())
      : focal_distance_(focal_distance),
        f_number_(std::max(1.0, f_number)),
        focal_length_(std::max(1.0, focal_length)),
        enabled_(false),
        generator_(seed),
        distribution_(0.0, 1.0) {
    updateAperture();
  }

  /**
   * uses concentric mapping for uniform distribution on disk
   *
   * @return glm::dvec2 point on aperture in camera space (x, y)
   */
  glm::dvec2 sampleAperturePoint();

  /**
   * sample a point on the lens with Gaussian distribution (better bokeh)
   *
   * @return glm::dvec2
   */
  glm::dvec2 sampleAperturePointGaussian();

  /**
   * set focal distance (distance to plane that's in focus)
   *
   * @param distance focal distance in world units
   */
  void setFocalDistance(double distance) {
    focal_distance_ = std::max(0.1, distance);
  }

  /**
   * set f-number (affects blur strength)
   *
   * @param f_num f-stop value
   */
  void setFNumber(double f_num) {
    f_number_ = std::max(1.0, f_num);
    updateAperture();
  }

  /**
   * get aperture radius
   *
   * @return double radius of lens aperture
   */
  double getApertureRadius() const { return aperture_radius_; }

  /**
   * directly set aperture radius in world units
   * bypasses focal_length/f_number calculation
   *
   * @param radius aperture radius in world units
   */
  void setApertureRadius(double radius) {
    aperture_radius_ = std::max(0.0, radius);
  }

  /**
   * get focal distance
   *
   * @return double
   */
  double getFocalDistance() const { return focal_distance_; }

  /**
   * enable/disable depth of field
   *
   * @param enabled
   */
  void setEnabled(bool enabled) { enabled_ = enabled; }

  /**
   * check if DoF is enabled
   *
   * @return bool
   */
  bool isEnabled() const { return enabled_; }

private:
  double focal_distance_;   ///< distance to focus plane
  double f_number_;         ///< f-stop (lower = larger aperture = more blur)
  double focal_length_;     ///< focal length of lens
  double aperture_radius_;  ///< radius of aperture disk
  bool enabled_;            ///< whether DoF is applied

  std::mt19937 generator_;
  std::uniform_real_distribution<double> distribution_;

  void updateAperture() {
    // aperture_diameter = focal_length / f_number
    // aperture_radius = aperture_diameter / 2
    aperture_radius_ = focal_length_ / (2.0 * f_number_);
  }

  double random() { return distribution_(generator_); }
};

/**
 * samples time for motion blur and temporal coherence
 *
 * motion blur is achieved by:
 * sampling a time value in the exposure interval
 * updating object transformations for that time
 * rendering at multiple time points and averaging
 *
 * creates the visual effect of moving objects leaving trails
 */
class TemporalSampler {
public:
  /**
   * @param shutter_open  time when shutter opens (0.0)
   * @param shutter_close time when shutter closes (1.0)
   * @param seed          random seed
   */
  TemporalSampler(double shutter_open = 0.0, double shutter_close = 1.0,
                  unsigned int seed = std::random_device{}())
      : shutter_open_(shutter_open),
        shutter_close_(shutter_close),
        generator_(seed),
        distribution_(0.0, 1.0),
        enabled_(false) {}

  /**
   * sample a time value in the shutter exposure interval
   *
   * @return double time value in [shutter_open, shutter_close]
   */
  double sampleTime();

  /**
   * set shutter open/close times
   *
   * @param open  shutter opens at this time
   * @param close shutter closes at this time
   */
  void setShutterInterval(double open, double close) {
    shutter_open_ = open;
    shutter_close_ = std::max(open + 0.001, close);
  }

  /**
   * enable/disable motion blur
   *
   * @param enabled
   */
  void setEnabled(bool enabled) { enabled_ = enabled; }

  /**
   * check if motion blur is enabled
   *
   * @return bool
   */
  bool isEnabled() const { return enabled_; }

  /**
   * get exposure duration
   *
   * @return double shutter_close - shutter_open
   */
  double getExposureDuration() const { return shutter_close_ - shutter_open_; }

private:
  double shutter_open_;
  double shutter_close_;
  bool enabled_;

  std::mt19937 generator_;
  std::uniform_real_distribution<double> distribution_;

  double random() { return distribution_(generator_); }
};

/**
 * uniform disk sampling using concentric mapping
 *
 * maps uniform random point in [0,1]^2 to uniform point on disk
 * this avoids the bias near the center that naive polar mapping creates
 *
 * @param u random value in [0, 1]
 * @param v random value in [0, 1]
 * @return glm::dvec2 point on unit disk
 */
inline glm::dvec2 concentricDiskSample(double u, double v) {
  double r = std::sqrt(u);
  double theta = 2.0 * M_PI * v;
  return glm::dvec2(r * std::cos(theta), r * std::sin(theta));
}

#endif // __CAMERA_EFFECTS_H__
