#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>

#include "ImplicitSurface.h"
#include "../scene/ray.h"

namespace {

constexpr double EPSILON = 1e-8; 
constexpr double NORMAL_EPS = 1e-6;
constexpr int MAX_STEPS = 1024;

} // namespace

ImplicitSurface::ImplicitSurface(Scene *scene, Material *mat, Type type)
    : SceneObject(scene, mat), type_(type) {}

double ImplicitSurface::eval(glm::dvec3 p) const {
  const double x = p.x, y = p.y, z = p.z;
  switch (type_) {
  case Type::Sphere:
    // SDF: distance to sphere exact distance, step by |f| directly
    return glm::length(p) - radius_;
  case Type::Torus:
    // SDF: distance to torus 
    return glm::length(glm::dvec2(glm::length(glm::dvec2(x, y)) - major_radius_, z)) - minor_radius_;
  case Type::Goursat: {
    double r2 = x * x + y * y + z * z;
    return x * x * x * x + y * y * y * y + z * z * z * z - goursat_a_ * r2 * r2 +
           goursat_b_ * r2 - goursat_c_;
  }
  case Type::Blob: {
    // Wyvill soft object: f(r) = (1 - r^2/b^2)^3 for r^2 < b^2, else 0
    // isosurface at threshold T: sum f_i - T = 0; each blob has its own influence radius
    const double b1_2 = blob_radius1_ * blob_radius1_;
    const double b2_2 = blob_radius2_ * blob_radius2_;
    auto soft = [](double r2, double b2) {
      if (r2 >= b2)
        return 0.0;
      double t = 1.0 - r2 / b2;
      return t * t * t;
    };
    double r1_2 = glm::dot(p - blob_c1_, p - blob_c1_);
    double r2_2 = glm::dot(p - blob_c2_, p - blob_c2_);
    return soft(r1_2, b1_2) + soft(r2_2, b2_2) - blob_threshold_;
  }
  }
  return 0;
}

glm::dvec3 ImplicitSurface::gradient(glm::dvec3 p) const {
  const double x = p.x, y = p.y, z = p.z;
  switch (type_) {
  case Type::Sphere: {
    // SDF gradient: change in f(p) = p/||p|| (unit normal)
    double len = glm::length(p);
    return len > 1e-10 ? p / len : glm::dvec3(1, 0, 0);
  }
  case Type::Torus: {
    // Torus SDF gradient: analytical for efficiency
    double xy_len = glm::length(glm::dvec2(x, y));
    if (xy_len < 1e-12)
      return glm::dvec3(0, 0, z > 0 ? 1 : -1);
    double qx = xy_len - major_radius_;
    glm::dvec2 q(qx, z);
    double q_len = glm::length(q);
    if (q_len < 1e-12)
      return glm::dvec3(1, 0, 0);
    double scale = (1.0 - major_radius_ / xy_len) / q_len;
    return glm::normalize(glm::dvec3(scale * x, scale * y, z / q_len));
  }
  case Type::Goursat: {
    double r2 = x * x + y * y + z * z;
    double factor = -2.0 * goursat_a_ * r2 + goursat_b_;
    return glm::dvec3(4.0 * x * x * x + 2.0 * x * factor,
                      4.0 * y * y * y + 2.0 * y * factor,
                      4.0 * z * z * z + 2.0 * z * factor);
  }
  default: {
    const double h = NORMAL_EPS;
    return glm::dvec3(
        (eval(p + glm::dvec3(h, 0, 0)) - eval(p - glm::dvec3(h, 0, 0))) /
            (2.0 * h),
        (eval(p + glm::dvec3(0, h, 0)) - eval(p - glm::dvec3(0, h, 0))) /
            (2.0 * h),
        (eval(p + glm::dvec3(0, 0, h)) - eval(p - glm::dvec3(0, 0, h))) /
            (2.0 * h));
  }
  }
}

double ImplicitSurface::lipschitzConstant() const {
  if (lipschitz_override_ > 0.0)
    return lipschitz_override_;

  switch (type_) {
  case Type::Sphere:
    return 2.5;
  case Type::Torus:
    return 1.0;
  case Type::Goursat:
    return 14.0;
  case Type::Blob: {
    // Wyvill gradient scales ~1/min_radius^2 so using larger L for smaller blobs to avoid holes
    double minR = std::min(blob_radius1_, blob_radius2_);
    return std::max(4.0, 1.0 / (minR * minR));
  }
  }
  return 4.0;
}

bool ImplicitSurface::sphereTrace(ray &r, isect &i, double tMin,
                                  double tMax) const {
  const double L = lipschitzConstant();
  double t = tMin;
  const glm::dvec3 o = r.getPosition();
  const glm::dvec3 d = r.getDirection();

  for (int step = 0; step < MAX_STEPS && t < tMax; ++step) {
    glm::dvec3 p = o + t * d;
    double f = eval(p);

    if (std::abs(f) < EPSILON) {
      glm::dvec3 g = gradient(p);
      double gLen = glm::length(g);
      glm::dvec3 n = gLen > 1e-10 ? g / gLen : glm::dvec3(1, 0, 0);
      if (glm::dot(n, d) > 0)
        n = -n;

      i.setObject(this);
      i.setMaterial(this->getMaterial());
      i.setT(t);
      i.setN(n);
      i.setUVCoordinates(glm::dvec2(0.5, 0.5));
      return true;
    }

    double stepSize = std::abs(f) / L;
    t += stepSize;
  }
  return false;
}

bool ImplicitSurface::intersectLocal(ray &r, isect &i) const {
  r.setDirection(glm::normalize(r.getDirection()));

  double tMin, tMax;
  BoundingBox localBox = getLocalBoundingBoxImpl();
  if (!localBox.intersect(r, tMin, tMax))
    return false;

  if (tMin < RAY_EPSILON)
    tMin = RAY_EPSILON;

  return sphereTrace(r, i, tMin, tMax);
}

BoundingBox ImplicitSurface::getLocalBoundingBoxImpl() const {
  BoundingBox b;
  switch (type_) {
  case Type::Sphere:
    b.setMin(glm::dvec3(-radius_, -radius_, -radius_));
    b.setMax(glm::dvec3(radius_, radius_, radius_));
    break;
  case Type::Torus: {
    double extent = major_radius_ + minor_radius_;
    b.setMin(glm::dvec3(-extent, -extent, -minor_radius_));
    b.setMax(glm::dvec3(extent, extent, minor_radius_));
    break;
  }
  case Type::Goursat: {
    double extent = 1.5;
    b.setMin(glm::dvec3(-extent, -extent, -extent));
    b.setMax(glm::dvec3(extent, extent, extent));
    break;
  }
  case Type::Blob: {
    double r = std::max(blob_radius1_, blob_radius2_) + 0.1;
    glm::dvec3 lo = glm::min(blob_c1_, blob_c2_) - glm::dvec3(r);
    glm::dvec3 hi = glm::max(blob_c1_, blob_c2_) + glm::dvec3(r);
    b.setMin(lo);
    b.setMax(hi);
    break;
  }
  }
  return b;
}

BoundingBox ImplicitSurface::ComputeLocalBoundingBox() {
  return getLocalBoundingBoxImpl();
}

void ImplicitSurface::glDrawLocal(int quality, bool actualMaterials,
                                  bool actualTextures) const {
  (void)quality;
  (void)actualMaterials;
  (void)actualTextures;
  // can't draw implicit surfaces with OpenGL
}
