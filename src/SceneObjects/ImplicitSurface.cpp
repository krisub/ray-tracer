#define _USE_MATH_DEFINES
#include <cmath>

#include "ImplicitSurface.h"
#include "../scene/ray.h"

namespace {

constexpr double EPSILON = 1e-6;
constexpr double NORMAL_EPS = 1e-5;
constexpr int MAX_STEPS = 256;

} // namespace

ImplicitSurface::ImplicitSurface(Scene *scene, Material *mat, Type type)
    : SceneObject(scene, mat), type_(type) {}

double ImplicitSurface::eval(glm::dvec3 p) const {
  const double x = p.x, y = p.y, z = p.z;
  switch (type_) {
  case Type::Sphere: {
    return x * x + y * y + z * z - radius_ * radius_;
  }
  case Type::Torus: {
    double q = std::sqrt(x * x + y * y) - major_radius_;
    return q * q + z * z - minor_radius_ * minor_radius_;
  }
  case Type::Goursat: {
    double r2 = x * x + y * y + z * z;
    return x * x * x * x + y * y * y * y + z * z * z * z - goursat_a_ * r2 * r2 +
           goursat_b_ * r2 - goursat_c_;
  }
  case Type::Blob: {
    // Wyvill soft object: f(r) = (1 - r²/b²)³ for r² < b², else 0
    // Isosurface at threshold T: sum f_i - T = 0
    const double b2 = 0.25; // b², so influence radius ~0.5
    const double T = 0.5;
    auto soft = [b2](double r2) {
      if (r2 >= b2)
        return 0.0;
      double t = 1.0 - r2 / b2;
      return t * t * t;
    };
    double r1_2 = glm::dot(p - blob_c1_, p - blob_c1_);
    double r2_2 = glm::dot(p - blob_c2_, p - blob_c2_);
    return soft(r1_2) + soft(r2_2) - T;
  }
  }
  return 0;
}

glm::dvec3 ImplicitSurface::gradient(glm::dvec3 p) const {
  const double h = NORMAL_EPS;
  double f0 = eval(p);
  return glm::dvec3((eval(p + glm::dvec3(h, 0, 0)) - f0) / h,
                    (eval(p + glm::dvec3(0, h, 0)) - f0) / h,
                    (eval(p + glm::dvec3(0, 0, h)) - f0) / h);
}

double ImplicitSurface::lipschitzConstant() const {
  switch (type_) {
  case Type::Sphere:
    return 2.0 * radius_;
  case Type::Torus:
    return 4.0 * (major_radius_ + minor_radius_);
  case Type::Goursat:
    return 8.0; // conservative bound for typical parameters
  case Type::Blob:
    return 4.0; // conservative for Wyvill
  }
  return 2.0;
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
      glm::dvec3 n = glm::normalize(gradient(p));
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
  BoundingBox localBox = ComputeLocalBoundingBox();
  if (!localBox.intersect(r, tMin, tMax))
    return false;

  if (tMin < RAY_EPSILON)
    tMin = RAY_EPSILON;

  return sphereTrace(r, i, tMin, tMax);
}

BoundingBox ImplicitSurface::ComputeLocalBoundingBox() {
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
    double r = 0.6;
    glm::dvec3 lo = glm::min(blob_c1_, blob_c2_) - glm::dvec3(r);
    glm::dvec3 hi = glm::max(blob_c1_, blob_c2_) + glm::dvec3(r);
    b.setMin(lo);
    b.setMax(hi);
    break;
  }
  }
  return b;
}

void ImplicitSurface::glDrawLocal(int quality, bool actualMaterials,
                                  bool actualTextures) const {
  (void)quality;
  (void)actualMaterials;
  (void)actualTextures;
  // Implicit surfaces are not easily drawn with OpenGL; skip
}
