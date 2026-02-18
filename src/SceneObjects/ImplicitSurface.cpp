#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <iostream>

#include "ImplicitSurface.h"
#include "../scene/ray.h"

namespace {
constexpr double THRESHOLD = 1e-5;
constexpr double EPSILON = 1e-8; 
constexpr double NORMAL_EPS = 1e-6;
constexpr int MAX_STEPS = 1024;

double evalRiemann(const glm::dvec3 &p) {
  const double r = std::sqrt(p.x * p.x + p.y * p.y);
  double theta = std::atan2(p.y, p.x);
  if (theta < 0.0)
    theta += 2.0 * M_PI;
  return std::pow(r, 2.0 / 3.0) * std::sin((2.0 * theta) / 3.0) - p.z;
}

bool refineRiemannHit(const glm::dvec3 &o, const glm::dvec3 &d,
                      double &ta, double &tb) {
  double fa = evalRiemann(o + ta * d);
  double fb = evalRiemann(o + tb * d);
  if (fa == 0.0)
    return true;
  if (fb == 0.0) {
    ta = tb;
    return true;
  }
  if (fa * fb > 0.0)
    return false;

  for (int it = 0; it < 24; ++it) {
    const double tm = 0.5 * (ta + tb);
    const double fm = evalRiemann(o + tm * d);
    if (std::abs(fm) < 1e-6 || std::abs(tb - ta) < 1e-6) {
      ta = tm;
      return true;
    }
    if (fa * fm <= 0.0) {
      tb = tm;
      fb = fm;
    } else {
      ta = tm;
      fa = fm;
    }
  }

  ta = 0.5 * (ta + tb);
  return true;
}

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
  case Type::Riemann:
    return evalRiemann(p);
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
    // torus SDF gradient: analytical for efficiency
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
  case Type::Riemann:
    return 8.0; // not going to sphere trace but here anyway
  }
  return 4.0;
}

bool ImplicitSurface::harnackTrace(ray &r, isect &i, double tMin,
                                   double tMax) const {
  double t = tMin;
  const glm::dvec3 o = r.getPosition();
  const glm::dvec3 d = r.getDirection();
  
  // Prevent self-intersection by starting slightly off the surface
  if (t < 1e-3) t = 1e-3;

  for (int step = 0; step < MAX_STEPS && t < tMax; ++step) {
    const glm::dvec3 p = o + t * d;
    const double f = evalRiemann(p);

    // 1. Check for intersection
    // Only accept hit if we have moved at least a little (avoid self-shadowing)
    if (std::abs(f) < THRESHOLD) {
        if (step > 0 || t > 1e-2) {
            glm::dvec3 g = gradient(p);
            // Flip normal if necessary
            if (glm::dot(g, d) > 0) g = -g; 


            std::cout << "Riemann hit at t=" << t << ", f=" << f << ", grad=(" << g.x << ", " << g.y << ", " << g.z << ")" << std::endl;

            i.setObject(this);
            i.setMaterial(this->getMaterial());
            i.setT(t);
            i.setN(glm::normalize(g));
            i.setUVCoordinates(glm::dvec2(0.5, 0.5));
            return true;
        }
    }

    // 2. Harnack Ball Radius
    const double rCyl = std::sqrt(p.x * p.x + p.y * p.y);
    // Keep ball away from singularity at r=0
    const double R = std::max(1e-4, 0.5 * rCyl);

    // 3. Compute Lower Bound C
    double C;
    if (f >= 0.0) {
      C = -std::pow(rCyl + R, 2.0 / 3.0) - (p.z + R) - 0.5;
    } else {
      C = (p.z - R) - std::pow(rCyl + R, 2.0 / 3.0) - 0.5;
    }

    double stepSize;

    // 4. Determine Step Size
    if (C > 0.0) {
        // If lower bound is positive, the surface is NOT in this ball.
        // We can safely step nearly the full radius.
        stepSize = 0.9 * R;
    } else {
        // The surface might be in the ball. Use Harnack acceleration.
        // We target the zero of the positive harmonic function v = |f| - C
        const double Ucurr = std::abs(f) - C;
        const double Utarg = -C; // We want to reach |f|=0, so v = -C

        // Default fallback if math fails
        stepSize = 0.1 * R; 

        if (Utarg > 1e-12) {
            const double A = Ucurr / Utarg;
            if (A > 1.0) {
                const double num = 2.0 * R * (A - 1.0);
                const double den = A + 2.0 + std::sqrt(A * A + 8.0 * A);
                stepSize = num / den;
            }
        }
    }

    // 5. Clamping
    stepSize = std::clamp(stepSize, 1e-4, 2.0);

    const double tNext = t + stepSize;

    // 6. Check for overstep (Root Refinement)
    if (tNext < tMax) {
        const double fNext = evalRiemann(o + tNext * d);
        if (f * fNext < 0.0) {
            double ta = t; 
            double tb = tNext;
            if (refineRiemannHit(o, d, ta, tb)) {
                glm::dvec3 pRef = o + ta * d;
                glm::dvec3 g = gradient(pRef);
                if (glm::dot(g, d) > 0) g = -g;

                std::cout << "Riemann refined hit at t=" << ta
                  << ", grad=(" << g.x << ", " << g.y << ", " << g.z << ")"
                  << std::endl;

                i.setObject(this);
                i.setMaterial(this->getMaterial());
                i.setT(ta);
                i.setN(glm::normalize(g));
                i.setUVCoordinates(glm::dvec2(0.5, 0.5));
                return true;
            }
        }
    }

    t = tNext;
  }

  return false;
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

  if (type_ == Type::Riemann) {
    return harnackTrace(r, i, tMin, tMax);
  }

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
  case Type::Riemann: {
    double extent = 20.0;
    b.setMin(glm::dvec3(-extent, -extent, -extent));
    b.setMax(glm::dvec3(extent, extent, extent));
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
