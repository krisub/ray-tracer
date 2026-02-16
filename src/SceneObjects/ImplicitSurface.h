#ifndef __IMPLICITSURFACE_H__
#define __IMPLICITSURFACE_H__

#include "../scene/scene.h"

/**
 * Ray-traces Lipschitz-continuous implicit surfaces f(x,y,z) = 0
 * using sphere tracing (Kalra-Barr / Hart algorithm).
 *
 * The surface is defined in local (object) space. The Lipschitz constant L
 * bounds |∇f|, so we can safely step by |f(p)|/L along the ray without
 * overshooting the surface.
 */
class ImplicitSurface : public SceneObject {
public:
  enum class Type {
    Sphere,    // x²+y²+z² - r² = 0
    Torus,     // ((√(x²+y²)-R)² + z² - r² = 0
    Goursat,   // x⁴+y⁴+z⁴ - a(x²+y²+z²)² + b(x²+y²+z²) - c = 0
    Blob,      // Sum of two Wyvill soft objects (blobby/metaball)
  };

  ImplicitSurface(Scene *scene, Material *mat, Type type);

  virtual bool intersectLocal(ray &r, isect &i) const override;
  virtual bool hasBoundingBoxCapability() const override { return true; }
  virtual BoundingBox ComputeLocalBoundingBox() override;

  // Type-specific parameters (set after construction)
  void setRadius(double r) { radius_ = r; }
  void setTorusRadii(double majorR, double minorR) {
    major_radius_ = majorR;
    minor_radius_ = minorR;
  }
  void setGoursatParams(double a, double b, double c) {
    goursat_a_ = a;
    goursat_b_ = b;
    goursat_c_ = c;
  }
  void setBlobCenters(const glm::dvec3 &c1, const glm::dvec3 &c2) {
    blob_c1_ = c1;
    blob_c2_ = c2;
  }

protected:
  void glDrawLocal(int quality, bool actualMaterials,
                   bool actualTextures) const override;

private:
  Type type_;

  // Sphere
  double radius_ = 1.0;

  // Torus: major radius R (distance from tube center to torus center),
  //        minor radius r (tube radius)
  double major_radius_ = 0.5;
  double minor_radius_ = 0.2;

  // Goursat: x⁴+y⁴+z⁴ - a(x²+y²+z²)² + b(x²+y²+z²) - c = 0
  double goursat_a_ = 0.0;
  double goursat_b_ = 0.0;
  double goursat_c_ = 1.0;

  // Blob: two soft object centers
  glm::dvec3 blob_c1_{-0.3, 0, 0};
  glm::dvec3 blob_c2_{0.3, 0, 0};

  double eval(glm::dvec3 p) const;
  glm::dvec3 gradient(glm::dvec3 p) const;
  double lipschitzConstant() const;
  bool sphereTrace(ray &r, isect &i, double tMin, double tMax) const;
};

#endif
