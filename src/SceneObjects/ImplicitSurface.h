#ifndef __IMPLICITSURFACE_H__
#define __IMPLICITSURFACE_H__

#include "../scene/scene.h"

/**
 * ray tracing Lipschitz-continuous implicit surfaces f(x,y,z) = 0
 * using sphere tracing (Kalra-Barr / Hart algorithm)
 *
 * surface is defined in local (object) space, the Lipschitz constant L
 * bounds |change in f(p)|, so we can safely step by |f(p)|/L along the ray without
 * overshooting the surface
 */
class ImplicitSurface : public SceneObject {
public:
  enum class Type {
    Sphere,    // x^2+y^2+z^2 - r^2 = 0
    Torus,     // ((sqrt(x^2+y^2)-R)^2 + z^2 - r^2 = 0
    Goursat,   // x^4+y^4+z^4 - a(x^2+y^2+z^2)^2 + b(x^2+y^2+z^2) - c = 0
    Blob,      // sum of two Wyvill soft objects (blobby/metaball)
    Riemann,   // example of a non-Lipschitz surface with a cusp: z^2 - x^3 - y^3 = 0 (Im(z^(2/3))-z=0 - Harnack tracing)
  };

  ImplicitSurface(Scene *scene, Material *mat, Type type);

  virtual bool intersectLocal(ray &r, isect &i) const override;
  virtual bool hasBoundingBoxCapability() const override { return true; }
  virtual BoundingBox ComputeLocalBoundingBox() override;

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
  void setBlobRadii(double r1, double r2) {
    blob_radius1_ = r1;
    blob_radius2_ = r2;
  }
  /** isosurface threshold: higher = thinner connection (and smaller blobs), lower = fatter. Default 0.5. */
  void setBlobThreshold(double T) { blob_threshold_ = T; }

  void setLipschitzConstant(double L) { lipschitz_override_ = L; }

  void setRiemannFraction(int n, int d) {
    riemann_n = n;
    riemann_d = d;
  }

protected:
  void glDrawLocal(int quality, bool actualMaterials,
                   bool actualTextures) const override;

private:
  Type type_;

  // sphere
  double radius_ = 1.0;

  // torus: major radius R (distance from tube center to torus center),
  //        minor radius r (tube radius)
  double major_radius_ = 0.5;
  double minor_radius_ = 0.2;

  // goursat: x^4+y^4+z^4 - a(x^2+y^2+z^2)^2 + b(x^2+y^2+z^2) - c = 0
  double goursat_a_ = 0.0;
  double goursat_b_ = 0.0;
  double goursat_c_ = 1.0;

  // blob: two soft object centers and influence radii (Wyvill b)
  glm::dvec3 blob_c1_{-0.3, 0, 0};
  glm::dvec3 blob_c2_{0.3, 0, 0};
  double blob_radius1_ = 0.5;
  double blob_radius2_ = 0.5;
  double blob_threshold_ = 0.5;  // isosurface sum(f_i) = T

  // default riemann fraction is gonna be 2/3
  int riemann_n = 2;
  int riemann_d = 3;

  double lipschitz_override_ = 0.0;

  double eval(glm::dvec3 p) const;
  glm::dvec3 gradient(glm::dvec3 p) const;
  double lipschitzConstant() const;
  bool sphereTrace(ray &r, isect &i, double tMin, double tMax) const;
  BoundingBox getLocalBoundingBoxImpl() const;
  bool harnackTrace(ray &r, isect &i, double tMin, double tMax) const;
};

#endif
