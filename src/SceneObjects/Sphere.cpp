#define _USE_MATH_DEFINES
#include <cmath>

#include "Sphere.h"
#include "../scene/ray.h"
#include <glm/gtx/io.hpp>
#include <iostream>

using namespace std;

bool Sphere::sampleSurfacePoint(double u, double v, glm::dvec3 &worldPoint,
                                glm::dvec3 &worldNormal,
                                double &worldArea) const {
  const double theta = 2.0 * M_PI * u;
  const double phi = std::acos(1.0 - 2.0 * v);
  const double sinPhi = std::sin(phi);
  const glm::dvec3 localPoint(sinPhi * std::cos(theta), sinPhi * std::sin(theta),
                              std::cos(phi));
  worldPoint = transform.localToGlobalCoords(localPoint);
  worldNormal = glm::normalize(transform.localToGlobalCoordsNormal(localPoint));
  const double scale = glm::length(glm::dvec3(transform.transform()[0]));
  worldArea = 4.0 * M_PI * scale * scale;
  return true;
}

bool Sphere::intersectLocal(ray &r, isect &i) const {
  r.setDirection(glm::normalize(r.getDirection()));
  glm::dvec3 v = -r.getPosition();
  double b = glm::dot(v, r.getDirection());
  double discriminant = b * b - glm::dot(v, v) + 1;

  if (discriminant < 0.0) {
    return false;
  }

  discriminant = sqrt(discriminant);
  double t2 = b + discriminant;

  if (t2 <= RAY_EPSILON) {
    return false;
  }

  i.setObject(this);
  i.setMaterial(this->getMaterial());

  double t1 = b - discriminant;

  if (t1 > RAY_EPSILON) {
    i.setT(t1);
    i.setN(glm::normalize(r.at(t1)));
  } else {
    i.setT(t2);
    i.setN(glm::normalize(r.at(t2)));
  }

  return true;
}
