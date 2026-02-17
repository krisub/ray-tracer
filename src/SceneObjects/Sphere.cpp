#define _USE_MATH_DEFINES
#include <cmath>

#include "Sphere.h"
#include "../scene/ray.h"
#include <glm/gtx/io.hpp>
#include <iostream>

using namespace std;

namespace {
// unit sphere: P(theta, phi) = (sin phi cos theta, sin phi sin theta, cos phi)
// u = longitude, v = latitude
void sphereUVAndTangentFrame(const glm::dvec3 &localHit, double &u, double &v,
                             glm::dvec3 &dPdu, glm::dvec3 &dPdv) {
  double x = localHit.x, y = localHit.y, z = localHit.z;
  double phi = std::acos(glm::clamp(z, -1.0, 1.0));  // [0, pi], 0 = +Z
  double theta = std::atan2(y, x);                     // [-pi, pi]
  u = theta / (2.0 * M_PI);  // longitude -> position along each stripe
  if (u < 0.0) u += 1.0;
  v = 1.0 - phi / M_PI;  // latitude -> which horizontal band/stripe
  double sinPhi = std::sin(phi);
  double cosPhi = std::cos(phi);
  double sinTheta = std::sin(theta);
  double cosTheta = std::cos(theta);
  if (sinPhi < 1e-10) sinPhi = 1e-10;
  // dP/du: dtheta/du = 2pi. dP/dv: v = 1 - phi/pi => dphi/dv = -pi
  dPdu = glm::dvec3(-sinPhi * sinTheta, sinPhi * cosTheta, 0.0) * (2.0 * M_PI);
  dPdv = glm::dvec3(cosPhi * cosTheta, cosPhi * sinTheta, -sinPhi) * (-M_PI);
}
}  // namespace

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
  double t = (t1 > RAY_EPSILON) ? t1 : t2;

  i.setT(t);
  glm::dvec3 localHit = glm::normalize(r.at(t));
  i.setN(localHit);

  // Spherical UV mapping for texture coordinates
  double u = 0.5 + std::atan2(localHit.y, localHit.x) / (2.0 * M_PI);
  double texV = std::acos(std::max(-1.0, std::min(1.0, localHit.z))) / M_PI;
  i.setUVCoordinates(glm::dvec2(u, texV));

  return true;
}
