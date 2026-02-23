#define _USE_MATH_DEFINES
#include <cmath>

#include "Portal.h"
#include "../scene/ray.h"

using namespace std;

bool Portal::intersectLocal(ray &r, isect &i) const {
  glm::dvec3 p = r.getPosition();
  glm::dvec3 d = r.getDirection();

  // disc lies in z=0 plane with normal (0,0,1), only accept front face (entering from -z)
  if (d.z >= 0.0) {
    return false; // ray parallel or hitting back
  }

  double t = -p.z / d.z;
  if (t <= RAY_EPSILON) {
    return false;
  }

  glm::dvec3 P = r.at(t);
  double distSq = P.x * P.x + P.y * P.y;
  if (distSq > radius_ * radius_) {
    return false;
  }

  i.setObject(this);
  i.setMaterial(this->getMaterial());
  i.setT(t);
  i.setN(glm::dvec3(0.0, 0.0, 1.0)); // local normal (front face)
  i.setUVCoordinates(glm::dvec2(P.x / radius_ * 0.5 + 0.5, P.y / radius_ * 0.5 + 0.5));
  return true;
}

void Portal::getFrame(glm::dvec3 &center, glm::dvec3 &right, glm::dvec3 &up,
                      glm::dvec3 &normal) const {
  center = transform.localToGlobalCoords(glm::dvec3(0, 0, 0));
  right =
      glm::normalize(transform.localToGlobalCoords(glm::dvec3(1, 0, 0)) - center);
  up = glm::normalize(transform.localToGlobalCoords(glm::dvec3(0, 1, 0)) - center);
  normal = glm::normalize(transform.localToGlobalCoordsNormal(glm::dvec3(0, 0, 1)));
}

bool Portal::computeExitRay(const glm::dvec3 &hitPoint,
                            const glm::dvec3 &rayDir, glm::dvec3 &exitOrigin,
                            glm::dvec3 &exitDir) const {
  if (!mapPointToPartner(hitPoint, exitOrigin) ||
      !mapDirectionToPartner(rayDir, exitDir)) {
    return false;
  }

  glm::dvec3 centerB, rightB, upB, normalB;
  partner_->getFrame(centerB, rightB, upB, normalB);
  exitOrigin += normalB * RAY_EPSILON;
  return true;
}

bool Portal::mapPointToPartner(const glm::dvec3 &point,
                               glm::dvec3 &mappedPoint) const {
  if (!partner_) {
    return false;
  }

  glm::dvec3 centerA, rightA, upA, normalA;
  glm::dvec3 centerB, rightB, upB, normalB;
  getFrame(centerA, rightA, upA, normalA);
  partner_->getFrame(centerB, rightB, upB, normalB);

  const glm::dvec3 rel = point - centerA;
  const double u = glm::dot(rel, rightA);
  const double v = glm::dot(rel, upA);
  const double w = glm::dot(rel, normalA);

  mappedPoint = centerB + u * rightB + v * upB - w * normalB;
  return true;
}

bool Portal::mapDirectionToPartner(const glm::dvec3 &dir,
                                   glm::dvec3 &mappedDir) const {
  if (!partner_) {
    return false;
  }

  glm::dvec3 centerA, rightA, upA, normalA;
  glm::dvec3 centerB, rightB, upB, normalB;
  getFrame(centerA, rightA, upA, normalA);
  partner_->getFrame(centerB, rightB, upB, normalB);

  const double dx = glm::dot(dir, rightA);
  const double dy = glm::dot(dir, upA);
  const double dz = glm::dot(dir, normalA);

  mappedDir = dx * rightB + dy * upB - dz * normalB;
  if (glm::length(mappedDir) < 1e-12) {
    return false;
  }
  mappedDir = glm::normalize(mappedDir);
  return true;
}

bool Portal::isPointInTeleportRegion(const glm::dvec3 &point) const {
  glm::dvec3 center, right, up, normal;
  getFrame(center, right, up, normal);

  const glm::dvec3 rel = point - center;
  const double u = glm::dot(rel, right);
  const double v = glm::dot(rel, up);
  const double w = glm::dot(rel, normal);

  const double radial2 = u * u + v * v;
  const double r2 = radius_ * radius_;
  constexpr double kEps = 1e-9;

  return (w < -kEps) && (radial2 <= r2 + kEps);
}

void Portal::glDrawLocal([[maybe_unused]] int quality,
                        [[maybe_unused]] bool actualMaterials,
                        [[maybe_unused]] bool actualTextures) const {
  // draw as a disc using GLU quadric
  GLUquadricObj *quadric = gluNewQuadric();
  gluQuadricDrawStyle(quadric, GLU_FILL);
  gluDisk(quadric, 0.0, radius_, 32, 1);
  gluDeleteQuadric(quadric);
}
