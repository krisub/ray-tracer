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

bool Portal::computeExitRay(const glm::dvec3 &hitPoint,
                            const glm::dvec3 &rayDir, glm::dvec3 &exitOrigin,
                            glm::dvec3 &exitDir) const {
  if (!partner_) {
    return false;
  }

  // worldspace frame for this (entry) portal
  glm::dvec3 centerA = transform.localToGlobalCoords(glm::dvec3(0, 0, 0));
  glm::dvec3 rightA =
      glm::normalize(transform.localToGlobalCoords(glm::dvec3(1, 0, 0)) -
                     transform.localToGlobalCoords(glm::dvec3(0, 0, 0)));
  glm::dvec3 upA =
      glm::normalize(transform.localToGlobalCoords(glm::dvec3(0, 1, 0)) -
                     transform.localToGlobalCoords(glm::dvec3(0, 0, 0)));
  glm::dvec3 normalA =
      glm::normalize(transform.localToGlobalCoordsNormal(glm::dvec3(0, 0, 1)));

  // worldspace frame for partner (exit) portal
  glm::dvec3 centerB =
      partner_->transform.localToGlobalCoords(glm::dvec3(0, 0, 0));
  glm::dvec3 rightB =
      glm::normalize(partner_->transform.localToGlobalCoords(glm::dvec3(1, 0, 0)) -
                     partner_->transform.localToGlobalCoords(glm::dvec3(0, 0, 0)));
  glm::dvec3 upB =
      glm::normalize(partner_->transform.localToGlobalCoords(glm::dvec3(0, 1, 0)) -
                     partner_->transform.localToGlobalCoords(glm::dvec3(0, 0, 0)));
  glm::dvec3 normalB =
      glm::normalize(partner_->transform.localToGlobalCoordsNormal(glm::dvec3(0, 0, 1)));

  // local (u,v) on portal A
  double u = glm::dot(hitPoint - centerA, rightA);
  double v = glm::dot(hitPoint - centerA, upA);

  // exit point on portal B
  exitOrigin = centerB + u * rightB + v * upB;

  // direction: express in A's frame, transfer to B's frame
  // when exiting B, we go out the front (positive normalB)
  double dx = glm::dot(rayDir, rightA);
  double dy = glm::dot(rayDir, upA);
  double dz = glm::dot(rayDir, normalA);

  exitDir = dx * rightB + dy * upB - dz * normalB;
  exitDir = glm::normalize(exitDir);

  // offset slightly along exit normal to avoid self-intersection
  exitOrigin += normalB * RAY_EPSILON;

  return true;
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
