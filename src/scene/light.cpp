#include <cmath>
#include <iostream>

#include "../SceneObjects/Portal.h"
#include "light.h"
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <vector>

using namespace std;
extern TraceUI *traceUI;

// emission integrated along path [0,t]: int_0^t Le * Kt^(t-s) ds = Le * (Kt^t - 1)/ln(Kt)
static glm::dvec3 integratedEmissiveAlongPath(const glm::dvec3 &Le,
                                              const glm::dvec3 &Kt, double t) {
  if (t <= 0.0)
    return glm::dvec3(0.0, 0.0, 0.0);
  glm::dvec3 out(0.0, 0.0, 0.0);
  const double eps = 1e-6;
  for (int c = 0; c < 3; ++c) {
    double ktc = Kt[c], lec = Le[c];
    if (lec < eps)
      continue;
    if (ktc < eps)
      out[c] = 0.0;
    else if (ktc >= 1.0 - eps)
      out[c] = lec * t;
    else
      out[c] = lec * (std::pow(ktc, t) - 1.0) / std::log(ktc);
  }
  return out;
}

static glm::dvec3 averageKtFromObjects(
    const std::vector<const SceneObject *> &objStack, const isect &i) {
  if (objStack.empty()) {
    return glm::dvec3(1.0, 1.0, 1.0);
  }
  glm::dvec3 sum(0.0, 0.0, 0.0);
  for (const auto *obj : objStack) {
    sum += obj->getMaterial().kt(i);
  }
  return sum / static_cast<double>(objStack.size());
}

double DirectionalLight::distanceAttenuation(const glm::dvec3 &) const {
  // distance to light is infinite, so f(di) goes to 0.  Return 1.
  return 1.0;
}

/**
 * @brief shadow attenuation for direction light sources
 *
 * @param r ray hitting the point
 * @param surfaceNormal ADDED BY ME TO TEST SHADOW RAY DIRECTION, CAN REMOVE
 * @param p point to shade
 */
glm::dvec3 DirectionalLight::shadowAttenuation(const ray &r,
                                               const glm::dvec3 &surfaceNormal,
                                               const glm::dvec3 &p,
                                               glm::dvec3 *emissionOut) const {
  if (traceUI->getHandleOverlap()) {
    return shadowAttenuationOverlap(r, surfaceNormal, p, emissionOut);
  }
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.
  glm::dvec3 lightAtten(1.0, 1.0, 1.0);
  glm::dvec3 dirToLight = getDirection(p);
  glm::dvec3 currPoint = p;
  constexpr int maxPortalHops = 16;
  int portalHops = 0;

  while (true) {
    ray shadowRay(currPoint + dirToLight * RAY_EPSILON, dirToLight,
                  glm::dvec3(1.0, 1.0, 1.0), ray::SHADOW);
    isect i;

    if (scene->intersect(shadowRay, i)) {
      const Portal *portal = dynamic_cast<const Portal *>(i.getObject());
      if (portal && portal->getPartner() && portalHops < maxPortalHops) {
        glm::dvec3 exitOrigin, exitDir;
        if (portal->computeExitRay(shadowRay.at(i), dirToLight, exitOrigin,
                                   exitDir)) {
          currPoint = exitOrigin;
          dirToLight = exitDir;
          ++portalHops;
          continue;
        }
      }

      const Material &m = i.getMaterial();
      glm::dvec3 kt = m.kt(i);

      if (glm::length(kt) < 1e-6) {
        return glm::dvec3(0.0, 0.0, 0.0);
      }

      double direction = glm::dot(i.getN(), dirToLight);

      if (direction > 0) {
        // traveling out of the material
        double distance = i.getT();
        lightAtten *= glm::pow(kt, glm::dvec3(distance, distance, distance));
        if (emissionOut && traceUI->getGoop()) {
          glm::dvec3 Le = m.ke(i);
          if (Le.x > 1e-9 || Le.y > 1e-9 || Le.z > 1e-9)
            *emissionOut += integratedEmissiveAlongPath(Le, kt, distance);
        }
      }

      if (glm::length(lightAtten) < 1e-6) {
        return glm::dvec3(0.0, 0.0, 0.0);
      }

      currPoint = shadowRay.at(i.getT() + RAY_EPSILON);
    } else {
      break;
    }
  }

  return lightAtten;
}

glm::dvec3 DirectionalLight::shadowAttenuationOverlap(
    const ray &r, const glm::dvec3 &surfaceNormal,
    const glm::dvec3 &p, glm::dvec3 *emissionOut) const {
  glm::dvec3 lightAtten(1.0, 1.0, 1.0);
  glm::dvec3 dirToLight = getDirection(p);
  glm::dvec3 currPoint = p;
  std::vector<const SceneObject *> objStack;
  constexpr int maxPortalHops = 16;
  int portalHops = 0;

  while (true) {
    ray shadowRay(currPoint + dirToLight * RAY_EPSILON, dirToLight,
                  glm::dvec3(1.0, 1.0, 1.0), ray::SHADOW);
    isect i;

    if (scene->intersect(shadowRay, i)) {
      const Portal *portal = dynamic_cast<const Portal *>(i.getObject());
      if (portal && portal->getPartner() && portalHops < maxPortalHops) {
        glm::dvec3 exitOrigin, exitDir;
        if (portal->computeExitRay(shadowRay.at(i), dirToLight, exitOrigin,
                                   exitDir)) {
          currPoint = exitOrigin;
          dirToLight = exitDir;
          ++portalHops;
          continue;
        }
      }

      double segmentDistance = i.getT();
      if (!objStack.empty()) {
        glm::dvec3 avgKt = averageKtFromObjects(objStack, i);
        lightAtten *=
            glm::pow(avgKt, glm::dvec3(segmentDistance, segmentDistance,
                                       segmentDistance));
        if (emissionOut && traceUI->getGoop()) {
          for (const SceneObject *obj : objStack) {
            const Material &mat = obj->getMaterial();
            glm::dvec3 Le = mat.ke(i), Kt = mat.kt(i);
            if (Le.x > 1e-9 || Le.y > 1e-9 || Le.z > 1e-9)
              *emissionOut += integratedEmissiveAlongPath(Le, Kt, segmentDistance);
          }
        }
      }

      if (glm::length(lightAtten) < 1e-6) {
        return glm::dvec3(0.0, 0.0, 0.0);
      }

      glm::dvec3 kt = i.getMaterial().kt(i);
      if (glm::length(kt) < 1e-6) {
        return glm::dvec3(0.0, 0.0, 0.0);
      }

      const SceneObject *hitObject = i.getObject();
      bool foundInStack = false;
      int foundIndex = -1;
      for (int k = static_cast<int>(objStack.size()) - 1; k >= 0; --k) {
        if (objStack[k] == hitObject) {
          foundInStack = true;
          foundIndex = k;
          break;
        }
      }

      if (!foundInStack) {
        objStack.push_back(hitObject);
      } else if (foundIndex >= 0) {
        objStack.erase(objStack.begin() + foundIndex);
      }

      currPoint = shadowRay.at(i.getT() + RAY_EPSILON);
    } else {
      break;
    }
  }

  return lightAtten;
}

glm::dvec3 DirectionalLight::getColor() const { return color; }

glm::dvec3 DirectionalLight::getDirection(const glm::dvec3 &) const {
  return -orientation;
}

/**
 * REWRITTEN VERSION
 */
double PointLight::distanceAttenuation(const glm::dvec3 &P) const {
  // YOUR CODE HERE

  // You'll need to modify this method to attenuate the intensity
  // of the light based on the distance between the source and the
  // point P.  For now, we assume no attenuation and just return 1.0
  double distToLight = glm::distance(position, P);

  double quadratic = constantTerm + linearTerm * distToLight +
                     quadraticTerm * distToLight * distToLight;
  if (quadratic < 1e-6)
    return 1.0;

  return min(1.0, 1.0 / quadratic);
  // return 1.0 / quadratic;
}

glm::dvec3 PointLight::getColor() const { return color; }

glm::dvec3 PointLight::getDirection(const glm::dvec3 &P) const {
  return glm::normalize(position - P);
}

/**
 * REWRITTEN VERSION
 */
glm::dvec3 PointLight::shadowAttenuation(const ray &r,
                                         const glm::dvec3 &surfaceNormal,
                                         const glm::dvec3 &p,
                                         glm::dvec3 *emissionOut) const {
  if (traceUI->getHandleOverlap()) {
    return shadowAttenuationOverlap(r, surfaceNormal, p, emissionOut);
  }
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.

  glm::dvec3 lightAtten(1.0, 1.0, 1.0);
  glm::dvec3 currPoint = p;
  constexpr int maxPortalHops = 16;
  int portalHops = 0;

  while (true) {
    glm::dvec3 pointToLight =
        position - currPoint; // vector from point to light
    double distToLight = glm::length(pointToLight);
    glm::dvec3 dirToLight = glm::normalize(pointToLight);

    ray shadowRay(currPoint + dirToLight * RAY_EPSILON, dirToLight,
                  glm::dvec3(1.0, 1.0, 1.0), ray::SHADOW);
    isect i;

    if (scene->intersect(shadowRay, i)) {
      if (i.getT() >= distToLight - RAY_EPSILON) {
        break; // behind light
      }

      const Portal *portal = dynamic_cast<const Portal *>(i.getObject());
      if (portal && portal->getPartner() && portalHops < maxPortalHops) {
        glm::dvec3 exitOrigin, exitDir;
        if (portal->computeExitRay(shadowRay.at(i), dirToLight, exitOrigin,
                                   exitDir)) {
          currPoint = exitOrigin;
          ++portalHops;
          continue;
        }
      }

      glm::dvec3 kt = i.getMaterial().kt(i);
      if (glm::length(kt) < 1e-6) {
        return glm::dvec3(0.0, 0.0, 0.0);
      }

      double direction = glm::dot(i.getN(), dirToLight);

      if (direction > 0) {
        double distance = i.getT();
        lightAtten *= glm::pow(kt, glm::dvec3(distance, distance, distance));
        if (emissionOut && traceUI->getGoop()) {
          glm::dvec3 Le = i.getMaterial().ke(i);
          if (Le.x > 1e-9 || Le.y > 1e-9 || Le.z > 1e-9)
            *emissionOut += integratedEmissiveAlongPath(Le, kt, distance);
        }
      }

      if (glm::length(lightAtten) < 1e-6) {
        return glm::dvec3(0.0, 0.0, 0.0);
      }

      currPoint = shadowRay.at(i.getT() + RAY_EPSILON);
    } else {
      break;
    }
  }

  return lightAtten;
}

glm::dvec3 PointLight::shadowAttenuationOverlap(
    const ray &r, const glm::dvec3 &surfaceNormal,
    const glm::dvec3 &p, glm::dvec3 *emissionOut) const {
  glm::dvec3 lightAtten(1.0, 1.0, 1.0);
  glm::dvec3 currPoint = p;
  std::vector<const SceneObject *> objStack;
  constexpr int maxPortalHops = 16;
  int portalHops = 0;

  while (true) {
    glm::dvec3 pointToLight = position - currPoint;
    double distToLight = glm::length(pointToLight);
    glm::dvec3 dirToLight = glm::normalize(pointToLight);

    ray shadowRay(currPoint + dirToLight * RAY_EPSILON, dirToLight,
                  glm::dvec3(1.0, 1.0, 1.0), ray::SHADOW);
    isect i;

    if (scene->intersect(shadowRay, i)) {
      if (i.getT() >= distToLight - RAY_EPSILON) {
        break;
      }

      const Portal *portal = dynamic_cast<const Portal *>(i.getObject());
      if (portal && portal->getPartner() && portalHops < maxPortalHops) {
        glm::dvec3 exitOrigin, exitDir;
        if (portal->computeExitRay(shadowRay.at(i), dirToLight, exitOrigin,
                                   exitDir)) {
          currPoint = exitOrigin;
          ++portalHops;
          continue;
        }
      }

      double segmentDistance = i.getT();
      if (!objStack.empty()) {
        glm::dvec3 avgKt = averageKtFromObjects(objStack, i);
        lightAtten *=
            glm::pow(avgKt, glm::dvec3(segmentDistance, segmentDistance,
                                       segmentDistance));
        if (emissionOut && traceUI->getGoop()) {
          for (const SceneObject *obj : objStack) {
            const Material &mat = obj->getMaterial();
            glm::dvec3 Le = mat.ke(i), Kt = mat.kt(i);
            if (Le.x > 1e-9 || Le.y > 1e-9 || Le.z > 1e-9)
              *emissionOut += integratedEmissiveAlongPath(Le, Kt, segmentDistance);
          }
        }
      }

      if (glm::length(lightAtten) < 1e-6) {
        return glm::dvec3(0.0, 0.0, 0.0);
      }

      glm::dvec3 kt = i.getMaterial().kt(i);
      if (glm::length(kt) < 1e-6) {
        return glm::dvec3(0.0, 0.0, 0.0);
      }

      const SceneObject *hitObject = i.getObject();
      bool foundInStack = false;
      int foundIndex = -1;
      for (int k = static_cast<int>(objStack.size()) - 1; k >= 0; --k) {
        if (objStack[k] == hitObject) {
          foundInStack = true;
          foundIndex = k;
          break;
        }
      }

      if (!foundInStack) {
        objStack.push_back(hitObject);
      } else if (foundIndex >= 0) {
        objStack.erase(objStack.begin() + foundIndex);
      }

      if (glm::length(lightAtten) < 1e-6) {
        return glm::dvec3(0.0, 0.0, 0.0);
      }

      currPoint = shadowRay.at(i.getT() + RAY_EPSILON);
    } else {
      break;
    }
  }

  return lightAtten;
}

#define VERBOSE 0
