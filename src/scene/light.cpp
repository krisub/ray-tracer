#include <cmath>
#include <iostream>

#include "light.h"
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <vector>

using namespace std;
extern TraceUI *traceUI;

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
                                               const glm::dvec3 &p) const {
  if (traceUI->getHandleOverlap()) {
    return shadowAttenuationOverlap(r, surfaceNormal, p);
  }
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.
  glm::dvec3 lightAtten(1.0, 1.0, 1.0);
  glm::dvec3 dirToLight = getDirection(p);
  glm::dvec3 currPoint = p;

  while (true) {
    ray shadowRay(currPoint + dirToLight * RAY_EPSILON, dirToLight,
                  glm::dvec3(1.0, 1.0, 1.0), ray::SHADOW);
    isect i;

    if (scene->intersect(shadowRay, i)) {
      const Material &m = i.getMaterial();
      glm::dvec3 kt = m.kt(i);

      if (glm::length(kt) < 1e-6) {
        return glm::dvec3(0.0, 0.0, 0.0);
      }

      double direction = glm::dot(i.getN(), dirToLight);

      if (direction > 0) {
        // traveling out of the material
        // std::cout << "exit distance: " << i.getT() << " | Kt: " << kt.x <<
        // std::endl;
        double distance =
            i.getT(); // the last time we traveled was entering the material
        lightAtten *= glm::pow(kt, glm::dvec3(distance, distance, distance));
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
    const glm::dvec3 &p) const {
  glm::dvec3 lightAtten(1.0, 1.0, 1.0);
  glm::dvec3 dirToLight = getDirection(p);
  glm::dvec3 currPoint = p;
  std::vector<const SceneObject *> objStack;

  while (true) {
    ray shadowRay(currPoint + dirToLight * RAY_EPSILON, dirToLight,
                  glm::dvec3(1.0, 1.0, 1.0), ray::SHADOW);
    isect i;

    if (scene->intersect(shadowRay, i)) {
      double segmentDistance = i.getT();
      if (!objStack.empty()) {
        glm::dvec3 avgKt = averageKtFromObjects(objStack, i);
        lightAtten *=
            glm::pow(avgKt, glm::dvec3(segmentDistance, segmentDistance,
                                       segmentDistance));
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
                                         const glm::dvec3 &p) const {
  if (traceUI->getHandleOverlap()) {
    return shadowAttenuationOverlap(r, surfaceNormal, p);
  }
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.

  glm::dvec3 lightAtten(1.0, 1.0, 1.0);
  glm::dvec3 currPoint = p;

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

      glm::dvec3 kt = i.getMaterial().kt(i);
      if (glm::length(kt) < 1e-6) {
        return glm::dvec3(0.0, 0.0, 0.0);
      }

      double direction = glm::dot(i.getN(), dirToLight);

      if (direction > 0) {
        double distance = i.getT();
        lightAtten *= glm::pow(kt, glm::dvec3(distance, distance, distance));
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
    const glm::dvec3 &p) const {
  glm::dvec3 lightAtten(1.0, 1.0, 1.0);
  glm::dvec3 currPoint = p;
  std::vector<const SceneObject *> objStack;

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

      double segmentDistance = i.getT();
      if (!objStack.empty()) {
        glm::dvec3 avgKt = averageKtFromObjects(objStack, i);
        lightAtten *=
            glm::pow(avgKt, glm::dvec3(segmentDistance, segmentDistance,
                                       segmentDistance));
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
