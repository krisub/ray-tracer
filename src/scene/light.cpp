#include <cmath>
#include <iostream>

#include "light.h"
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>

using namespace std;

double DirectionalLight::distanceAttenuation(const glm::dvec3 &) const {
  // distance to light is infinite, so f(di) goes to 0.  Return 1.
  return 1.0;
}

glm::dvec3 DirectionalLight::shadowAttenuation(const ray &r,
                                               const glm::dvec3 &p) const {
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.

  glm::dvec3 dtl = getDirection(p); // dtl = direction to light
  // https://digitalrune.github.io/DigitalRune-Documentation/html/3f4d959e-9c98-4a97-8d85-7a73c26145d7.htm
  ray shadowRay(p + (RAY_EPSILON * dtl), dtl, glm::dvec3(1.0, 1.0, 1.0),
                ray::SHADOW);
  isect i;
  if (scene->intersect(shadowRay, i)) {
    return glm::dvec3(0.0, 0.0, 0.0); // black cuz shadow
  }

  return glm::dvec3(1.0, 1.0, 1.0); // light cuz not shadow
}

glm::dvec3 DirectionalLight::getColor() const { return color; }

glm::dvec3 DirectionalLight::getDirection(const glm::dvec3 &) const {
  return -orientation;
}

double PointLight::distanceAttenuation(const glm::dvec3 &P) const {
  // YOUR CODE HERE

  // You'll need to modify this method to attenuate the intensity
  // of the light based on the distance between the source and the
  // point P.  For now, we assume no attenuation and just return 1.0

  double dist = glm::distance(position, P);
  /* light.h
  // These three values are the a, b, and c in the distance attenuation function
  // (from the slide labelled "Intensity drop-off with distance"):
  //    f(d) = min( 1, 1/( a + b d + c d^2 ) )
  float constantTerm;  // a
  float linearTerm;    // b
  float quadraticTerm; // c
  */
  double atten_denom =
      /*1.0 /*/ (constantTerm + linearTerm * dist +
                 quadraticTerm * dist * dist);
  if (atten_denom < 1e6) {
    atten_denom = 1e6; // div by 0
  }

  double attenuation = 1.0 / atten_denom;

  return std::min(1.0, attenuation); // bw 0 and 1
}

glm::dvec3 PointLight::getColor() const { return color; }

glm::dvec3 PointLight::getDirection(const glm::dvec3 &P) const {
  return glm::normalize(position - P);
}

glm::dvec3 PointLight::shadowAttenuation(const ray &r,
                                         const glm::dvec3 &p) const {
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.
  glm::dvec3 dirToLight = position - p;
  double distToLight =
      glm::length(dirToLight); // overloading of dtl abbreviation :(
  dirToLight = glm::normalize(dirToLight);

  ray shadowRay(p + (RAY_EPSILON * dirToLight), dirToLight,
                glm::dvec3(1.0, 1.0, 1.0), ray::SHADOW);

  // POINT LIGHTS- only care if obj u hit is closer than light
  // if obj behind light doesnt matter
  isect i;
  if (scene->intersect(shadowRay, i)) {
    if (i.getT() < distToLight) {
      return glm::dvec3(0.0, 0.0, 0.0);
    }
  }

  return glm::dvec3(1, 1, 1);
}

#define VERBOSE 0
