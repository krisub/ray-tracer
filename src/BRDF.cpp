#include "BRDF.h"
#include <cmath>
#include <glm/glm.hpp>

glm::dvec3 HemisphereSampler::sampleCosineWeightedHemisphere(
    const glm::dvec3 &normal) {
  double u = random();
  double v = random();

  // convert to spherical coordinates using cosine-weighted distribution
  double theta = std::acos(std::sqrt(u));    // elevation angle
  double phi = 2.0 * M_PI * v;               // azimuth angle

  glm::dvec3 tangent, bitangent;
  buildOrthonormalBasis(normal, tangent, bitangent);

  double sin_theta = std::sin(theta);
  double cos_theta = std::cos(theta); // = sqrt(u)
  double cos_phi = std::cos(phi);
  double sin_phi = std::sin(phi);

  glm::dvec3 dir = sin_theta * cos_phi * tangent + sin_theta * sin_phi * bitangent +
                   cos_theta * normal;

  return glm::normalize(dir);
}

glm::dvec3 HemisphereSampler::sampleUniformHemisphere(
    const glm::dvec3 &normal) {
  // generate two uniform random numbers
  double u = random();
  double v = random();

  // uniform hemisphere sampling
  double theta = std::acos(u);    // elevation angle (u controls cos(theta))
  double phi = 2.0 * M_PI * v;    // azimuth angle

  glm::dvec3 tangent, bitangent;
  buildOrthonormalBasis(normal, tangent, bitangent);

  double sin_theta = std::sin(theta);
  double cos_theta = std::cos(theta);
  double cos_phi = std::cos(phi);
  double sin_phi = std::sin(phi);

  glm::dvec3 dir = sin_theta * cos_phi * tangent + sin_theta * sin_phi * bitangent +
                   cos_theta * normal;

  return glm::normalize(dir);
}

glm::dvec3 HemisphereSampler::samplePhongSpecular(const glm::dvec3 &normal,
                                                   const glm::dvec3 &reflection,
                                                   double roughness) {
  double u = random();
  double v = random();

  double shininess = 2.0 / (roughness * roughness) - 2.0;
  shininess = std::max(1.0, shininess);


  double theta = std::acos(std::pow(u, 1.0 / (shininess + 1.0)));
  double phi = 2.0 * M_PI * v;

  glm::dvec3 tangent, bitangent;
  buildOrthonormalBasis(reflection, tangent, bitangent);

  double sin_theta = std::sin(theta);
  double cos_theta = std::cos(theta);
  double cos_phi = std::cos(phi);
  double sin_phi = std::sin(phi);

  glm::dvec3 dir = sin_theta * cos_phi * tangent + sin_theta * sin_phi * bitangent +
                   cos_theta * reflection;

  return glm::normalize(dir);
}
