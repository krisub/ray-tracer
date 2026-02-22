#ifndef __BRDF_H__
#define __BRDF_H__

#include <algorithm>
#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <random>

/**
 * - representations of physically-based BRDFs
 * - hemisphere sampling methods (cosine-weighted for diffuse, importance-sampled for specular)
 * - conversion from Phong shading parameters to BRDF form
 * - Monte Carlo integration utilities for the rendering eqn
 */

/**
 * The Phong model properties (Kd, Ks, shininess) are converted to:
 * - diffuse albedo: normalized diffuse component
 * - specular albedo: normalized specular component
 * - roughness: derived from Phong shininess exponent
 * - metallic factor: indicates if surface is dielectric or metallic (0 = dielectric)
 */
struct BRDF {
public:
  glm::dvec3 kd;        ///< diffuse albedo (range [0,1])
  glm::dvec3 ks;        ///< specular albedo (range [0,1])
  double roughness;     ///< material roughness (range [0,1]); derived from shininess
  double metallic;      ///< metallic factor (0 = dielectric, 1 = metallic)
  double fresnel_f0;    ///< Fresnel reflectance at viewing angle 0 degrees

  BRDF() 
      : kd(0.0), ks(0.0), roughness(0.5), metallic(0.0), fresnel_f0(0.04) {}

  BRDF(const glm::dvec3 &diffuse, const glm::dvec3 &specular, 
       double rough, double metal = 0.0)
      : kd(diffuse), ks(specular), roughness(rough), metallic(metal) {
    // fresnel_f0 for dielectrics is typically 0.04, for metals use specular color
    fresnel_f0 = (metal > 0.5) ? 0.5 : 0.04;
  }

  /**
   * converts Phong shading parameters to BRDF form
   *
   * - diffuse component (Kd) becomes diffuse albedo
   * - specular component (Ks) becomes specular albedo (with Fresnel consideration)
   * - Phong shininess exponent is converted to roughness: roughness ≈ 2.0 / (shininess + 2.0)
   *
   * @param kd_phong        diffuse component from Phong model
   * @param ks_phong        specular component from Phong model
   * @param shininess_phong Phong shininess exponent (higher = shinier)
   * @return BRDF          the converted BRDF parameters
   */
  static BRDF fromPhong(const glm::dvec3 &kd_phong, 
                        const glm::dvec3 &ks_phong, 
                        double shininess_phong) {
    // normalize albedos to ensure energy conservation
    // sum of diffuse + specular should typically be <= 1.0
    double max_component = std::max({kd_phong.x, kd_phong.y, kd_phong.z,
                                     ks_phong.x, ks_phong.y, ks_phong.z});
    glm::dvec3 kd = kd_phong;
    glm::dvec3 ks = ks_phong;
    
    if (max_component > 1.0) {
      kd /= max_component;
      ks /= max_component;
    }

    // convert Phong shininess to roughness
    // higher shininess -> lower roughness (shinier)
    // shininess=1 -> roughness=1.0 (very rough/matte)
    // shininess=128 -> roughness~0.12 (shiny)
    // formula: roughness = sqrt(2.0 / (shininess + 2.0))
    double shininess = std::max(1.0, shininess_phong);
    double roughness = std::sqrt(2.0 / (shininess + 2.0));

    BRDF brdf(kd, ks, roughness);
    return brdf;
  }
};

/**
 * sampling strategies:
 * cosine-weighted hemisphere: for diffuse BRDF importance sampling
 * uniform hemisphere: basic baseline
 * phong-weighted: importance sampling for Phong specular lobes
 */
class HemisphereSampler {
public:
  HemisphereSampler(unsigned int seed = std::random_device{}())
      : generator(seed), distribution(0.0, 1.0) {}

  /**
   * cosine-weighted hemisphere
   *
   * for a Lambertian (diffuse) BRDF
   * probability density function is proportional to cos(theta)
   * returns a direction in tangent space aligned with the surface normal
   * - sample two uniform random variables (u, v) in [0, 1]
   * - map to spherical coordinates: theta = acos(sqrt(u)), phi = 2*pi*v
   * - convert to Cartesian coordinates
   *
   * @param normal the surface normal (assumed to be unit vector)
   * @return glm::dvec3 a sampled direction in world space on the hemisphere
   */
  glm::dvec3 sampleCosineWeightedHemisphere(const glm::dvec3 &normal);

  /**
   * samples a direction on a uniform hemisphere
   *
   * basic sampling where all directions are equally likely.
   * PDF is 1/(2*pi).
   *
   * @param normal the surface normal
   * @return glm::dvec3 a sampled direction in world space
   */
  glm::dvec3 sampleUniformHemisphere(const glm::dvec3 &normal);

  /**
   * samples a direction with Phong specular distribution
   *
   * importance samples the Phong specular lobe
   * the reflection direction is perturbed based on the roughness parameter
   *
   * @param normal       the surface normal
   * @param reflection   the ideal reflection direction
   * @param roughness    roughness parameter (0 = perfectly shiny, 1 = very rough)
   * @return glm::dvec3  a sampled direction in world space
   */
  glm::dvec3 samplePhongSpecular(const glm::dvec3 &normal,
                                 const glm::dvec3 &reflection,
                                 double roughness);

  /**
   * computes the probability density of a direction under cosine-weighted hemisphere
   *
   * @param normal     the surface normal
   * @param direction  the direction to evaluate
   * @return double    the PDF value (cos(theta) / pi)
   */
  static double pdfCosineWeighted(const glm::dvec3 &normal,
                                  const glm::dvec3 &direction) {
    double cos_theta = glm::dot(normal, direction);
    if (cos_theta < 0.0)
      return 0.0;
    return cos_theta / M_PI;
  }

  /**
   * computes the PDF of a direction under uniform hemisphere
   *
   * @param normal     the surface normal
   * @param direction  the direction to evaluate
   * @return double    the PDF value (1 / (2*pi))
   */
  static double pdfUniformHemisphere(const glm::dvec3 &normal,
                                     const glm::dvec3 &direction) {
    if (glm::dot(normal, direction) < 0.0)
      return 0.0;
    return 1.0 / (2.0 * M_PI);
  }

private:
  std::mt19937 generator;                        ///< random number generator
  std::uniform_real_distribution<double> distribution; ///< uniform [0, 1] distribution

  /**
   * generates a random number in [0, 1]
   * @return double random value
   */
  double random() {
    return distribution(generator);
  }

  /**
   * builds an orthonormal basis with the given normal as the Z axis
   *
   * @param normal the Z axis
   * @param tangent  [output] the X axis (perpendicular to normal)
   * @param bitangent [output] the Y axis (perpendicular to both)
   */
  static void buildOrthonormalBasis(const glm::dvec3 &normal,
                                   glm::dvec3 &tangent,
                                   glm::dvec3 &bitangent) {
    // choose a vector that is not parallel to the normal
    glm::dvec3 up = (std::abs(normal.y) < 0.9) ? glm::dvec3(0.0, 1.0, 0.0)
                                                : glm::dvec3(1.0, 0.0, 0.0);
    tangent = glm::normalize(glm::cross(up, normal));
    bitangent = glm::cross(normal, tangent);
  }
};

#endif // __BRDF_H__
