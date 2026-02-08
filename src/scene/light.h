/**
 * @file light.h
 * @brief defines the abstract Light base class and specific light types
 *
 * class hierarchy for lighting in a scene:
 * - `Light`: abstract base class
 * - `DirectionalLight`: infinite distance light (Sun)
 * - `PointLight`: localized light source with falloff (light bulb)
 *
 * these classes are responsible for:
 * 1. Telling the shader how bright the light is at a given point
 * (`distanceAttenuation`)
 * 2. telling the shader if a point is in shadow (`shadowAttenuation`)
 *
 * mainly referenced by:
 * - `Scene`: stores a list of lights
 * - `Material::shade`: iterates over lights to calculate Phong shading
 */

#ifndef __LIGHT_H__
#define __LIGHT_H__

#ifndef _WIN32
#include <algorithm>
using std::max;
using std::min;
#endif

#include "../ui/TraceUI.h"
#include "scene.h"
#include <FL/gl.h>

/**
 * @class Light
 * @brief abstract base class for all light sources
 *
 * provides the interface that `Material::shade` uses to interact with lights.
 * inherits from `SceneElement` so it knows which Scene it belongs to.
 */
class Light : public SceneElement {
public:
  /**
   * @brief determines how much light reaches a point `P` (Shadow Test)
   *
   * this method casts a "shadow ray" from the point `P` towards the light
   * source. if the ray hits an object before reaching the light, the point is
   * in shadow.
   *
   * @param r   the primary ray (used for context, though often not needed for
   * simple shadows).
   * @param pos the surface point `P` being shaded.
   * @return glm::dvec3 the attenuation factor (1,1,1) for fully lit, (0,0,0)
   * for fully shadowed. colored shadows are possible if light passes through
   * transparent objects.
   */
  virtual glm::dvec3 shadowAttenuation(const ray &r,
                                       const glm::dvec3 &surfaceNormal,
                                       const glm::dvec3 &pos) const = 0;
  /**
   * @brief calculates light intensity falloff based on distance
   *
   * @param P the surface point being shaded
   * @return a scalar double value between 0.0 and 1.0 representing intensity
   */
  virtual double distanceAttenuation(const glm::dvec3 &P) const = 0;

  /**
   * @brief retrieves the light color/intensity
   * @return glm::dvec3 the color/intensity of the light
   */
  virtual glm::dvec3 getColor() const = 0;

  /**
   * @brief retrieves the direction from point P to the light source
   *
   * for directional lights, this is constant. for point lights, this varies
   * based on the position `P`.
   *
   * @param P the surface point being shaded
   * @return glm::dvec3 the (normalized) direction vector from `P` to the light
   */
  virtual glm::dvec3 getDirection(const glm::dvec3 &P) const = 0;

protected:
  /**
   * @brief Light constructor
   * @param scene pointer to the parent Scene
   * @param col the color/intensity of the light
   */
  Light(Scene *scene, const glm::dvec3 &col)
      : SceneElement(scene), color(col) {}

  glm::dvec3 color; ///< the light's base color/intensity

public:
  virtual void glDrawLight([[maybe_unused]] GLenum lightID) const {}
  virtual void glDrawLight() const {}
};

/**
 * @class DirectionalLight
 * @brief represents a light source infintely far away
 *
 * rays from this light are parallel, they have direction but no specific
 * position. distance attenuation is always 1.0 (no falloff).
 */
class DirectionalLight : public Light {
public:
  /**
   * @brief DirectionalLight constructor
   * @param scene pointer to parent Scene
   * @param orien the orientation/direction of the light rays (-Y would be
   * "down")
   * @param color the color/intensity of the light
   */
  DirectionalLight(Scene *scene, const glm::dvec3 &orien,
                   const glm::dvec3 &color)
      : Light(scene, color), orientation(glm::normalize(orien)) {}

  /** @brief checks shadows for infinite lights, casts a ray in the direction
   * `-orientation` to see if anything is blocking the light
   */
  virtual glm::dvec3 shadowAttenuation(const ray &r,
                                       const glm::dvec3 &surfaceNormal,
                                       const glm::dvec3 &pos) const;
  virtual glm::dvec3 shadowAttenuationOverlap(const ray &r,
                                              const glm::dvec3 &surfaceNormal,
                                              const glm::dvec3 &pos) const;

  /** @brief directional lights do not decay with distance. always 1.0. */
  virtual double distanceAttenuation(const glm::dvec3 &P) const;
  virtual glm::dvec3 getColor() const;

  /** @brief for directional lights, this is just the (constant) -orientation */
  virtual glm::dvec3 getDirection(const glm::dvec3 &P) const;

protected:
  glm::dvec3 orientation; ///< the (normalized) direction of the light rays

public:
  void glDrawLight(GLenum lightID) const;
  void glDrawLight() const;
};

/**
 * @class PointLight
 * @brief represents a light source at a specific position in space
 *
 * emits light in all directions from `position`
 * intensity decays based on the inverse-square law (or a custom quadratic
 * function)
 */
class PointLight : public Light {
public:
  /**
   * @brief PointLight constructor
   *
   * @param scene parent scene
   * @param pos   world space position
   * @param color light color
   * @param constantAttenuationTerm  'a' term
   * @param linearAttenuationTerm    'b' term
   * @param quadraticAttenuationTerm 'c' term
   */
  PointLight(Scene *scene, const glm::dvec3 &pos, const glm::dvec3 &color,
             float constantAttenuationTerm, float linearAttenuationTerm,
             float quadraticAttenuationTerm)
      : Light(scene, color), position(pos),
        constantTerm(constantAttenuationTerm),
        linearTerm(linearAttenuationTerm),
        quadraticTerm(quadraticAttenuationTerm) {}

  /**
   * @brief checks shadows for point lights by casting a ray from
   * `pos` to the light's `position` and checks if the intersection distance
   * is less than the distance to the light.
   *
   * @param r   the primary ray
   * @param pos the surface point `P` being shaded
   * @return glm::dvec3 the attenuation factor
   */
  virtual glm::dvec3 shadowAttenuation(const ray &r,
                                       const glm::dvec3 &surfaceNormal,
                                       const glm::dvec3 &pos) const;
  virtual glm::dvec3 shadowAttenuationOverlap(const ray &r,
                                              const glm::dvec3 &surfaceNormal,
                                              const glm::dvec3 &pos) const;
  /**
   * @brief calculates attentuation with the formula:
   * f(d) = min( 1, 1/( a + b d + c d^2 ) ) where d is the distance to the
   * light.
   */
  virtual double distanceAttenuation(const glm::dvec3 &P) const;
  virtual glm::dvec3 getColor() const;
  /**
   * @brief returns the dircetion from P to the light source.
   * @param P the surface point being shaded
   * @return glm::dvec3 the (normalized) direction vector from `P`
   */
  virtual glm::dvec3 getDirection(const glm::dvec3 &P) const;

  void setAttenuationConstants(float a, float b, float c) {
    constantTerm = a;
    linearTerm = b;
    quadraticTerm = c;
  }

protected:
  glm::dvec3 position;

  // These three values are the a, b, and c in the distance attenuation function
  // (from the slide labelled "Intensity drop-off with distance"):
  //    f(d) = min( 1, 1/( a + b d + c d^2 ) )
  float constantTerm;  ///< a (constant)
  float linearTerm;    ///< b (linear)
  float quadraticTerm; ///< c (quadratic)

public:
  void glDrawLight(GLenum lightID) const;
  void glDrawLight() const;

protected:
};

#endif // __LIGHT_H__
