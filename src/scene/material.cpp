#include "material.h"
#include "../ui/TraceUI.h"
#include "light.h"
#include "ray.h"
extern TraceUI *traceUI;

#include "../fileio/images.h"
#include <glm/gtx/io.hpp>
#include <iostream>

using namespace std;
extern bool debugMode;

Material::~Material() {}

// Apply the phong model to this point on the surface of the object, returning
// the color of that point.
glm::dvec3 Material::shade(Scene *scene, const ray &r, const isect &i) const {
  // YOUR CODE HERE

  // For now, this method just returns the diffuse color of the object.
  // This gives a single matte color for every distinct surface in the
  // scene, and that's it.  Simple, but enough to get you started.
  // (It's also inconsistent with the phong model...)

  // Your mission is to fill in this method with the rest of the phong
  // shading model, including the contributions of all the light sources.
  // You will need to call both distanceAttenuation() and
  // shadowAttenuation()
  // somewhere in your code in order to compute shadows and light falloff.
  //	if( debugMode )
  //		std::cout << "Debugging Phong code..." << std::endl;

  // When you're iterating through the lights,
  // you'll want to use code that looks something
  // like this:
  //
  // for ( const auto& pLight : scene->getAllLights() )
  // {
  //              // pLight has type Light*
  // 		.
  // 		.
  // 		.
  // }
  // https://learnopengl.com/Lighting/Basic-Lighting
  /*
  final color is emissive + ambient +
    diffuse for each light + specular for each light

  emissive- obj glows on its own
  ambient- bg
  diffuse- light scatter equally all direxns
  specular- light reflect in specific direxn

  need intersection point, surface normal, where the camera is
  then: can get emissive and ambient (base color) with ke, ka * scene->ambient

  ka * scene ambient -> shadows aren't completely black

  atp the color is the minimum visible color

  loop over lights like in comment:
  for each light:
    get direction from surface to light bc light depends on angle btwn light and
    surface normal get distance attentuation (further away light is dim and vice
    versa, skip if light is nothing) get shadow attentuation if in shadow skip other components
    diffuse component: lambert's cosine law: dot(surface normal, dir to light)
    specular component: reflection vector = 2 * dot(surface normal, dir to light) *
    surface normal - dir to light
    then dot(reflection vector, camera direction) ^ shininess
      - objects have shininess values. lower means higher scatter -> less highlight.
  */
  glm::dvec3 hitPoint = r.at(i);
  glm::dvec3 surfaceNormal = i.getN();
  glm::dvec3 cameraDir =
      glm::normalize(-r.getDirection()); // negative ray direction
  glm::dvec3 color = ke(i) + ka(i) * scene->ambient();

  for (const auto &pLight : scene->getAllLights()) {
    glm::dvec3 dirToLight =
        pLight->getDirection(hitPoint); // direction from SURFACE (from the
                                        // intersection/hit point) to light

    double distAtten = pLight->distanceAttenuation(hitPoint);
    if (distAtten < 1e-6)
      continue; // skip if light doesn't contribute anything

    glm::dvec3 shadowAtten = pLight->shadowAttenuation(r, hitPoint);
    if (shadowAtten.x + shadowAtten.y + shadowAtten.z < 1e-6)
      continue; // skip diffuse and specular if in shadow

    double diffuseReflection = glm::dot(surfaceNormal, dirToLight);
    if (diffuseReflection > 0) {
      glm::dvec3 diffuse = kd(i) * pLight->getColor() * diffuseReflection;
      color += distAtten * shadowAtten * diffuse;

      // only do specular if diffuse component exists
      glm::dvec3 reflectionVector = glm::normalize(
          (2.0 * diffuseReflection * surfaceNormal) - dirToLight);
      double specularReflection = glm::dot(reflectionVector, cameraDir);
      if (specularReflection > 0) {
        double shine = shininess(i);
        glm::dvec3 specular =
            ks(i) * pLight->getColor() * pow(specularReflection, shine);
        color += distAtten * shadowAtten * specular;
      }
    }
  }

  // return kd(i);
  return glm::clamp(color, glm::dvec3(0.0, 0.0, 0.0),
                        glm::dvec3(1.0, 1.0, 1.0));
}

TextureMap::TextureMap(string filename) {
  data = readImage(filename.c_str(), width, height);
  if (data.empty()) {
    width = 0;
    height = 0;
    string error("Unable to load texture map '");
    error.append(filename);
    error.append("'.");
    throw TextureMapException(error);
  }
}

glm::dvec3 TextureMap::getMappedValue(const glm::dvec2 &coord) const {
  // YOUR CODE HERE
  //
  // In order to add texture mapping support to the
  // raytracer, you need to implement this function.
  // What this function should do is convert from
  // parametric space which is the unit square
  // [0, 1] x [0, 1] in 2-space to bitmap coordinates,
  // and use these to perform bilinear interpolation
  // of the values.

  return glm::dvec3(1, 1, 1);
}

glm::dvec3 TextureMap::getPixelAt(int x, int y) const {
  // YOUR CODE HERE
  //
  // In order to add texture mapping support to the
  // raytracer, you need to implement this function.

  return glm::dvec3(1, 1, 1);
}

glm::dvec3 MaterialParameter::value(const isect &is) const {
  if (0 != _textureMap)
    return _textureMap->getMappedValue(is.getUVCoordinates());
  else
    return _value;
}

double MaterialParameter::intensityValue(const isect &is) const {
  if (0 != _textureMap) {
    glm::dvec3 value(_textureMap->getMappedValue(is.getUVCoordinates()));
    return (0.299 * value[0]) + (0.587 * value[1]) + (0.114 * value[2]);
  } else
    return (0.299 * _value[0]) + (0.587 * _value[1]) + (0.114 * _value[2]);
}
