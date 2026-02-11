#include "material.h"
#include "../ui/TraceUI.h"
#include "light.h"
#include "ray.h"
extern TraceUI *traceUI;

#include "../fileio/images.h"
#include <glm/gtx/io.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace std;
extern bool debugMode;

Material::~Material() {}

/*
get Emissive component (self-glowing light)
glm::dvec3 ke(const isect &i) const { return _ke.value(i); }

get Ambient component (interaction with background light)
glm::dvec3 ka(const isect &i) const { return _ka.value(i); }

get Specular component (shiny highlights)
glm::dvec3 ks(const isect &i) const { return _ks.value(i); }

get Diffuse component (matte surface color)
glm::dvec3 kd(const isect &i) const { return _kd.value(i); }

get Reflective component (mirror-like reflection)
glm::dvec3 kr(const isect &i) const { return _kr.value(i); }

get Transmissive component (refraction/transparency)
glm::dvec3 kt(const isect &i) const { return _kt.value(i); }
*/

// TODO CHANGE THIS TO MATCH THE SLIDES
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
    versa, skip if light is nothing) get shadow attentuation if in shadow skip
  other components diffuse component: lambert's cosine law: dot(surface normal,
  dir to light) specular component: reflection vector = 2 * dot(surface normal,
  dir to light) * surface normal - dir to light then dot(reflection vector,
  camera direction) ^ shininess
      - objects have shininess values. lower means higher scatter -> less
  highlight.
  */
  // std::cout << "shading at t=" << i.getT() << std::endl;
  glm::dvec3 hitPoint = r.at(i);
  glm::dvec3 surfaceNormal = glm::normalize(i.getN());
  glm::dvec3 cameraDir =
      glm::normalize(-r.getDirection()); // negative ray direction
  glm::dvec3 color = ke(i) + ka(i) * scene->ambient();

  for (const auto &pLight : scene->getAllLights()) {
    glm::dvec3 dirToLight = glm::normalize(
        pLight->getDirection(hitPoint)); // direction from SURFACE (from the
                                         // intersection/hit point) to light

    double distAtten = pLight->distanceAttenuation(hitPoint);
    if (distAtten < 1e-6)
      continue; // skip if light doesn't contribute anything

    glm::dvec3 shadowAtten =
        pLight->shadowAttenuation(r, surfaceNormal, hitPoint);
    if (shadowAtten.x + shadowAtten.y + shadowAtten.z < 1e-6)
      continue; // skip diffuse and specular if in shadow

    double diffuseDot = glm::dot(surfaceNormal, dirToLight);
    double diffuseReflection;

    // ADDED TO FIX CONE AND SPHERE AT THE SAME TIME
    if (kt(i).x > 1e-6 || kt(i).y > 1e-6 || kt(i).z > 1e-6) {
      diffuseReflection = glm::abs(diffuseDot);
    } else {
      diffuseReflection = std::max(0.0, diffuseDot);
    }
    
    // if (kd(i) != glm::dvec3(0, 0, 0))
    //   cout << "kd is " << kd(i) << endl;

    if (diffuseReflection > 0) {
      glm::dvec3 diffuse = kd(i) * pLight->getColor() * diffuseReflection;
      color += distAtten * shadowAtten * diffuse;
    }

    // only do specular if diffuse component exists
    glm::dvec3 effectiveNormal =
        (diffuseDot < 0) ? -surfaceNormal : surfaceNormal;
    glm::dvec3 reflectionVector = glm::normalize(
        (2.0 * glm::abs(diffuseDot) * effectiveNormal) - dirToLight);

    double specularReflection = glm::dot(reflectionVector, cameraDir);
    if (specularReflection > 0) {
      double shine = shininess(i);
      glm::dvec3 specular =
          ks(i) * pLight->getColor() * pow(specularReflection, shine);
      color += distAtten * shadowAtten * specular;
    }
  }

  return color;
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

  if (data.empty() || width <= 0 || height <= 0) {
    return glm::dvec3(1.0, 1.0, 1.0);
  }

  auto clamp01 = [](double v) {
    if (v < 0.0)
      return 0.0;
    if (v > 1.0)
      return 1.0;
    return v;
  };

  double u = clamp01(coord.x);
  double v = clamp01(coord.y);

  double x = u * (width - 1);
  double y = v * (height - 1);

  int x0 = static_cast<int>(floor(x));
  int y0 = static_cast<int>(floor(y));
  int x1 = std::min(x0 + 1, width - 1);
  int y1 = std::min(y0 + 1, height - 1);

  double dx = x - x0;
  double dy = y - y0;

  glm::dvec3 c00 = getPixelAt(x0, y0);
  glm::dvec3 c10 = getPixelAt(x1, y0);
  glm::dvec3 c01 = getPixelAt(x0, y1);
  glm::dvec3 c11 = getPixelAt(x1, y1);

  glm::dvec3 c0 = (1.0 - dx) * c00 + dx * c10;
  glm::dvec3 c1 = (1.0 - dx) * c01 + dx * c11;

  return (1.0 - dy) * c0 + dy * c1;
}

glm::dvec3 TextureMap::getPixelAt(int x, int y) const {
  // YOUR CODE HERE
  //
  // In order to add texture mapping support to the
  // raytracer, you need to implement this function.
  if (data.empty() || width <= 0 || height <= 0) {
    return glm::dvec3(1.0, 1.0, 1.0);
  }

  x = std::max(0, std::min(x, width - 1));
  y = std::max(0, std::min(y, height - 1));

  int rowStride = static_cast<int>(data.size() / height);
  int channels = (width > 0 && rowStride % width == 0) ? rowStride / width : 3;
  if (channels < 3)
    channels = 3;

  int idx = y * rowStride + x * channels;
  if (idx + 2 >= static_cast<int>(data.size())) {
    return glm::dvec3(1.0, 1.0, 1.0);
  }

  double r = static_cast<double>(data[idx]) / 255.0;
  double g = static_cast<double>(data[idx + 1]) / 255.0;
  double b = static_cast<double>(data[idx + 2]) / 255.0;

  return glm::dvec3(r, g, b);
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
