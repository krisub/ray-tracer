// The main ray tracer.

#pragma warning(disable : 4786)

#include "RayTracer.h"
#include "SceneObjects/Portal.h"
#include "interp.h"
#include "scene/light.h"
#include "scene/material.h"
#include "scene/ray.h"
#include "scene/scene.h"
#include <omp.h>

#include "parser/JsonParser.h"
#include "parser/Parser.h"
#include "parser/Tokenizer.h"
#include <json.hpp>

#include "ui/TraceUI.h"
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <glm/gtc/noise.hpp>
#include <string.h>

#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <utility>

#include <unistd.h> 
#include <limits.h>
#include <onnxruntime_cxx_api.h> // aaNeuralImage

using namespace std;
extern TraceUI *traceUI;

namespace {

inline glm::dvec3 schlickFresnel(double cosTheta, const glm::dvec3 &F0) {
  double clampedCos = glm::clamp(cosTheta, 0.0, 1.0);
  double t = 1.0 - clampedCos;
  constexpr double kRecursiveFresnelPower = 5.0;
  return F0 + (glm::dvec3(1.0) - F0) * std::pow(t, kRecursiveFresnelPower);
}

inline glm::dvec3 computeF0(const Material &m, const isect &i) {
  glm::dvec3 ks = glm::clamp(m.ks(i), glm::dvec3(0.0), glm::dvec3(1.0));
  double metallic = glm::clamp(m.metallic(), 0.0, 1.0);
  double ksLum = 0.2126 * ks.x + 0.7152 * ks.y + 0.0722 * ks.z;
  glm::dvec3 dielectricF0(0.02 + 0.12 * ksLum);
  return glm::mix(dielectricF0, ks, metallic);
}

inline glm::dvec3 recursiveFresnelFactor(const ray &r, const isect &i,
                                         const Material &m) {
  glm::dvec3 V = glm::normalize(-r.getDirection());
  glm::dvec3 N = i.getN();
  if (glm::dot(N, V) < 0.0) {
    N = -N;
  }
  double cosNV = glm::clamp(glm::dot(N, V), 0.0, 1.0);
  return schlickFresnel(cosNV, computeF0(m, i));
}

inline glm::dvec3 recursiveRimWeight(const glm::dvec3 &F,
                                     const glm::dvec3 &F0) {
  glm::dvec3 denom =
      glm::max(glm::dvec3(1e-6), glm::dvec3(1.0) - glm::clamp(F0, glm::dvec3(0.0), glm::dvec3(1.0)));
  glm::dvec3 rim = glm::clamp((F - F0) / denom, glm::dvec3(0.0), glm::dvec3(1.0));
  return glm::pow(rim, glm::dvec3(1.6));
}

} // namespace

// Use this variable to decide if you want to print out debugging messages. Gets
// set in the "trace single ray" mode in TraceGLWindow, for example.
bool debugMode = false;

glm::dvec3
RayTracer::computeReflectionDirection(const glm::dvec3 &incidentDir,
                                      const glm::dvec3 &surfaceNormal) {
  double incidentNormalDot = glm::dot(surfaceNormal, incidentDir);
  return incidentDir - 2.0 * incidentNormalDot * surfaceNormal;
}

glm::dvec3
RayTracer::computeRefractionDirection(const glm::dvec3 &incidentDir,
                                      const glm::dvec3 &surfaceNormal,
                                      double iorRatio) {
  double cosIncident = glm::dot(surfaceNormal, incidentDir);
  double sin2Transmitted =
      iorRatio * iorRatio * (1.0 - cosIncident * cosIncident);
  if (sin2Transmitted > 1.0) {
    return glm::dvec3(0.0, 0.0, 0.0);
  }

  double cosTransmitted = sqrt(1.0 - sin2Transmitted);
  return iorRatio * incidentDir -
         (iorRatio * cosIncident + cosTransmitted) * surfaceNormal;
}

// emission integrated along path [0,t]: int_0^t Le * Kt^(t-s) ds = Le * (Kt^t - 1)/ln(Kt); limit Le*t as Kt->1.
glm::dvec3 RayTracer::integratedEmissiveAlongPath(const glm::dvec3 &Le,
                                                  const glm::dvec3 &Kt,
                                                  double t) {
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

void RayTracer::accumulateReflection(ray &r, const isect &i,
                                     const glm::dvec3 &Kr,
                                     const glm::dvec3 &thresh, int depth,
                                     glm::dvec3 &colorC) {
  // adaptive termination: skip reflection when max(Kr) < thresh (thresh=0 disables)
  if (thresh.x > 0 && std::max({Kr.x, Kr.y, Kr.z}) < thresh.x)
    return;
  if (Kr.x > 0 || Kr.y > 0 || Kr.z > 0) {
    const glm::dvec3 &rayDir = r.getDirection();   // already unit (camera/reflect/refract)
    const glm::dvec3 &surfaceNormal = i.getN();    // already unit (intersection code)
    glm::dvec3 reflectDir = computeReflectionDirection(rayDir, surfaceNormal);

    ray reflectedRay(r.at(i) + reflectDir * RAY_EPSILON, reflectDir,
                     glm::dvec3(1.0, 1.0, 1.0), ray::REFLECTION);

    double t_val;
    colorC += Kr * traceRay(reflectedRay, thresh, depth - 1, t_val);
  }
}

void RayTracer::accumulateRefraction(ray &r, const isect &i, const Material &m,
                                     const glm::dvec3 &Kt,
                                     const glm::dvec3 &thresh, int depth,
                                     glm::dvec3 &colorC) {
  // adaptive termination: skip refraction when max(Kt) < thresh (thresh=0 disables)
  if (thresh.x > 0 && std::max({Kt.x, Kt.y, Kt.z}) < thresh.x)
    return;
  if (Kt.x > 0 || Kt.y > 0 || Kt.z > 0) {
    const glm::dvec3 &rayDir = r.getDirection();
    const glm::dvec3 &surfaceNormal = i.getN();
    double indexOfRefraction = m.index(i);

    double cosIncidence = glm::dot(surfaceNormal, rayDir);
    double iorRatio;
    glm::dvec3 normalForRefraction;

    if (cosIncidence < 0) {
      iorRatio = 1.0 / indexOfRefraction;
      normalForRefraction = surfaceNormal;
    } else {
      iorRatio = indexOfRefraction / 1.0;
      normalForRefraction = -surfaceNormal;
    }

    glm::dvec3 refractionDir =
        computeRefractionDirection(rayDir, normalForRefraction, iorRatio);

    if (glm::length(refractionDir) > 0.0) {
      ray refractedRay(r.at(i) + refractionDir * RAY_EPSILON, refractionDir,
                       glm::dvec3(1.0, 1.0, 1.0), ray::REFRACTION);

      double t_val;
      glm::dvec3 incomingLight =
          traceRay(refractedRay, thresh, depth - 1, t_val);
      if (cosIncidence < 0)
        incomingLight *= glm::pow(Kt, glm::dvec3(t_val, t_val, t_val));
      colorC += incomingLight;
      if (goopEnabled && cosIncidence < 0) {
        glm::dvec3 Le = m.ke(i);
        if (Le.x > 1e-9 || Le.y > 1e-9 || Le.z > 1e-9)
          colorC += integratedEmissiveAlongPath(Le, Kt, t_val);
      }
    } else {
      if (tir) {
        glm::dvec3 reflectDir =
            computeReflectionDirection(rayDir, normalForRefraction);

        ray reflectedRay(r.at(i) + reflectDir * RAY_EPSILON, reflectDir,
                         glm::dvec3(1.0, 1.0, 1.0), ray::REFLECTION);

        double t_val;
        glm::dvec3 incomingLight =
            traceRay(reflectedRay, thresh, depth - 1, t_val);
        incomingLight *= glm::pow(Kt, glm::dvec3(t_val, t_val, t_val));
        colorC += incomingLight;
        if (goopEnabled) {
          glm::dvec3 Le = m.ke(i);
          if (Le.x > 1e-9 || Le.y > 1e-9 || Le.z > 1e-9)
            colorC += integratedEmissiveAlongPath(Le, Kt, t_val);
        }
      }
    }
  }
}

double RayTracer::averageIorFromObjects(
    const std::vector<const SceneObject *> &objStack, const isect &i) const {
  if (objStack.empty()) {
    return 1.0;
  }
  double sum = 0.0;
  for (const auto *obj : objStack) {
    sum += obj->getMaterial().index(i);
  }
  return sum / static_cast<double>(objStack.size());
}

glm::dvec3 RayTracer::averageKtFromObjects(
    const std::vector<const SceneObject *> &objStack, const isect &i) const {
  if (objStack.empty()) {
    return glm::dvec3(1.0, 1.0, 1.0);
  }
  glm::dvec3 sum(0.0, 0.0, 0.0);
  for (const auto *obj : objStack) {
    sum += obj->getMaterial().kt(i);
  }
  return sum / static_cast<double>(objStack.size());
}

void RayTracer::accumulateReflectionOverlap(
    ray &r, const isect &i, const glm::dvec3 &Kr, const glm::dvec3 &thresh,
    int depth, glm::dvec3 &colorC,
    const std::vector<const SceneObject *> &objStack) {
  if (thresh.x > 0 && std::max({Kr.x, Kr.y, Kr.z}) < thresh.x)
    return;
  if (Kr.x > 0 || Kr.y > 0 || Kr.z > 0) {
    const glm::dvec3 &rayDir = r.getDirection();
    const glm::dvec3 &surfaceNormal = i.getN();
    glm::dvec3 reflectDir = computeReflectionDirection(rayDir, surfaceNormal);

    ray reflectedRay(r.at(i) + reflectDir * RAY_EPSILON, reflectDir,
                     glm::dvec3(1.0, 1.0, 1.0), ray::REFLECTION);

    double t_val;
    colorC +=
        Kr * traceRayOverlap(reflectedRay, thresh, depth - 1, t_val, objStack);
  }
}

void RayTracer::accumulateRefractionOverlap(
    ray &r, const isect &i, const Material &m, const glm::dvec3 &Kt,
    const glm::dvec3 &thresh, int depth, glm::dvec3 &colorC,
    const std::vector<const SceneObject *> &objStack) {
  if (thresh.x > 0 && std::max({Kt.x, Kt.y, Kt.z}) < thresh.x)
    return;
  if (Kt.x > 0 || Kt.y > 0 || Kt.z > 0) {
    const glm::dvec3 &rayDir = r.getDirection();
    const glm::dvec3 &surfaceNormal = i.getN();
    std::vector<const SceneObject *> nextObjStack = objStack;

    const SceneObject *hitObject = i.getObject();
    bool foundInStack = false;
    int foundIndex = -1;
    for (int k = static_cast<int>(nextObjStack.size()) - 1; k >= 0; --k) {
      if (nextObjStack[k] == hitObject) {
        foundInStack = true;
        foundIndex = k;
        break;
      }
    }

    glm::dvec3 normalForRefraction;
    if (!foundInStack) {
      normalForRefraction = surfaceNormal;
      nextObjStack.push_back(hitObject);
    } else {
      normalForRefraction = -surfaceNormal;
      if (foundIndex >= 0) {
        nextObjStack.erase(nextObjStack.begin() + foundIndex);
      }
    }

    double n1 = averageIorFromObjects(objStack, i);
    double n2 = averageIorFromObjects(nextObjStack, i);
    if (n2 <= 0.0) {
      n2 = 1.0;
    }
    double iorRatio = n1 / n2;

    glm::dvec3 refractionDir =
        computeRefractionDirection(rayDir, normalForRefraction, iorRatio);

    if (glm::length(refractionDir) > 0.0) {
      ray refractedRay(r.at(i) + refractionDir * RAY_EPSILON, refractionDir,
                       glm::dvec3(1.0, 1.0, 1.0), ray::REFRACTION);

      double t_val;
      glm::dvec3 incomingLight =
          traceRayOverlap(refractedRay, thresh, depth - 1, t_val, nextObjStack);
      glm::dvec3 attenKt = averageKtFromObjects(nextObjStack, i);
      incomingLight *= glm::pow(attenKt, glm::dvec3(t_val, t_val, t_val));
      colorC += incomingLight;
      if (goopEnabled && !foundInStack) {
        glm::dvec3 Le = m.ke(i);
        glm::dvec3 KtObj = m.kt(i);
        if (Le.x > 1e-9 || Le.y > 1e-9 || Le.z > 1e-9)
          colorC += integratedEmissiveAlongPath(Le, KtObj, t_val);
      }
    } else {
      if (tir) {
        glm::dvec3 reflectDir =
            computeReflectionDirection(rayDir, normalForRefraction);

        ray reflectedRay(r.at(i) + reflectDir * RAY_EPSILON, reflectDir,
                         glm::dvec3(1.0, 1.0, 1.0), ray::REFLECTION);

        double t_val;
        glm::dvec3 incomingLight =
            traceRayOverlap(reflectedRay, thresh, depth - 1, t_val, objStack);
        glm::dvec3 attenKt = averageKtFromObjects(objStack, i);
        incomingLight *= glm::pow(attenKt, glm::dvec3(t_val, t_val, t_val));
        colorC += incomingLight;
        if (goopEnabled) {
          glm::dvec3 Le = m.ke(i);
          glm::dvec3 KtObj = m.kt(i);
          if (Le.x > 1e-9 || Le.y > 1e-9 || Le.z > 1e-9)
            colorC += integratedEmissiveAlongPath(Le, KtObj, t_val);
        }
      }
    }
  }
}

glm::dvec3
RayTracer::traceRayOverlap(ray &r, const glm::dvec3 &thresh, int depth,
                           double &t,
                           const std::vector<const SceneObject *> &objStack) {
  if (depth <= 0) {
    return glm::dvec3(0.0, 0.0, 0.0);
  }

  isect i;
  glm::dvec3 colorC;
#if VERBOSE
  std::cerr << "== current depth: " << depth << std::endl;
#endif

  if (scene->intersect(r, i)) {
    t = i.getT();

    const Portal *portal = dynamic_cast<const Portal *>(i.getObject());
    if (portal && portal->getPartner()) {
      glm::dvec3 exitOrigin, exitDir;
      if (portal->computeExitRay(r.at(i), r.getDirection(), exitOrigin,
                                 exitDir)) {
        constexpr int maxPortalHops = 16;
        static thread_local int portalHops = 0;
        if (portalHops < maxPortalHops) {
          ++portalHops;
          ray exitRay(exitOrigin, exitDir, r.getAtten(), r.type());
          glm::dvec3 teleportedColor =
              traceRayOverlap(exitRay, thresh, depth, t, objStack);
          --portalHops;
          glm::dvec2 uv = i.getUVCoordinates();
          double du = uv.x - 0.5, dv = uv.y - 0.5;
          double distFromCenter = 2.0 * std::sqrt(du * du + dv * dv);
          double rim = (distFromCenter > 0.85)
              ? std::min(1.0, (distFromCenter - 0.85) / 0.15)
              : 0.0;
          glm::dvec3 ke = i.getMaterial().ke(i);
          if (ke.x < 1e-9 && ke.y < 1e-9 && ke.z < 1e-9)
            ke = glm::dvec3(0.35, 0.65, 1.0);
          return teleportedColor + rim * ke;
        }
      }
    }

    const Material &m = i.getMaterial();
    colorC = m.shade(scene.get(), r, i);

    glm::dvec3 KrEff = m.kr(i);
    glm::dvec3 KtEff = m.kt(i);
    if (traceUI && traceUI->getFresnel()) {
      // Fresnel-rim coupling for recursive reflection/refraction when --fresnel is enabled.
      glm::dvec3 F0 = computeF0(m, i);
      glm::dvec3 F = recursiveFresnelFactor(r, i, m);
      glm::dvec3 rimW = recursiveRimWeight(F, F0);
      glm::dvec3 oneMinusF =
          glm::clamp(glm::dvec3(1.0) - F, glm::dvec3(0.0), glm::dvec3(1.0));
      // Keep baseline Kr reflectance and add Fresnel rim boost on top.
      // This avoids the "fresnel is less reflective than baseline" issue.
      glm::dvec3 baseKr = glm::clamp(KrEff, glm::dvec3(0.0), glm::dvec3(1.0));
      constexpr double kRimBoost = 1.0;
      KrEff = glm::clamp(
          baseKr + (glm::dvec3(1.0) - baseKr) * (kRimBoost * rimW),
          glm::dvec3(0.0), glm::dvec3(1.0));
      KtEff *= oneMinusF;
    }
    accumulateReflectionOverlap(r, i, KrEff, thresh, depth, colorC, objStack);
    accumulateRefractionOverlap(r, i, m, KtEff, thresh, depth, colorC, objStack);

  } else {
    t = 0.0;
    if (traceUI && traceUI->cubeMap()) {
      colorC = traceUI->getCubeMap()->getColor(r);
    } else {
      colorC = backgroundColor();
    }
  }
#if VERBOSE
  std::cerr << "== depth: " << depth + 1 << " done, returning: " << colorC
            << std::endl;
#endif
  return colorC;
}

// global illumination
glm::dvec3 RayTracer::computeIndirectIllumination(const ray &r, const isect &i,
                                                   const BRDF &brdf,
                                                   const glm::dvec3 &thresh,
                                                   int depth, int numSamples) {
  if (thresh.x > 0 && std::max({brdf.kd.x, brdf.kd.y, brdf.kd.z,
                                 brdf.ks.x, brdf.ks.y, brdf.ks.z}) < thresh.x)
    return glm::dvec3(0.0);

  if (depth <= 0 || numSamples <= 0)
    return glm::dvec3(0.0);

  glm::dvec3 hitPoint = r.at(i);
  glm::dvec3 surfaceNormal = i.getN();
  glm::dvec3 incidentDir = r.getDirection();

  glm::dvec3 indirectLight(0.0);
  HemisphereSampler sampler;

  int diffuseSamples = static_cast<int>(numSamples * 0.5);
  int specularSamples = numSamples - diffuseSamples;

  glm::dvec3 diffuseEstimate(0.0);
  for (int sampleIdx = 0; sampleIdx < diffuseSamples; ++sampleIdx) {
    glm::dvec3 sampleDir =
        sampler.sampleCosineWeightedHemisphere(surfaceNormal);
    const double cosTheta = glm::dot(surfaceNormal, sampleDir);
    if (cosTheta <= 0.0) {
      continue;
    }
    const double pdf =
        HemisphereSampler::pdfCosineWeighted(surfaceNormal, sampleDir);
    if (pdf <= 1e-12) {
      continue;
    }
    glm::dvec3 incomingRadiance =
        traceIndirectRay(hitPoint, sampleDir, thresh, depth - 1);
    const glm::dvec3 brdfValue = brdf.kd / M_PI;
    diffuseEstimate += incomingRadiance * brdfValue * (cosTheta / pdf);
  }
  if (diffuseSamples > 0) {
    diffuseEstimate /= static_cast<double>(diffuseSamples);
  }
  indirectLight += diffuseEstimate;

  double specularWeight = std::max({brdf.ks.x, brdf.ks.y, brdf.ks.z});
  if (specularWeight > 1e-6) {
    glm::dvec3 idealReflection =
        glm::normalize(computeReflectionDirection(incidentDir, surfaceNormal));
    const double shininess =
        std::max(1.0, 2.0 / (brdf.roughness * brdf.roughness) - 2.0);

    glm::dvec3 specularEstimate(0.0);
    for (int sampleIdx = 0; sampleIdx < specularSamples; ++sampleIdx) {
      glm::dvec3 sampleDir = sampler.samplePhongSpecular(
          surfaceNormal, idealReflection, brdf.roughness);

      const double cosTheta = glm::dot(surfaceNormal, sampleDir);
      if (cosTheta <= 0.0) {
        continue;
      }

      const double rDotWi = glm::dot(idealReflection, sampleDir);
      if (rDotWi <= 0.0) {
        continue;
      }

      const double pdf =
          ((shininess + 1.0) / (2.0 * M_PI)) * std::pow(rDotWi, shininess);
      if (pdf <= 1e-12) {
        continue;
      }

      glm::dvec3 incomingRadiance =
          traceIndirectRay(hitPoint, sampleDir, thresh, depth - 1);

      const glm::dvec3 brdfValue =
          brdf.ks * ((shininess + 2.0) / (2.0 * M_PI)) *
          std::pow(rDotWi, shininess);
      specularEstimate += incomingRadiance * brdfValue * (cosTheta / pdf);
    }
    if (specularSamples > 0) {
      specularEstimate /= static_cast<double>(specularSamples);
    }
    indirectLight += specularEstimate;
  }

  return indirectLight;
}

glm::dvec3 RayTracer::traceIndirectRay(const glm::dvec3 &hitPoint,
                                       const glm::dvec3 &direction,
                                       const glm::dvec3 &thresh, int depth) {
  if (depth <= 0)
    return glm::dvec3(0.0);

  glm::dvec3 rayOrigin = hitPoint + direction * RAY_EPSILON;
  ray secondaryRay(rayOrigin, glm::normalize(direction),
                   glm::dvec3(1.0, 1.0, 1.0), ray::SHADOW);

  double t_dummy;
  return traceRay(secondaryRay, thresh, depth, t_dummy);
}


// path tracer (monte carlo path tracing with next event estimation)
// distributed ray tracing- N secondary rays per bounce (exponential tree)

// path tracing- 1 random path per sample through the scene
// SPP is param- higher means more samples to average

// emission (first bounce or after specular bounce only, to avoid double-counting with NEE)
// next event estimation: shadow rays to all lights, weighted by BRDF
// sample 1 direction from the BRDF (diffuse or specular chosen probabilistically)
// update throughput by BRDF * cos(theta) / pdf
// adaptive termination
//
// this should capture-
// - color bleeding (diffuse interreflection)
// - soft shadows (via area light NEE)
// - ambient occlusion (dark corners/edges)
// - caustics (specular-to-diffuse paths, though noisy)
glm::dvec3 RayTracer::tracePathRay(ray &r, int maxDepth) {
  glm::dvec3 radiance(0.0);
  glm::dvec3 throughput(1.0);
  ray currentRay = r;

  static thread_local std::mt19937 rng(std::random_device{}());
  static thread_local std::uniform_real_distribution<double> unifDist(0.0, 1.0);
  HemisphereSampler sampler;

  bool lastBounceSpecular = true; // camera ray counts as "specular"

  for (int bounce = 0; bounce <= maxDepth; ++bounce) {
    isect is;
    if (!scene->intersect(currentRay, is)) {
      if (traceUI && traceUI->cubeMap()) {
        radiance += throughput * traceUI->getCubeMap()->getColor(currentRay);
      } else {
        radiance += throughput * backgroundColor();
      }
      break;
    }

    const Material &m = is.getMaterial();
    glm::dvec3 hitPoint = currentRay.at(is);
    glm::dvec3 N = is.getN(); // unit normal from intersection
    glm::dvec3 wo = -currentRay.getDirection(); // outgoing dir (toward camera)

    bool backfacing = glm::dot(N, wo) < 0.0;
    if (backfacing)
      N = -N;

    glm::dvec3 ke = m.ke(is);
    if ((bounce == 0 || lastBounceSpecular) &&
        (ke.x > 1e-9 || ke.y > 1e-9 || ke.z > 1e-9)) {
      radiance += throughput * ke;
    }

    glm::dvec3 kd = m.kd(is);
    glm::dvec3 ks = m.ks(is);
    double shininess = std::max(1.0, m.shininess(is));
    glm::dvec3 kt = m.kt(is);
    glm::dvec3 kr = m.kr(is);

    double kdMax = std::max({kd.x, kd.y, kd.z});
    double ksMax = std::max({ks.x, ks.y, ks.z});
    double ktMax = std::max({kt.x, kt.y, kt.z});
    double krMax = std::max({kr.x, kr.y, kr.z});

    if (kdMax + ksMax + ktMax + krMax < 1e-9)
      break;

    if (kdMax + ksMax > 1e-9) {
      for (const auto &pLight : scene->getAllLights()) {
        glm::dvec3 dirToLight = pLight->getDirection(hitPoint);
        double NdotL = glm::dot(N, dirToLight);
        if (NdotL <= 0.0)
          continue;

        double distAtten = pLight->distanceAttenuation(hitPoint);
        if (distAtten < 1e-6)
          continue;

        glm::dvec3 shadowAtten =
            pLight->shadowAttenuation(currentRay, N, hitPoint);
        if (shadowAtten.x + shadowAtten.y + shadowAtten.z < 1e-6)
          continue;

        glm::dvec3 directContrib = kd * NdotL;

        if (ksMax > 1e-9) {
          glm::dvec3 R_wo =
              glm::normalize(2.0 * glm::dot(N, wo) * N - wo);
          double specCos = std::max(0.0, glm::dot(R_wo, dirToLight));
          if (specCos > 0.0) {
            directContrib += ks * std::pow(specCos, shininess);
          }
        }

        radiance += throughput * directContrib * pLight->getColor() *
                    distAtten * shadowAtten;
      }
    }

    // caustics
    if (!scene->causticMap.empty() && kdMax > 1e-9) {
      glm::dvec3 causticIrradiance = scene->causticMap.irradianceEstimate(
          hitPoint, N,
          scene->causticSearchRadius,
          scene->causticGatherCount);
      radiance += throughput * kd * causticIrradiance;
    }

    double kdLum = 0.299 * kd.x + 0.587 * kd.y + 0.114 * kd.z;
    double ksLum = 0.299 * ks.x + 0.587 * ks.y + 0.114 * ks.z;
    double ktLum = 0.299 * kt.x + 0.587 * kt.y + 0.114 * kt.z;
    double krLum = 0.299 * kr.x + 0.587 * kr.y + 0.114 * kr.z;
    double totalLum = kdLum + ksLum + ktLum + krLum;
    if (totalLum < 1e-10)
      break;

    double pDiffuse = kdLum / totalLum;
    double pSpecular = ksLum / totalLum;
    double pMirror = krLum / totalLum;
    // pTransmit = ktLum / totalLum (implicit: 1 - pDiffuse - pSpecular - pMirror)

    double xi = unifDist(rng);
    glm::dvec3 wi;

    if (xi < pDiffuse) {
      wi = sampler.sampleCosineWeightedHemisphere(N);
      double cosTheta = glm::dot(N, wi);
      if (cosTheta <= 0.0)
        break;

      double pdf_d = cosTheta / M_PI;
      double pdf_s = 0.0;
      if (ksMax > 1e-9) {
        glm::dvec3 R_wo = glm::normalize(2.0 * glm::dot(N, wo) * N - wo);
        double rDotWi = std::max(0.0, glm::dot(R_wo, wi));
        pdf_s = ((shininess + 1.0) / (2.0 * M_PI)) *
                std::pow(rDotWi, shininess);
      }
      double mixedPdf = pDiffuse * pdf_d + pSpecular * pdf_s;
      if (mixedPdf <= 1e-12)
        break;

      // PBR BRDF for energy-conserving throughput: kd/pi + ks*(n+2)/(2pi)*(R·wi)^n
      glm::dvec3 brdfVal = kd / M_PI;
      if (ksMax > 1e-9) {
        glm::dvec3 R_wo = glm::normalize(2.0 * glm::dot(N, wo) * N - wo);
        double specCos = std::max(0.0, glm::dot(R_wo, wi));
        if (specCos > 0.0) {
          brdfVal += ks * ((shininess + 2.0) / (2.0 * M_PI)) *
                     std::pow(specCos, shininess);
        }
      }

      throughput *= brdfVal * cosTheta / mixedPdf;
      lastBounceSpecular = false;

    } else if (xi < pDiffuse + pSpecular) {
      glm::dvec3 R_wo = glm::normalize(2.0 * glm::dot(N, wo) * N - wo);
      double roughness = std::sqrt(2.0 / (shininess + 2.0));
      wi = sampler.samplePhongSpecular(N, R_wo, roughness);
      double cosTheta = glm::dot(N, wi);
      if (cosTheta <= 0.0)
        break;

      double rDotWi = std::max(0.0, glm::dot(R_wo, wi));
      double pdf_s = ((shininess + 1.0) / (2.0 * M_PI)) *
                     std::pow(rDotWi, shininess);
      double pdf_d = cosTheta / M_PI;
      double mixedPdf = pDiffuse * pdf_d + pSpecular * pdf_s;
      if (mixedPdf <= 1e-12)
        break;

      // PBR BRDF for energy-conserving throughput
      glm::dvec3 brdfVal = kd / M_PI;
      if (rDotWi > 0.0) {
        brdfVal += ks * ((shininess + 2.0) / (2.0 * M_PI)) *
                   std::pow(rDotWi, shininess);
      }

      throughput *= brdfVal * cosTheta / mixedPdf;
      lastBounceSpecular = (shininess > 50.0);

    } else if (xi < pDiffuse + pSpecular + pMirror) {
      wi = computeReflectionDirection(-wo, N);
      throughput *= kr / (krLum / totalLum);
      lastBounceSpecular = true;

    } else {
      double ior = m.index(is);
      double iorRatio = backfacing ? (ior / 1.0) : (1.0 / ior);
      glm::dvec3 normalForRefract = N;

      glm::dvec3 refrDir =
          computeRefractionDirection(-wo, normalForRefract, iorRatio);

      if (glm::length(refrDir) < 1e-6) {
        wi = computeReflectionDirection(-wo, N);
      } else {
        wi = glm::normalize(refrDir);
      }

      throughput *= kt / (ktLum / totalLum);
      lastBounceSpecular = true;
    }

    if (bounce >= 3) {
      double maxT = std::max({throughput.x, throughput.y, throughput.z});
      double pContinue = std::min(maxT, 0.95);
      if (pContinue < 0.01)
        break;
      if (unifDist(rng) >= pContinue)
        break;
      throughput /= pContinue;
    }

    currentRay = ray(hitPoint + wi * RAY_EPSILON, glm::normalize(wi),
                     glm::dvec3(1.0), ray::VISIBILITY);
  }

  return radiance;
}

glm::dvec3 RayTracer::backgroundColor() const {
  return glm::dvec3(0.0, 0.0, 0.0);
}

// Trace a top-level ray through pixel(i,j), i.e. normalized window coordinates
// (x,y), through the projection plane, and out into the scene. All we do is
// enter the main ray-tracing method, getting things started by plugging in an
// initial ray weight of (0.0,0.0,0.0) and an initial recursion depth of 0.
glm::dvec3 RayTracer::trace(double x, double y) {
  // Clear out the ray cache in the scene for debugging purposes,
  if (TraceUI::m_debug) {
    scene->clearIntersectCache(); // do not run debug mode with multithreading
  }

  if (usePathIntegrator) {
    int spp = std::max(1, pathSPP);
    int maxDepth = std::max(5, traceUI->getDepth());
    glm::dvec3 accumColor(0.0);

    static thread_local std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> jitter(-0.5, 0.5);

    for (int s = 0; s < spp; ++s) {
      double jx = jitter(rng) / buffer_width;
      double jy = jitter(rng) / buffer_height;

      ray r(glm::dvec3(0, 0, 0), glm::dvec3(0, 0, 0), glm::dvec3(1, 1, 1),
            ray::VISIBILITY);
      scene->getCamera().rayThrough(x + jx, y + jy, r);

      if (dofLens && dofLens->isEnabled()) {
        const Camera &cam = scene->getCamera();
        glm::dvec3 camLook = glm::normalize(cam.getLook());
        glm::dvec3 focusPlanePoint =
            cam.getEye() + camLook * dofLens->getFocalDistance();
        glm::dvec3 rayDir = glm::normalize(r.getDirection());
        double denom = glm::dot(rayDir, camLook);
        double t = (std::abs(denom) > 1e-6)
                       ? glm::dot(focusPlanePoint - r.getPosition(), camLook) /
                             denom
                       : dofLens->getFocalDistance();
        glm::dvec3 focusPoint = r.getPosition() + rayDir * t;

        glm::dvec2 apertureOffset = dofLens->sampleAperturePoint();
        glm::dvec3 right = glm::normalize(cam.getU());
        glm::dvec3 up = glm::normalize(cam.getV());
        glm::dvec3 apertureOrigin =
            cam.getEye() + right * apertureOffset.x + up * apertureOffset.y;
        r.setPosition(apertureOrigin);
        r.setDirection(glm::normalize(focusPoint - apertureOrigin));
      }

      accumColor += tracePathRay(r, maxDepth);
    }

    return glm::clamp(accumColor / static_cast<double>(spp), 0.0, 1.0);
  }

  if (dofLens && dofLens->isEnabled()) {
    constexpr int dofSamples = 32;
    glm::dvec3 accumColor(0.0);

    const Camera &cam = scene->getCamera();
    glm::dvec3 camLook = glm::normalize(cam.getLook());
    glm::dvec3 focusPlanePoint = cam.getEye() +
                                 camLook * dofLens->getFocalDistance();

    for (int s = 0; s < dofSamples; ++s) {
      ray r(glm::dvec3(0, 0, 0), glm::dvec3(0, 0, 0), glm::dvec3(1, 1, 1),
            ray::VISIBILITY);

      cam.rayThrough(x, y, r);

      glm::dvec3 rayOrigin = r.getPosition();
      glm::dvec3 rayDir = glm::normalize(r.getDirection());
      double denom = glm::dot(rayDir, camLook);
      double t = (std::abs(denom) > 1e-6)
                     ? glm::dot(focusPlanePoint - rayOrigin, camLook) / denom
                     : dofLens->getFocalDistance();
      glm::dvec3 focusPoint = rayOrigin + rayDir * t;

      glm::dvec2 apertureOffset = dofLens->sampleAperturePoint();
      glm::dvec3 right = glm::normalize(cam.getU());
      glm::dvec3 up = glm::normalize(cam.getV());
      glm::dvec3 apertureOrigin = cam.getEye() + right * apertureOffset.x +
                                  up * apertureOffset.y;

      r.setPosition(apertureOrigin);
      r.setDirection(glm::normalize(focusPoint - apertureOrigin));

      double dummy;
      const double adaptThresh = rayTermThresh;
      glm::dvec3 sampleColor =
          traceRay(r, glm::dvec3(adaptThresh, adaptThresh, adaptThresh),
                   traceUI->getDepth(), dummy);
      accumColor += glm::clamp(sampleColor, 0.0, 1.0);
    }

    return accumColor / static_cast<double>(dofSamples);
  }

  ray r(glm::dvec3(0, 0, 0), glm::dvec3(0, 0, 0), glm::dvec3(1, 1, 1),
        ray::VISIBILITY);
  scene->getCamera().rayThrough(x, y, r);
  double dummy;
  const double adaptThresh = rayTermThresh;
  glm::dvec3 ret =
      traceRay(r, glm::dvec3(adaptThresh, adaptThresh, adaptThresh),
               traceUI->getDepth(), dummy);
  ret = glm::clamp(ret, 0.0, 1.0);
  return ret;
}

glm::dvec3 RayTracer::tracePixel(int i, int j) {
  glm::dvec3 col(0, 0, 0); // col = color

  if (!sceneLoaded())
    return col;

  // if i = 1 and width = 512 -> x = 1/512 -> edge between pixel 0 and pixel 1,
  // not the center of the pixel

  // double x = double(i) / double(buffer_width);  // og code
  // double y = double(j) / double(buffer_height); // og code

  // to go to center of pixel
  double x = (double(i)) / double(buffer_width);  // og code
  double y = (double(j)) / double(buffer_height); // og code
  
  // double x = (double(i) + 0.5) / double(buffer_width);  // og code
  // double y = (double(j) + 0.5) / double(buffer_height); // og code
  

  col = trace(x, y); // og code
  col = glm::clamp(col, 0.0, 1.0);

  unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;
  pixel[0] = (int)(255.0 * col[0]);
  pixel[1] = (int)(255.0 * col[1]);
  pixel[2] = (int)(255.0 * col[2]);

  return col;
}

#define VERBOSE 0

/**
 * Henyey-Greenstein phase function
 * p(cosTheta, g) = (1 - g^2) / (4*pi * (1 + g^2 - 2*g*cosTheta)^(3/2))
 *
 * g=0, isotropic scattering
 * g>0, forward scattering (like fog/smoke)
 * g<0, backward scattering
 */
static double henyeyGreenstein(double cosTheta, double g) {
  constexpr double inv4pi = 1.0 / (4.0 * M_PI);
  if (std::abs(g) < 1e-6) {
    return inv4pi; // isotropic
  }
  double g2 = g * g;
  double denom = 1.0 + g2 - 2.0 * g * cosTheta;
  denom = std::pow(denom, 1.5);
  return inv4pi * (1.0 - g2) / denom;
}

/**
 * Beer-Lambert transmittance for a homogeneous medium
 * T(s) = exp(-sigma_t * s) applied per channel
 */
static glm::dvec3 beerLambertTransmittance(const glm::dvec3 &sigma_t, double distance) {
  return glm::dvec3(std::exp(-sigma_t.x * distance),
                    std::exp(-sigma_t.y * distance),
                    std::exp(-sigma_t.z * distance));
}

static double fbmDensity(const glm::dvec3 &worldPos, double noiseScale,
                         int octaves, double gain) {
  glm::vec3 p = glm::vec3(worldPos * noiseScale);
  float n = 0.0f, amp = 1.0f, freq = 1.0f, totalAmp = 0.0f;
  for (int i = 0; i < octaves; ++i) {
    n += amp * std::abs(glm::perlin(p * freq));
    totalAmp += amp;
    freq *= 2.0f;
    amp *= static_cast<float>(gain);
  }
  double normalized = static_cast<double>(n / totalAmp);
  return glm::clamp(normalized, 0.0, 1.0);
}

/**
 * ray march through a volumetric participating medium inside an object.
 *
 * @param r           The incoming ray  
 * @param tNear       Entry distance into the volume (along ray)
 * @param tFar        Exit distance from the volume (along ray)
 * @param mat         Material with volume properties
 * @param scene       The scene (for lights and shadow queries)
 * @param thresh      Adaptive termination threshold
 * @param depth       Remaining recursion depth
 * @param behindColor Color of whatever is behind the volume (attenuated by total transmittance)
 * @return            Final accumulated color through the volume
 */

struct VolumeInterval {
  const SceneObject *obj;
  const Material *mat;
  double tNear, tFar;
};

static glm::dvec3 volumeShadowAttenuation(
    const Light *light, const glm::dvec3 &samplePos,
    const std::vector<VolumeInterval> &volumes, Scene *scene) {

  glm::dvec3 lightDir = light->getDirection(samplePos);

  // for each volume, find how much of the shadow ray is still inside it
  // and apply Beer-Lambert for that segment
  glm::dvec3 volAtten(1.0);
  double maxExitDist = 0.0;
  for (const auto &vol : volumes) {
    ray toExit(samplePos + lightDir * RAY_EPSILON, lightDir,
               glm::dvec3(1, 1, 1), ray::SHADOW);
    isect exitHit;
    if (vol.obj->intersect(toExit, exitHit)) {
      double dist = exitHit.getT();
      glm::dvec3 sigma_t = vol.mat->volumeExtinction() * vol.mat->volumeDensity();
      volAtten *= beerLambertTransmittance(sigma_t, dist);
      maxExitDist = std::max(maxExitDist, dist);
    }
  }

  glm::dvec3 exitPoint = samplePos + lightDir * (maxExitDist + 2.0 * RAY_EPSILON);
  ray shadowRay(exitPoint, lightDir, glm::dvec3(1, 1, 1), ray::SHADOW);
  glm::dvec3 sceneAtten = light->shadowAttenuation(
      shadowRay, glm::dvec3(0, 0, 0), exitPoint);

  return volAtten * sceneAtten;
}


static double sampleVolumeDensity(const glm::dvec3 &samplePos,
                                  const SceneObject *volObj,
                                  const Material &mat) {
  double localDensity = mat.volumeDensity();
  double falloff = mat.volumeFalloff();
  double noiseScale = mat.volumeNoiseScale();

  if (falloff > 1e-9) {
    glm::dvec3 localPos = volObj->getTransform().globalToLocalCoords(samplePos);
    double r = glm::length(localPos);
    localDensity *= std::exp(-falloff * r * r);
  }

  if (noiseScale > 1e-9) {
    double noiseFactor = fbmDensity(samplePos, noiseScale,
                                   mat.volumeNoiseOctaves(),
                                   mat.volumeNoiseGain());
    localDensity *= noiseFactor;
  }

  return localDensity;
}

glm::dvec3 RayTracer::traceVolumetric(ray &r, double tNear, double tFar,
                                      const std::vector<VolumeInterval> &volumes,
                                      const glm::dvec3 &thresh, int depth,
                                      const glm::dvec3 &behindColor) {
  // Use the smallest step size among all volumes
  double stepSize = 0.02;
  for (const auto &vol : volumes) {
    stepSize = std::min(stepSize, vol.mat->volumeStepSize());
  }

  double pathLength = tFar - tNear;
  if (pathLength <= 0.0) return behindColor;

  int numSteps = std::max(1, static_cast<int>(std::ceil(pathLength / stepSize)));
  double dt = pathLength / numSteps;

  glm::dvec3 accumulatedColor(0.0);
  glm::dvec3 transmittance(1.0);
  const glm::dvec3 &rayDir = r.getDirection();

  for (int step = 0; step < numSteps; ++step) {
    double tSample = tNear + (step + 0.5) * dt;
    glm::dvec3 samplePos = r.at(tSample);

    // Sum contributions from ALL volumes at this sample point
    glm::dvec3 sigma_a(0.0), sigma_s(0.0), volEmission(0.0);
    double avgG = 0.0;
    double totalWeight = 0.0;

    for (const auto &vol : volumes) {
      // Skip volumes we're not inside at this t
      if (tSample < vol.tNear || tSample > vol.tFar) continue;

      double d = sampleVolumeDensity(samplePos, vol.obj, *vol.mat);
      if (d < 1e-8) continue;

      sigma_a += vol.mat->volumeAbsorption() * d;
      sigma_s += vol.mat->volumeScattering() * d;
      volEmission += vol.mat->volumeEmission() * d;
      avgG += vol.mat->volumePhaseG() * d;
      totalWeight += d;
    }

    if (totalWeight < 1e-8) continue;

    double g = avgG / totalWeight;  // density-weighted average phase
    glm::dvec3 sigma_t = sigma_a + sigma_s;

    glm::dvec3 stepTransmittance = beerLambertTransmittance(sigma_t, dt);
    glm::dvec3 emissionContrib = volEmission * dt;
    glm::dvec3 inscatter(0.0);

    for (auto lightIter = scene->beginLights();
         lightIter != scene->endLights(); ++lightIter) {
      const Light *light = *lightIter;
      glm::dvec3 lightDir = light->getDirection(samplePos);
      glm::dvec3 shadowAtten = volumeShadowAttenuation(
          light, samplePos, volumes, scene.get());
      double distAtten = light->distanceAttenuation(samplePos);
      double cosTheta = glm::dot(rayDir, lightDir);
      double phase = henyeyGreenstein(cosTheta, g);
      glm::dvec3 lightColor = light->getColor();
      inscatter += sigma_s * phase * lightColor * shadowAtten * distAtten * dt;
    }

    accumulatedColor += transmittance * (emissionContrib + inscatter);
    transmittance *= stepTransmittance;

    if (transmittance.x < 1e-6 && transmittance.y < 1e-6 && transmittance.z < 1e-6) {
      break;
    }
  }

  accumulatedColor += transmittance * behindColor;
  return accumulatedColor;
}

// Do recursive ray tracing! You'll want to insert a lot of code here (or places
// called from here) to handle reflection, refraction, etc etc.
glm::dvec3 RayTracer::traceRay(ray &r, const glm::dvec3 &thresh, int depth,
                               double &t) {
  if (handleOverlap) {
    std::vector<const SceneObject *> objStack;
    return traceRayOverlap(r, thresh, depth, t, objStack);
  }

  if (depth <= 0) {
    return glm::dvec3(0.0, 0.0, 0.0);
  }

  isect i;
  glm::dvec3 colorC;
#if VERBOSE
  std::cerr << "== current depth: " << depth << std::endl;
#endif

  if (scene->intersect(r, i)) {
    t = i.getT();

    const Portal *portal = dynamic_cast<const Portal *>(i.getObject());
    if (portal && portal->getPartner()) {
      glm::dvec3 exitOrigin, exitDir;
      if (portal->computeExitRay(r.at(i), r.getDirection(), exitOrigin,
                                 exitDir)) {
        constexpr int maxPortalHops = 16;
        static thread_local int portalHops = 0;
        if (portalHops < maxPortalHops) {
          ++portalHops;
          ray exitRay(exitOrigin, exitDir, r.getAtten(), r.type());
          glm::dvec3 teleportedColor = traceRay(exitRay, thresh, depth, t);
          --portalHops;
          glm::dvec2 uv = i.getUVCoordinates();
          double du = uv.x - 0.5, dv = uv.y - 0.5;
          double distFromCenter = 2.0 * std::sqrt(du * du + dv * dv);
          double rim = (distFromCenter > 0.85)
              ? std::min(1.0, (distFromCenter - 0.85) / 0.15)
              : 0.0;
          glm::dvec3 ke = i.getMaterial().ke(i);
          if (ke.x < 1e-9 && ke.y < 1e-9 && ke.z < 1e-9)
            ke = glm::dvec3(0.35, 0.65, 1.0);
          return teleportedColor + rim * ke;
        }
      }
    }

    const Material &m = i.getMaterial();


    if (m.isVolumetric()) {
      // Gather ALL volumetric objects the ray intersects
      std::vector<VolumeInterval> volumes;
      double unionNear = 1e30, unionFar = -1e30;

      for (auto it = scene->beginObjects(); it != scene->endObjects(); ++it) {
        const SceneObject *obj = dynamic_cast<const SceneObject *>(*it);
        if (!obj || !obj->getMaterial().isVolumetric()) continue;

        ray testRay(r.getPosition(), r.getDirection(),
                    glm::dvec3(1, 1, 1), ray::VISIBILITY);
        isect entryIsect;
        if (!obj->intersect(testRay, entryIsect)) continue;

        double tN = entryIsect.getT();
        // find exit
        glm::dvec3 entryPt = testRay.at(tN);
        ray exitRay(entryPt + r.getDirection() * RAY_EPSILON,
                    r.getDirection(), glm::dvec3(1, 1, 1), ray::VISIBILITY);
        isect exitIsect;
        double tF = tN;
        if (obj->intersect(exitRay, exitIsect)) {
          tF = tN + RAY_EPSILON + exitIsect.getT();
        } else {
          tF = tN + 20.0;
        }

        volumes.push_back({obj, &obj->getMaterial(), tN, tF});
        unionNear = std::min(unionNear, tN);
        unionFar = std::max(unionFar, tF);
      }

      if (volumes.empty()) {
        // shouldn't happen but fall through to surface shading
        colorC = m.shade(scene.get(), r, i);
      } else {
        // behind color: trace ray past all volumes
        glm::dvec3 exitPoint = r.at(unionFar);
        ray behindRay(exitPoint + r.getDirection() * RAY_EPSILON,
                      r.getDirection(), r.getAtten(), ray::VISIBILITY);
        double tBehind;
        glm::dvec3 behindColor = traceRay(behindRay, thresh, depth - 1, tBehind);

        colorC = traceVolumetric(r, unionNear, unionFar, volumes, thresh, depth, behindColor);
      }
    } else {
      colorC = m.shade(scene.get(), r, i);

      if (useDistributedGI && giNumSamples > 0 && depth > 1) {
        glm::dvec3 kd = m.kd(i);
        glm::dvec3 ks = m.ks(i);
        double shininess = m.shininess(i);
        BRDF brdf = BRDF::fromPhong(kd, ks, shininess);

        glm::dvec3 indirectColor =
            computeIndirectIllumination(r, i, brdf, thresh, depth, giNumSamples);
        colorC += giIntensity * indirectColor;
      }

      glm::dvec3 KrEff = m.kr(i);
      glm::dvec3 KtEff = m.kt(i);
      if (traceUI && traceUI->getFresnel()) {
        // Fresnel-rim coupling for recursive reflection/refraction when --fresnel is enabled.
        glm::dvec3 F0 = computeF0(m, i);
        glm::dvec3 F = recursiveFresnelFactor(r, i, m);
        glm::dvec3 rimW = recursiveRimWeight(F, F0);
        glm::dvec3 oneMinusF =
            glm::clamp(glm::dvec3(1.0) - F, glm::dvec3(0.0), glm::dvec3(1.0));
        // Keep baseline Kr reflectance and add Fresnel rim boost on top.
        // This avoids the "fresnel is less reflective than baseline" issue.
        glm::dvec3 baseKr = glm::clamp(KrEff, glm::dvec3(0.0), glm::dvec3(1.0));
        constexpr double kRimBoost = 1.0;
        KrEff = glm::clamp(
            baseKr + (glm::dvec3(1.0) - baseKr) * (kRimBoost * rimW),
            glm::dvec3(0.0), glm::dvec3(1.0));
        KtEff *= oneMinusF;
      }
      accumulateReflection(r, i, KrEff, thresh, depth, colorC);
      accumulateRefraction(r, i, m, KtEff, thresh, depth, colorC);
    }

  } else {
    t = 0.0;
    // std::cout << "no intersection." << std::endl;
    // No intersection. This ray travels to infinity, so we color
    // it according to the background color, which in this (simple)
    // case is just black.
    //
    if (traceUI && traceUI->cubeMap()) {
      colorC = traceUI->getCubeMap()->getColor(r);
    } else {
      colorC = backgroundColor();
    }
  }
#if VERBOSE
  std::cerr << "== depth: " << depth + 1 << " done, returning: " << colorC
            << std::endl;
#endif
  return colorC;
}

RayTracer::RayTracer()
    : scene(nullptr), buffer(0), rayTermThresh(0), blockInterpThresh(0),
    buffer_width(0), buffer_height(0),
      m_bBufferReady(false), goopEnabled(false), giNumSamples(0), 
      giIntensity(1.0), areaLightSamples(1), usePathIntegrator(false),
      useDistributedGI(false), pathSPP(64), causticPhotons(0) {}

RayTracer::~RayTracer() {}


void RayTracer::emitCausticPhotons() {
  if (!scene || causticPhotons <= 0) return;

  scene->causticMap = PhotonMap();
  scene->causticSearchRadius = traceUI->getCausticRadius();
  scene->causticGatherCount = traceUI->getCausticGather();

  const int maxBounces = std::max(traceUI->getDepth(), 10);
  const auto &lights = scene->getAllLights();
  if (lights.empty()) return;


  std::vector<const PointLight *> pointLights;
  for (const auto *light : lights) {
    if (auto *pl = dynamic_cast<const PointLight *>(light))
      pointLights.push_back(pl);
  }
  if (pointLights.empty()) return;
  struct SpecTarget {
    glm::dvec3 center;
    double radius;
  };
  std::vector<SpecTarget> specTargets;

  for (auto it = scene->beginObjects(); it != scene->endObjects(); ++it) {
    const SceneObject *obj = dynamic_cast<const SceneObject *>(*it);
    if (!obj) continue;
    const Material &mat = obj->getMaterial();
    if (!mat.Recur()) continue;

    if (obj->hasBoundingBoxCapability()) {
      const BoundingBox &bb = obj->getBoundingBox();
      glm::dvec3 bbMin = bb.getMin();
      glm::dvec3 bbMax = bb.getMax();
      glm::dvec3 center = 0.5 * (bbMin + bbMax);
      double radius = 0.5 * glm::length(bbMax - bbMin);
      if (radius > 1e-6) {
        specTargets.push_back({center, radius});
      }
    }
  }

  if (specTargets.empty()) {
    std::cerr << "[caustics] no specular objects found, skipping." << std::endl;
    return;
  }

  const int totalPhotons = causticPhotons;
  const int photonsPerLightTarget =
      totalPhotons / static_cast<int>(pointLights.size() * specTargets.size());
  if (photonsPerLightTarget <= 0) return;

  scene->causticMap.reserve(totalPhotons);

  int numThreads = std::max(1u, threads);
  std::vector<std::vector<Photon>> threadPhotons(numThreads);

  for (const PointLight *pl : pointLights) {
    const glm::dvec3 lightPos = pl->getPosition();
    const glm::dvec3 lightColor = pl->getColor();

    for (const auto &target : specTargets) {
      glm::dvec3 toTarget = target.center - lightPos;
      double dist = glm::length(toTarget);
      if (dist < 1e-6) continue;

      double sinHalf = std::min(1.0, target.radius / dist);
      double cosHalf = std::sqrt(1.0 - sinHalf * sinHalf);

      double solidAngle = 2.0 * M_PI * (1.0 - cosHalf);

      double distAtten = pl->distanceAttenuation(target.center);
      glm::dvec3 photonPower =
          lightColor * solidAngle * (dist * dist) * distAtten
          / static_cast<double>(photonsPerLightTarget);

      glm::dvec3 w = toTarget / dist;
      glm::dvec3 u, v;
      if (std::abs(w.x) < 0.9) {
        u = glm::normalize(glm::cross(glm::dvec3(1, 0, 0), w));
      } else {
        u = glm::normalize(glm::cross(glm::dvec3(0, 1, 0), w));
      }
      v = glm::cross(w, u);

      #pragma omp parallel
      {
        int tid = 0;
        #ifdef _OPENMP
        tid = omp_get_thread_num();
        #endif
        std::mt19937 rng(42 + tid * 1000 +
                         static_cast<int>(reinterpret_cast<uintptr_t>(pl) & 0xFFFF) +
                         static_cast<int>(reinterpret_cast<uintptr_t>(&target) & 0xFFFF));
        std::uniform_real_distribution<double> dist01(0.0, 1.0);

        #pragma omp for schedule(dynamic, 256)
        for (int p = 0; p < photonsPerLightTarget; ++p) {
          double cosTheta = 1.0 - dist01(rng) * (1.0 - cosHalf);
          double sinTheta = std::sqrt(1.0 - cosTheta * cosTheta);
          double phi = 2.0 * M_PI * dist01(rng);
          glm::dvec3 dir = sinTheta * std::cos(phi) * u +
                           sinTheta * std::sin(phi) * v +
                           cosTheta * w;
          dir = glm::normalize(dir);

          ray photonRay(lightPos + dir * RAY_EPSILON, dir,
                        glm::dvec3(1.0), ray::VISIBILITY);
          glm::dvec3 power = photonPower;
          bool hadSpecularBounce = false;

          for (int bounce = 0; bounce < maxBounces; ++bounce) {
            isect hit;
            if (!scene->intersect(photonRay, hit))
              break;

            const Material &mat = hit.getMaterial();
            glm::dvec3 hitPoint = photonRay.at(hit);
            glm::dvec3 normal = hit.getN();

            glm::dvec3 kd = mat.kd(hit);
            glm::dvec3 kr = mat.kr(hit);
            glm::dvec3 kt = mat.kt(hit);
            double kdLum = 0.299 * kd.x + 0.587 * kd.y + 0.114 * kd.z;
            double krLum = 0.299 * kr.x + 0.587 * kr.y + 0.114 * kr.z;
            double ktLum = 0.299 * kt.x + 0.587 * kt.y + 0.114 * kt.z;
            double totalLum = kdLum + krLum + ktLum;

            if (totalLum < 1e-10) break;

            if (hadSpecularBounce && kdLum > 1e-6) {
              Photon stored;
              stored.position = hitPoint;
              stored.power = power;
              stored.incomingDir = photonRay.getDirection();
              stored.splitAxis = 0;
              threadPhotons[tid].push_back(stored);
              break;
            }

            if (bounce >= 3) {
              double survival = std::min(0.9, totalLum);
              if (dist01(rng) > survival) break;
              power /= survival;
            }

            double xi = dist01(rng) * totalLum;

            if (xi < krLum) {
              hadSpecularBounce = true;
              glm::dvec3 reflDir = computeReflectionDirection(
                  photonRay.getDirection(), normal);
              power *= kr;
              photonRay = ray(hitPoint + reflDir * RAY_EPSILON, reflDir,
                              glm::dvec3(1.0), ray::REFLECTION);
            } else if (xi < krLum + ktLum) {
              hadSpecularBounce = true;
              double ior = mat.index(hit);
              double cosI = glm::dot(normal, photonRay.getDirection());
              double iorRatio;
              glm::dvec3 normalForRefraction;
              if (cosI < 0) {
                iorRatio = 1.0 / ior;
                normalForRefraction = normal;
              } else {
                iorRatio = ior;
                normalForRefraction = -normal;
              }
              glm::dvec3 refrDir = computeRefractionDirection(
                  photonRay.getDirection(), normalForRefraction, iorRatio);
              if (glm::length(refrDir) < 1e-10) {
                glm::dvec3 reflDir = computeReflectionDirection(
                    photonRay.getDirection(), normalForRefraction);
                power *= kr.x > 1e-6 || kr.y > 1e-6 || kr.z > 1e-6 ? kr : kt;
                photonRay = ray(hitPoint + reflDir * RAY_EPSILON, reflDir,
                                glm::dvec3(1.0), ray::REFLECTION);
              } else {
                power *= kt;
                photonRay = ray(hitPoint + refrDir * RAY_EPSILON, refrDir,
                                glm::dvec3(1.0), ray::REFRACTION);
              }
            } else {
              break;
            }
          }
        }
      }
    }
  }

  for (auto &tp : threadPhotons) {
    for (auto &phot : tp) {
      scene->causticMap.store(phot);
    }
  }

  if (!scene->causticMap.empty()) {
    scene->causticMap.balance();
    std::cerr << "[caustics] stored " << scene->causticMap.size()
              << " caustic photons." << std::endl;
  } else {
    std::cerr << "[caustics] no caustic photons stored (no specular→diffuse paths found)." << std::endl;
  }
}

void RayTracer::getBuffer(unsigned char *&buf, int &w, int &h) {
  buf = buffer.data();
  w = buffer_width;
  h = buffer_height;
}

void RayTracer::getVisualizationBuffer(unsigned char *&buf, int &w, int &h) {
  buf = aaVisBuffer.empty() ? nullptr : aaVisBuffer.data();
  w = aaVisBuffer.empty() ? 0 : buffer_width;
  h = aaVisBuffer.empty() ? 0 : buffer_height;
}

double RayTracer::aspectRatio() {
  return sceneLoaded() ? scene->getCamera().getAspectRatio() : 1;
}

void RayTracer::setScene(Scene *s) { scene.reset(s); }

bool RayTracer::loadScene(const char *fn) {
  ifstream ifs(fn);
  if (!ifs) {
    string msg("Error: couldn't read scene file ");
    msg.append(fn);
    traceUI->alert(msg);
    return false;
  }

  // Check if fn ends in '.ray'
  bool isRay = false;
  const char *ext = strrchr(fn, '.');
  if (ext && !strcmp(ext, ".ray"))
    isRay = true;

  // Strip off filename, leaving only the path:
  string path(fn);
  if (path.find_last_of("\\/") == string::npos)
    path = ".";
  else
    path = path.substr(0, path.find_last_of("\\/"));

  if (isRay) {
    hasSceneJsonTemplate = false;
    sceneJsonContents.clear();
    sceneJsonDir.clear();
    // .ray Parsing Path
    // Call this with 'true' for debug output from the tokenizer
    Tokenizer tokenizer(ifs, false);
    Parser parser(tokenizer, path);
    try {
      scene.reset(parser.parseScene());
    } catch (SyntaxErrorException &pe) {
      traceUI->alert(pe.formattedMessage());
      return false;
    } catch (ParserException &pe) {
      string msg("Parser: fatal exception ");
      msg.append(pe.message());
      traceUI->alert(msg);
      return false;
    } catch (TextureMapException e) {
      string msg("Texture mapping exception: ");
      msg.append(e.message());
      traceUI->alert(msg);
      return false;
    }
  } else {
    // JSON Parsing Path
    try {
      hasSceneJsonTemplate = true;
      sceneJsonDir = path;
      std::ostringstream sstr;
      sstr << ifs.rdbuf();
      sceneJsonContents = sstr.str();
      if (sceneJsonContents.empty()) {
        hasSceneJsonTemplate = false;
        sceneJsonContents.clear();
        sceneJsonDir.clear();
      } else {
        nlohmann::json baseSceneJson = nlohmann::json::parse(sceneJsonContents);
        nlohmann::json initialJson = baseSceneJson;
        substituteExpressions(initialJson, 0.0);
        JsonParser parser(path);
        scene.reset(parser.parseSceneFromJson(initialJson));
      }
    } catch (ParserException &pe) {
      string msg("Parser: fatal exception ");
      msg.append(pe.message());
      traceUI->alert(msg);
      return false;
    } catch (const json::exception &je) {
      string msg("Invalid JSON encountered ");
      msg.append(je.what());
      traceUI->alert(msg);
      return false;
    }
  }

  if (!sceneLoaded())
    return false;

  return true;
}

void RayTracer::traceSetup(int w, int h) {
  // Always start a new render in the "not stopped" state.
  stopTrace.store(false, std::memory_order_release);

  size_t newBufferSize = w * h * 3;
  if (newBufferSize != buffer.size()) {
    bufferSize = newBufferSize;
    buffer.resize(bufferSize);
  }
  buffer_width = w;
  buffer_height = h;
  std::fill(buffer.begin(), buffer.end(), 0);
  m_bBufferReady = true;
  skipNextPostAAPass = false;

  /*
   * Sync with TraceUI
   */

  threads = traceUI->getThreads();
  block_size = traceUI->getBlockSize();
  rayTermThresh = traceUI->getRayTermThreshold();
  blockInterpThresh = traceUI->getThreshold();
  samples = traceUI->getSuperSamples();
  aaThresh = traceUI->getAaThreshold();
  tir = traceUI->getTotalInternalReflection();
  handleOverlap = traceUI->getHandleOverlap();
  aaVisualize = traceUI->aaVisualize();
  goopEnabled = traceUI->getGoop();
  usePathIntegrator = (traceUI->tracerMode() == TraceUI::TracerMode::Path);
  useDistributedGI = (traceUI->tracerMode() == TraceUI::TracerMode::Distributed);
  pathSPP = traceUI->getPathSPP();
  areaLightSamples = traceUI->getAreaLightSamples();
  setGIEnabled(useDistributedGI && traceUI->getGISamples() > 0,
               traceUI->getGISamples(), traceUI->getGIIntensity());
  causticPhotons = traceUI->getCausticPhotons();
  if (aaVisualize) {
    aaVisBuffer.resize(buffer_width * buffer_height * 3);
    std::fill(aaVisBuffer.begin(), aaVisBuffer.end(),
              static_cast<unsigned char>(0));
  }

  if (scene) {
    const Camera &cam = scene->getCamera();
    if (cam.hasDepthOfField()) {
      setDepthOfField(cam.getFocalDistance(), cam.getFNumber());
    } else if (dofLens) {
      dofLens->setEnabled(false);
    }

    if ((usePathIntegrator || useDistributedGI) && cam.hasMotionBlur()) {
      setMotionBlur(cam.getShutterOpen(), cam.getShutterClose());
    } else if (temporalSampler) {
      temporalSampler->setEnabled(false);
    }

    bool hasAreaLights = false;
    for (const auto &light : scene->getAllLights()) {
      if (const PointLight *pointLight =
              dynamic_cast<const PointLight *>(light)) {
        if (pointLight->isAreaLight()) {
          hasAreaLights = true;
          break;
        }
      }
    }
    if ((usePathIntegrator || useDistributedGI) && hasAreaLights) {
      setAreaLightSamples(std::max(areaLightSamples, 4));
    }
    traceUI->setAreaLightSamples(areaLightSamples);
  }
}

/*
 * RayTracer::traceImage
 *
 *	Trace the image and store the pixel data in RayTracer::buffer.
 *
 *	Arguments:
 *		w:	width of the image buffer
 *		h:	height of the image buffer
 *
 */
void RayTracer::renderCurrentScene(int w, int h) {
  if (threads > 0) {
    omp_set_num_threads(threads);
  }

  // YOUR CODE HERE
  // FIXME: Start one or more threads for ray tracing
  //
  // TIPS: Ideally, the traceImage should be executed asynchronously,
  //       i.e. returns IMMEDIATELY after working threads are launched.
  //
  //       An asynchronous traceImage lets the GUI update your results
  //       while rendering.

  const int tileSize = std::max(1, block_size);
  const int tilesX = (w + tileSize - 1) / tileSize;
  const int tilesY = (h + tileSize - 1) / tileSize;
  const bool useBlockInterp = (blockInterpThresh > 0);

#pragma omp parallel for collapse(2) schedule(dynamic, 1)
  for (int ty = 0; ty < tilesY; ++ty) {
    for (int tx = 0; tx < tilesX; ++tx) {
      if (stopTrace.load(std::memory_order_acquire))
        continue;

      const int x0 = tx * tileSize;
      const int y0 = ty * tileSize;
      const int x1 = std::min(x0 + tileSize, w);
      const int y1 = std::min(y0 + tileSize, h);

      if (useBlockInterp && (x1 - x0) >= 2 && (y1 - y0) >= 2) {
        // block interpolation
        const int ixR = x1 - 1;
        const int jyB = y1 - 1;
        const double invW = 1.0 / static_cast<double>(buffer_width);
        const double invH = 1.0 / static_cast<double>(buffer_height);

        glm::dvec3 cTL = glm::clamp(trace((x0 + 0.5) * invW, (y0 + 0.5) * invH),
                                    0.0, 1.0);
        glm::dvec3 cTR = glm::clamp(trace((ixR + 0.5) * invW, (y0 + 0.5) * invH),
                                    0.0, 1.0);
        glm::dvec3 cBL = glm::clamp(trace((x0 + 0.5) * invW, (jyB + 0.5) * invH),
                                    0.0, 1.0);
        glm::dvec3 cBR =
            glm::clamp(trace((ixR + 0.5) * invW, (jyB + 0.5) * invH),
                       0.0, 1.0);

        double maxDiff = 0.0;
        for (int c = 0; c < 3; ++c) {
          double lo = std::min({cTL[c], cTR[c], cBL[c], cBR[c]});
          double hi = std::max({cTL[c], cTR[c], cBL[c], cBR[c]});
          maxDiff = std::max(maxDiff, hi - lo);
        }

        if (maxDiff <= blockInterpThresh) {
          const double denomX = std::max(1, ixR - x0);
          const double denomY = std::max(1, jyB - y0);
          for (int j = y0; j < y1; ++j) {
            if (stopTrace.load(std::memory_order_acquire))
              break;
            const double ty = (j - y0) / denomY;
            for (int i = x0; i < x1; ++i) {
              const double tx = (i - x0) / denomX;
              glm::dvec3 col = (1.0 - tx) * (1.0 - ty) * cTL +
                               tx * (1.0 - ty) * cTR +
                               (1.0 - tx) * ty * cBL + tx * ty * cBR;
              unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;
              pixel[0] = static_cast<unsigned char>(255.0 * col[0]);
              pixel[1] = static_cast<unsigned char>(255.0 * col[1]);
              pixel[2] = static_cast<unsigned char>(255.0 * col[2]);
            }
          }
          continue;
        }
      }

      for (int j = y0; j < y1; ++j) {
        if (stopTrace.load(std::memory_order_acquire))
          break;
        for (int i = x0; i < x1; ++i) {
          tracePixel(i, j);
        }
      }
    }
  }
}

void RayTracer::traceImage(int w, int h) {
  traceSetup(w, h);

  if (causticPhotons > 0 && scene && (usePathIntegrator || useDistributedGI)) {
    emitCausticPhotons();
  }

  if ((usePathIntegrator || useDistributedGI) && temporalSampler && temporalSampler->isEnabled() && hasSceneJsonTemplate &&
      !sceneJsonContents.empty()) {
    const int temporalSamples = std::max(1, traceUI->getMotionBlurSamples());
    std::vector<glm::dvec3> accum(static_cast<size_t>(w) * h,
                                  glm::dvec3(0.0));

    nlohmann::json baseSceneJson;
    try {
      baseSceneJson = nlohmann::json::parse(sceneJsonContents);
    } catch (const nlohmann::json::exception &) {
      renderCurrentScene(w, h);
      return;
    }

    JsonParser parser(sceneJsonDir);
    int usedSamples = 0;
    for (int s = 0; s < temporalSamples; ++s) {
      if (stopTrace.load(std::memory_order_acquire)) {
        break;
      }

      const double sampleTime = temporalSampler->sampleTime();
      nlohmann::json sampledJson = baseSceneJson;
      substituteExpressions(sampledJson, sampleTime);

      Scene *sampledScene = nullptr;
      try {
        sampledScene = parser.parseSceneFromJson(sampledJson);
      } catch (const std::exception &) {
        continue;
      }

      setScene(sampledScene);
      traceSetup(w, h);
      renderCurrentScene(w, h);
      if (traceUI && traceUI->aaSwitch()) {
        // motion blur and AA together by AAing each temporal sample
        aaImage();
      }

      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const size_t idx = static_cast<size_t>(x + y * w);
          const unsigned char *pixel = buffer.data() + idx * 3;
          accum[idx] += glm::dvec3(pixel[0], pixel[1], pixel[2]) / 255.0;
        }
      }
      ++usedSamples;
    }

    if (usedSamples > 0) {
      const double invSamples = 1.0 / static_cast<double>(usedSamples);
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const size_t idx = static_cast<size_t>(x + y * w);
          glm::dvec3 col = glm::clamp(accum[idx] * invSamples, 0.0, 1.0);
          unsigned char *pixel = buffer.data() + idx * 3;
          pixel[0] = static_cast<unsigned char>(255.0 * col[0]);
          pixel[1] = static_cast<unsigned char>(255.0 * col[1]);
          pixel[2] = static_cast<unsigned char>(255.0 * col[2]);
        }
      }
      if (traceUI && traceUI->aaSwitch()) {
        skipNextPostAAPass = true;
      }
      return;
    }
  }

  renderCurrentScene(w, h);
}

void RayTracer::buildPrimaryHitBuffer() {
  primaryHitBuffer.assign(static_cast<size_t>(buffer_width) * buffer_height,
                          PrimaryHitInfo{});
  if (!scene || buffer_width <= 0 || buffer_height <= 0) {
    return;
  }

  if (threads > 0) {
    omp_set_num_threads(threads);
  }

#pragma omp parallel for collapse(2) schedule(dynamic, 1)
  for (int y = 0; y < buffer_height; ++y) {
    for (int x = 0; x < buffer_width; ++x) {
      if (stopTrace.load(std::memory_order_acquire)) {
        continue;
      }

      const double nx = (static_cast<double>(x) + 0.5) /
                        static_cast<double>(buffer_width);
      const double ny = (static_cast<double>(y) + 0.5) /
                        static_cast<double>(buffer_height);
      ray primaryRay(glm::dvec3(0.0), glm::dvec3(0.0), glm::dvec3(1.0),
                     ray::VISIBILITY);
      scene->getCamera().rayThrough(nx, ny, primaryRay);

      isect is;
      PrimaryHitInfo info;
      if (scene->intersect(primaryRay, is)) {
        info.hit = true;
        info.t = std::max(0.0, is.getT());
        glm::dvec3 n = is.getN();
        double nLen = glm::length(n);
        info.normal = (nLen > 1e-12) ? (n / nLen) : glm::dvec3(0.0, 0.0, 1.0);
        info.object = is.getObject();
      }
      primaryHitBuffer[static_cast<size_t>(x + y * buffer_width)] = info;
    }
  }
}

void RayTracer::applyCelOutlineFromPrimaryHits() {
  const size_t pixelCount = static_cast<size_t>(buffer_width) * buffer_height;
  if (primaryHitBuffer.size() != pixelCount || pixelCount == 0 ||
      buffer.empty()) {
    return;
  }

  const double edgeThreshold = traceUI
                                   ? std::clamp(traceUI->getCelEdgeThreshold(),
                                                0.0, 1.0)
                                   : 0.20;
  const int edgeThickness =
      traceUI ? std::max(1, traceUI->getCelEdgeThickness()) : 1;
  constexpr double kDepthEps = 1e-6;

  std::vector<uint8_t> edgeMask(pixelCount, 0);

  for (int y = 0; y < buffer_height; ++y) {
    for (int x = 0; x < buffer_width; ++x) {
      const size_t idx = static_cast<size_t>(x + y * buffer_width);
      const PrimaryHitInfo &center = primaryHitBuffer[idx];
      bool isEdge = false;

      for (int dy = -1; dy <= 1 && !isEdge; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= buffer_width || ny >= buffer_height) {
            continue;
          }

          const size_t nIdx = static_cast<size_t>(nx + ny * buffer_width);
          const PrimaryHitInfo &neighbor = primaryHitBuffer[nIdx];

          if (center.hit != neighbor.hit) {
            isEdge = true;
            break;
          }

          if (!center.hit || !neighbor.hit) {
            continue;
          }

          if (center.object != neighbor.object) {
            isEdge = true;
            break;
          }

          const double minDepth = std::max(std::min(center.t, neighbor.t), kDepthEps);
          const double relativeDepthJump =
              std::abs(center.t - neighbor.t) / minDepth;
          if (relativeDepthJump > edgeThreshold) {
            isEdge = true;
            break;
          }

          const double normalDot =
              std::clamp(glm::dot(center.normal, neighbor.normal), -1.0, 1.0);
          const double normalDiscontinuity = 1.0 - std::abs(normalDot);
          if (normalDiscontinuity > edgeThreshold) {
            isEdge = true;
            break;
          }
        }
      }

      edgeMask[idx] = isEdge ? 1 : 0;
    }
  }

  if (edgeThickness > 1) {
    const int radius = edgeThickness - 1;
    std::vector<uint8_t> dilated = edgeMask;
    for (int y = 0; y < buffer_height; ++y) {
      for (int x = 0; x < buffer_width; ++x) {
        const size_t idx = static_cast<size_t>(x + y * buffer_width);
        if (edgeMask[idx] == 0) {
          continue;
        }
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= buffer_width || ny >= buffer_height) {
              continue;
            }
            dilated[static_cast<size_t>(nx + ny * buffer_width)] = 1;
          }
        }
      }
    }
    edgeMask.swap(dilated);
  }

  if (threads > 0) {
    omp_set_num_threads(threads);
  }
#pragma omp parallel for schedule(dynamic, 256)
  for (int idx = 0; idx < static_cast<int>(pixelCount); ++idx) {
    if (edgeMask[static_cast<size_t>(idx)] == 0) {
      continue;
    }
    unsigned char *pixel = buffer.data() + static_cast<size_t>(idx) * 3;
    pixel[0] = 0;
    pixel[1] = 0;
    pixel[2] = 0;
  }
}

void RayTracer::applyCelPostProcess() {
  if (!traceUI || !traceUI->getCel()) {
    return;
  }
  if (traceUI->tracerMode() == TraceUI::TracerMode::Path) {
    return;
  }
  if (buffer_width <= 0 || buffer_height <= 0 || buffer.empty()) {
    return;
  }

  buildPrimaryHitBuffer();
  applyCelOutlineFromPrimaryHits();
}

int RayTracer::aaImage() {
  if (skipNextPostAAPass) {
    skipNextPostAAPass = false;
    return 0;
  }
  if (threads > 0)
    omp_set_num_threads(threads);

  const int tileSize = std::max(1, block_size);
  const int tilesX = (buffer_width + tileSize - 1) / tileSize;
  const int tilesY = (buffer_height + tileSize - 1) / tileSize;

#pragma omp parallel for collapse(2) schedule(dynamic, 1)
  for (int ty = 0; ty < tilesY; ++ty) {
    for (int tx = 0; tx < tilesX; ++tx) {
      if (stopTrace.load(std::memory_order_acquire))
        continue;

      const int x0 = tx * tileSize;
      const int y0 = ty * tileSize;
      const int x1 = std::min(x0 + tileSize, buffer_width);
      const int y1 = std::min(y0 + tileSize, buffer_height);

      for (int j = y0; j < y1; ++j) {
        if (stopTrace.load(std::memory_order_acquire))
          break;
        for (int i = x0; i < x1; ++i) {
          glm::dvec3 col(0, 0, 0);
          int gridSize = samples;

          if (gridSize < 1)
            gridSize = 1;

          for (int p = 0; p < gridSize; ++p) {
            for (int q = 0; q < gridSize; ++q) {
              double offsetX = (double(p)) / double(gridSize);
              double offsetY = (double(q)) / double(gridSize);
              /*
              double offsetX = (double(p) + 0.5) / double(gridSize);
              double offsetY = (double(q) + 0.5) / double(gridSize);
              */
              double x = (double(i) + offsetX) / double(buffer_width);
              double y = (double(j) + offsetY) / double(buffer_height);

              col += trace(x, y);
            }
          }

          col /= double(gridSize * gridSize);
          col = glm::clamp(col, 0.0, 1.0);

          unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;
          pixel[0] = (int)(255.0 * col[0]);
          pixel[1] = (int)(255.0 * col[1]);
          pixel[2] = (int)(255.0 * col[2]);
          /*
          pixel[0] = (int)(255.0 * col[0] + 0.5);
          pixel[1] = (int)(255.0 * col[1] + 0.5);
          pixel[2] = (int)(255.0 * col[2] + 0.5);
          */
        }
      }
    }
  }
  return 0;
}

int RayTracer::aaStochasticImage() {
  if (skipNextPostAAPass) {
    skipNextPostAAPass = false;
    return 0;
  }
  if (threads > 0)
    omp_set_num_threads(threads);

  const int tileSize = std::max(1, block_size);
  const int tilesX = (buffer_width + tileSize - 1) / tileSize;
  const int tilesY = (buffer_height + tileSize - 1) / tileSize;

  // seed from (i,j,p,q,dim)
  auto jitterCoord = [](int i, int j, int p, int q, int dim) -> double {
    unsigned seed =
        static_cast<unsigned>(i) * 31u * 31u * 31u * 31u +
        static_cast<unsigned>(j) * 31u * 31u * 31u +
        static_cast<unsigned>(p) * 31u * 31u +
        static_cast<unsigned>(q) * 31u +
        static_cast<unsigned>(dim);
    std::minstd_rand gen(seed);
    std::uniform_real_distribution<double> dis(0.0, 1.0);
    return dis(gen);
  };

#pragma omp parallel for collapse(2) schedule(dynamic, 1)
  for (int ty = 0; ty < tilesY; ++ty) {
    for (int tx = 0; tx < tilesX; ++tx) {
      if (stopTrace.load(std::memory_order_acquire))
        continue;

      const int x0 = tx * tileSize;
      const int y0 = ty * tileSize;
      const int x1 = std::min(x0 + tileSize, buffer_width);
      const int y1 = std::min(y0 + tileSize, buffer_height);

      for (int j = y0; j < y1; ++j) {
        if (stopTrace.load(std::memory_order_acquire))
          break;
        for (int i = x0; i < x1; ++i) {
          glm::dvec3 col(0, 0, 0);
          int gridSize = samples;

          if (gridSize < 1)
            gridSize = 1;

          for (int p = 0; p < gridSize; ++p) {
            for (int q = 0; q < gridSize; ++q) {
              double jx = jitterCoord(i, j, p, q, 0);
              double jy = jitterCoord(i, j, p, q, 1);
              double offsetX = (double(p) + jx) / double(gridSize);
              double offsetY = (double(q) + jy) / double(gridSize);
              double x = (double(i) + offsetX) / double(buffer_width);
              double y = (double(j) + offsetY) / double(buffer_height);

              col += trace(x, y);
            }
          }

          col /= double(gridSize * gridSize);
          col = glm::clamp(col, 0.0, 1.0);

          unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;
          pixel[0] = (int)(255.0 * col[0]);
          pixel[1] = (int)(255.0 * col[1]);
          pixel[2] = (int)(255.0 * col[2]);
        }
      }
    }
  }
  return 0;
}

glm::dvec3 RayTracer::aaARecurse(double x, double y, double w, double h,
                                 int depth, int maxDepth, const glm::dvec3 &cTL,
                                 const glm::dvec3 &cTR, const glm::dvec3 &cBL,
                                 const glm::dvec3 &cBR) {
  double halfW = w * 0.5;
  double halfH = h * 0.5;

  glm::dvec3 cCenter = trace((x + halfW) / double(buffer_width),
                             (y + halfH) / double(buffer_height));

  if (depth >= maxDepth) {
    return (cTL + cTR + cBL + cBR + cCenter) * 0.2;
  }

  double d1 = glm::length(cTL - cCenter);
  double d2 = glm::length(cTR - cCenter);
  double d3 = glm::length(cBL - cCenter);
  double d4 = glm::length(cBR - cCenter);

  if (d1 <= aaThresh && d2 <= aaThresh && d3 <= aaThresh && d4 <= aaThresh) {
    return (cTL + cTR + cBL + cBR + cCenter) * 0.2;
  }

  glm::dvec3 cTop =
      trace((x + halfW) / double(buffer_width), y / double(buffer_height));
  glm::dvec3 cBottom = trace((x + halfW) / double(buffer_width),
                             (y + h) / double(buffer_height));
  glm::dvec3 cLeft =
      trace(x / double(buffer_width), (y + halfH) / double(buffer_height));
  glm::dvec3 cRight = trace((x + w) / double(buffer_width),
                            (y + halfH) / double(buffer_height));

  // top left
  glm::dvec3 q1 = aaARecurse(x, y, halfW, halfH, depth + 1, maxDepth, cTL, cTop,
                             cLeft, cCenter);
  // top right
  glm::dvec3 q2 = aaARecurse(x + halfW, y, halfW, halfH, depth + 1, maxDepth,
                             cTop, cTR, cCenter, cRight);
  // bottom left
  glm::dvec3 q3 = aaARecurse(x, y + halfH, halfW, halfH, depth + 1, maxDepth,
                             cLeft, cCenter, cBL, cBottom);
  // bottom right
  glm::dvec3 q4 = aaARecurse(x + halfW, y + halfH, halfW, halfH, depth + 1,
                             maxDepth, cCenter, cRight, cBottom, cBR);

  // average
  return (q1 + q2 + q3 + q4) * 0.25;
}

std::pair<glm::dvec3, int> RayTracer::aaARecurseWithCount(
    double x, double y, double w, double h, int depth, int maxDepth,
    const glm::dvec3 &cTL, const glm::dvec3 &cTR, const glm::dvec3 &cBL,
    const glm::dvec3 &cBR) {
  double halfW = w * 0.5;
  double halfH = h * 0.5;

  glm::dvec3 cCenter = trace((x + halfW) / double(buffer_width),
                             (y + halfH) / double(buffer_height));
  int count = 1;

  if (depth >= maxDepth) {
    return {(cTL + cTR + cBL + cBR + cCenter) * 0.2, count};
  }

  double d1 = glm::length(cTL - cCenter);
  double d2 = glm::length(cTR - cCenter);
  double d3 = glm::length(cBL - cCenter);
  double d4 = glm::length(cBR - cCenter);

  if (d1 <= aaThresh && d2 <= aaThresh && d3 <= aaThresh && d4 <= aaThresh) {
    return {(cTL + cTR + cBL + cBR + cCenter) * 0.2, count};
  }

  glm::dvec3 cTop =
      trace((x + halfW) / double(buffer_width), y / double(buffer_height));
  glm::dvec3 cBottom = trace((x + halfW) / double(buffer_width),
                             (y + h) / double(buffer_height));
  glm::dvec3 cLeft =
      trace(x / double(buffer_width), (y + halfH) / double(buffer_height));
  glm::dvec3 cRight = trace((x + w) / double(buffer_width),
                            (y + halfH) / double(buffer_height));
  count += 4;

  auto [q1, c1] =
      aaARecurseWithCount(x, y, halfW, halfH, depth + 1, maxDepth,
                          cTL, cTop, cLeft, cCenter);
  auto [q2, c2] = aaARecurseWithCount(x + halfW, y, halfW, halfH, depth + 1,
                                      maxDepth, cTop, cTR, cCenter, cRight);
  auto [q3, c3] = aaARecurseWithCount(x, y + halfH, halfW, halfH, depth + 1,
                                      maxDepth, cLeft, cCenter, cBL, cBottom);
  auto [q4, c4] = aaARecurseWithCount(x + halfW, y + halfH, halfW, halfH,
                                      depth + 1, maxDepth, cCenter, cRight,
                                      cBottom, cBR);
  count += c1 + c2 + c3 + c4;

  return {(q1 + q2 + q3 + q4) * 0.25, count};
}

int RayTracer::aaAdaptiveImage() {
  if (skipNextPostAAPass) {
    skipNextPostAAPass = false;
    return 0;
  }
  if (threads > 0)
    omp_set_num_threads(threads);

  const int tileSize = std::max(1, block_size);
  const int tilesX = (buffer_width + tileSize - 1) / tileSize;
  const int tilesY = (buffer_height + tileSize - 1) / tileSize;

#pragma omp parallel for collapse(2) schedule(dynamic, 1)
  for (int ty = 0; ty < tilesY; ++ty) {
    for (int tx = 0; tx < tilesX; ++tx) {
      if (stopTrace.load(std::memory_order_acquire))
        continue;

      const int x0 = tx * tileSize;
      const int y0 = ty * tileSize;
      const int x1 = std::min(x0 + tileSize, buffer_width);
      const int y1 = std::min(y0 + tileSize, buffer_height);

      for (int j = y0; j < y1; ++j) {
        if (stopTrace.load(std::memory_order_acquire))
          break;
        for (int i = x0; i < x1; ++i) {
          double x = double(i);
          double y = double(j);
          double w = 1.0;
          double h = 1.0;

          glm::dvec3 cTL =
              trace(x / double(buffer_width), y / double(buffer_height));
          glm::dvec3 cTR =
              trace((x + w) / double(buffer_width), y / double(buffer_height));
          glm::dvec3 cBL =
              trace(x / double(buffer_width), (y + h) / double(buffer_height));
          glm::dvec3 cBR = trace((x + w) / double(buffer_width),
                                 (y + h) / double(buffer_height));

          int maxDepth = (int)std::ceil(std::log2((double)samples));
          if (maxDepth < 1)
            maxDepth = 1;
          if (maxDepth > 5)
            maxDepth = 5;

          glm::dvec3 col;
          int rayCount = 4;
          if (aaVisualize) {
            auto [c, cnt] =
                aaARecurseWithCount(x, y, w, h, 0, maxDepth, cTL, cTR, cBL, cBR);
            col = c;
            rayCount += cnt;
            int idx = (i + j * buffer_width) * 3;
            if (idx + 2 < static_cast<int>(aaVisBuffer.size())) {
              double t = std::min(1.0, double(rayCount) / 150.0);
              unsigned char val = static_cast<unsigned char>(255.0 * t);
              aaVisBuffer[idx] = aaVisBuffer[idx + 1] = aaVisBuffer[idx + 2] =
                  val;
            }
          } else {
            col = aaARecurse(x, y, w, h, 0, maxDepth, cTL, cTR, cBL, cBR);
          }

          col = glm::clamp(col, 0.0, 1.0);
          unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;
          pixel[0] = (int)(255.0 * col[0]);
          pixel[1] = (int)(255.0 * col[1]);
          pixel[2] = (int)(255.0 * col[2]);
          /*
          pixel[0] = (int)(255.0 * col[0] + 0.5);
          pixel[1] = (int)(255.0 * col[1] + 0.5);
          pixel[2] = (int)(255.0 * col[2] + 0.5);
          */
        }
      }
    }
  }
  return 0;
}

int RayTracer::aaFastApproxImage() {
  if (skipNextPostAAPass) {
    skipNextPostAAPass = false;
    return 0;
  }
  if (threads > 0)
    omp_set_num_threads(threads);

  const std::vector<unsigned char> srcBuffer = buffer;

  auto fetchColor = [&](int x, int y) -> glm::dvec3 {
    x = std::max(0, std::min(buffer_width - 1, x));
    y = std::max(0, std::min(buffer_height - 1, y));
    const unsigned char *pixel = srcBuffer.data() + (x + y * buffer_width) * 3;
    return glm::dvec3(pixel[0] / 255.0, pixel[1] / 255.0, pixel[2] / 255.0);
  };

  auto sampleColor = [&](double x, double y) -> glm::dvec3 {
    x = std::max(0.0, std::min(x, double(buffer_width - 1)));
    y = std::max(0.0, std::min(y, double(buffer_height - 1)));
    int x0 = static_cast<int>(std::floor(x));
    int y0 = static_cast<int>(std::floor(y));
    int x1 = std::min(x0 + 1, buffer_width - 1);
    int y1 = std::min(y0 + 1, buffer_height - 1);
    double tx = x - double(x0);
    double ty = y - double(y0);

    glm::dvec3 c00 = fetchColor(x0, y0);
    glm::dvec3 c10 = fetchColor(x1, y0);
    glm::dvec3 c01 = fetchColor(x0, y1);
    glm::dvec3 c11 = fetchColor(x1, y1);

    return interp::bilerp(c00, c10, c01, c11, tx, ty);
  };

  double contrastThreshold = 1.0 / 255.0;
  if (aaThresh > 0.0)
    contrastThreshold = std::min(contrastThreshold, aaThresh);

  const int tileSize = std::max(1, block_size);
  const int tilesX = (buffer_width + tileSize - 1) / tileSize;
  const int tilesY = (buffer_height + tileSize - 1) / tileSize;

#pragma omp parallel for collapse(2) schedule(dynamic, 1)
  for (int ty = 0; ty < tilesY; ++ty) {
    for (int tx = 0; tx < tilesX; ++tx) {
      if (stopTrace.load(std::memory_order_acquire))
        continue;

      const int x0 = tx * tileSize;
      const int y0 = ty * tileSize;
      const int x1 = std::min(x0 + tileSize, buffer_width);
      const int y1 = std::min(y0 + tileSize, buffer_height);

      for (int j = y0; j < y1; ++j) {
        if (stopTrace.load(std::memory_order_acquire))
          break;
        for (int i = x0; i < x1; ++i) {
          glm::dvec3 c = fetchColor(i, j);

          glm::dvec3 cN = fetchColor(i, j - 1);
          glm::dvec3 cS = fetchColor(i, j + 1);
          glm::dvec3 cE = fetchColor(i + 1, j);
          glm::dvec3 cW = fetchColor(i - 1, j);

          double lC = interp::luminance601(c);
          double lN = interp::luminance601(cN);
          double lS = interp::luminance601(cS);
          double lE = interp::luminance601(cE);
          double lW = interp::luminance601(cW);

          double lumaMin =
              std::min(lC, std::min(std::min(lN, lS), std::min(lE, lW)));
          double lumaMax =
              std::max(lC, std::max(std::max(lN, lS), std::max(lE, lW)));
          double contrast = lumaMax - lumaMin;

          if (contrast < contrastThreshold) {
            setPixel(i, j, c);
            continue;
          }

          glm::dvec3 filtered = (c + cN + cS + cE + cW) * 0.2;
          double blend =
              std::clamp(contrast / (contrast + contrastThreshold), 0.0, 1.0);
          blend *= 0.9;
          glm::dvec3 finalColor = c * (1.0 - blend) + filtered * blend;

          finalColor = glm::clamp(finalColor, 0.0, 1.0);
          setPixel(i, j, finalColor);
        }
      }
    }
  }

  return 0;
}

int RayTracer::aaNeuralImage() {
  if (skipNextPostAAPass) {
    skipNextPostAAPass = false;
    return 0;
  }
  char path[PATH_MAX];
  ssize_t count = readlink("/proc/self/exe", path, PATH_MAX);
  std::string exeDir = (count > 0) ? std::string(path, count) : "";
  size_t last_slash = exeDir.find_last_of('/');
  if (last_slash != std::string::npos) {
      exeDir = exeDir.substr(0, last_slash);
  }

  std::string modelFile = exeDir + "/neural_aa_best.onnx";

  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "aaNeuralImage");
  Ort::SessionOptions sessionOptions;
  sessionOptions.SetIntraOpNumThreads(threads);
  sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);

  Ort::Session session(env, modelFile.c_str(), sessionOptions);

  // buffer to tensor
  // RayTracer Buffer: [H, W, 3] (RGBRGB...) values 0-255
  // model input:      [1, 3, H, W] (RRR...GGG...BBB...) values 0.0-1.0

  std::vector<int64_t> inputShape = {1, 3, buffer_height, buffer_width};
  size_t inputTensorSize = 3 * buffer_height * buffer_width;
  std::vector<float> inputTensorValues(inputTensorSize);

  #pragma omp parallel for collapse(2)
  for (int y = 0; y < buffer_height; ++y) {
      for (int x = 0; x < buffer_width; ++x) {
          int pixelIdx = (x + y * buffer_width) * 3;
          
          float r = buffer[pixelIdx + 0] / 255.0f;
          float g = buffer[pixelIdx + 1] / 255.0f;
          float b = buffer[pixelIdx + 2] / 255.0f;

          // [batch, channel, height, width]
          size_t planeSize = buffer_width * buffer_height;
          size_t offset = y * buffer_width + x;

          inputTensorValues[offset] = r;                 
          inputTensorValues[planeSize + offset] = g;     
          inputTensorValues[2 * planeSize + offset] = b;
      }
  }

  auto memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
      memoryInfo, inputTensorValues.data(), inputTensorSize, inputShape.data(), inputShape.size());

  // const char* inputNames[] = {"input"};
  // const char* outputNames[] = {"output"};

  Ort::AllocatorWithDefaultOptions allocator;

  auto inputNamePtr = session.GetInputNameAllocated(0, allocator);
  auto outputNamePtr = session.GetOutputNameAllocated(0, allocator);

  const char* inputNames[] = { inputNamePtr.get() };
  const char* outputNames[] = { outputNamePtr.get() };

  auto outputTensors = session.Run(
      Ort::RunOptions{nullptr}, inputNames, &inputTensor, 1, outputNames, 1);

  // tensor to buffer
  float* floatArr = outputTensors.front().GetTensorMutableData<float>();

  #pragma omp parallel for collapse(2)
  for (int y = 0; y < buffer_height; ++y) {
      for (int x = 0; x < buffer_width; ++x) {
          size_t planeSize = buffer_width * buffer_height;
          size_t offset = y * buffer_width + x;

          float r = floatArr[offset];
          float g = floatArr[planeSize + offset];
          float b = floatArr[2 * planeSize + offset];

          r = std::max(0.0f, std::min(1.0f, r));
          g = std::max(0.0f, std::min(1.0f, g));
          b = std::max(0.0f, std::min(1.0f, b));

          unsigned char* pixel = buffer.data() + (x + y * buffer_width) * 3;
          pixel[0] = static_cast<unsigned char>(r * 255.0f);
          pixel[1] = static_cast<unsigned char>(g * 255.0f);
          pixel[2] = static_cast<unsigned char>(b * 255.0f);
      }
  }

  return 0;
}


bool RayTracer::checkRender() {
  // YOUR CODE HERE
  // FIXME: Return true if tracing is done.
  //        This is a helper routine for GUI.
  //
  // TIPS: Introduce an array to track the status of each worker thread.
  //       This array is maintained by the worker threads.

  // openmp blocks main thread until rendering is completed
  return true;
}

void RayTracer::waitRender() {
  // YOUR CODE HERE
  // FIXME: Wait until the rendering process is done.
  //        This function is essential if you are using an asynchronous
  //        traceImage implementation.
  //
  // TIPS: Join all worker threads here.

  // openmp automatically waits/joins all threads before the functino returns
}

glm::dvec3 RayTracer::getPixel(int i, int j) {
  unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;
  return glm::dvec3((double)pixel[0] / 255.0, (double)pixel[1] / 255.0,
                    (double)pixel[2] / 255.0);
}

void RayTracer::setPixel(int i, int j, glm::dvec3 color) {
  unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;

  pixel[0] = (int)(255.0 * color[0]);
  pixel[1] = (int)(255.0 * color[1]);
  pixel[2] = (int)(255.0 * color[2]);
}
