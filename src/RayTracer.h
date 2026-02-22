#ifndef __RAYTRACER_H__
#define __RAYTRACER_H__

#define MAX_THREADS 32

// The main ray tracer.

#include "scene/cubeMap.h"
#include "scene/ray.h"
#include "BRDF.h"
#include "CameraEffects.h"
#include <atomic>
#include <glm/vec3.hpp>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <time.h>

class Scene;
class Material;
class SceneObject;
class Pixel {
public:
  Pixel(int i, int j, unsigned char *ptr) : ix(i), jy(j), value(ptr) {}

  int ix;
  int jy;
  unsigned char *value;
};


class RayTracer {
public:
  RayTracer();
  ~RayTracer();

  glm::dvec3 tracePixel(int i, int j);
  glm::dvec3 traceRay(ray &r, const glm::dvec3 &thresh, int depth,
                      double &length);

  glm::dvec3 getPixel(int i, int j);
  void setPixel(int i, int j, glm::dvec3 color);
  void getBuffer(unsigned char *&buf, int &w, int &h);
  void getVisualizationBuffer(unsigned char *&buf, int &w, int &h);
  double aspectRatio();

  void traceImage(int w, int h);
  int aaImage();
  int aaAdaptiveImage();
  int aaFastApproxImage();
  int aaStochasticImage();
  int aaNeuralImage();
  bool checkRender();
  void waitRender();

  void traceSetup(int w, int h);

  bool loadScene(const char *fn);
  bool sceneLoaded() { return scene != 0; }

  void setReady(bool ready) { m_bBufferReady = ready; }
  bool isReady() const { return m_bBufferReady; }

  const Scene &getScene() { return *scene; }
  void setScene(Scene *s);

  std::atomic<bool> stopTrace{false};

private:
  void renderCurrentScene(int w, int h);
  glm::dvec3 trace(double x, double y);
  glm::dvec3 computeReflectionDirection(const glm::dvec3 &incidentDir,
                                        const glm::dvec3 &surfaceNormal);
  glm::dvec3 computeRefractionDirection(const glm::dvec3 &incidentDir,
                                        const glm::dvec3 &surfaceNormal,
                                        double iorRatio);
  void accumulateReflection(ray &r, const isect &i, const glm::dvec3 &Kr,
                            const glm::dvec3 &thresh, int depth,
                            glm::dvec3 &colorC);
  void accumulateRefraction(ray &r, const isect &i, const Material &m,
                            const glm::dvec3 &Kt, const glm::dvec3 &thresh,
                            int depth, glm::dvec3 &colorC);
  glm::dvec3 traceRayOverlap(ray &r, const glm::dvec3 &thresh, int depth,
                             double &t,
                             const std::vector<const SceneObject *> &objStack);
  void accumulateReflectionOverlap(ray &r, const isect &i,
                                   const glm::dvec3 &Kr,
                                   const glm::dvec3 &thresh, int depth,
                                   glm::dvec3 &colorC,
                                   const std::vector<const SceneObject *> &objStack);
  void accumulateRefractionOverlap(ray &r, const isect &i, const Material &m,
                                   const glm::dvec3 &Kt,
                                   const glm::dvec3 &thresh, int depth,
                                   glm::dvec3 &colorC,
                                   const std::vector<const SceneObject *> &objStack);
  static glm::dvec3 integratedEmissiveAlongPath(const glm::dvec3 &Le, const glm::dvec3 &Kt, double t);
  double averageIorFromObjects(
      const std::vector<const SceneObject *> &objStack,
      const isect &i) const;
  glm::dvec3 averageKtFromObjects(
      const std::vector<const SceneObject *> &objStack,
      const isect &i) const;
  glm::dvec3 backgroundColor() const;

  glm::dvec3 aaARecurse(double x, double y, double w, double h, int depth,
    int maxDepth, const glm::dvec3 &cTL,
    const glm::dvec3 &cTR, const glm::dvec3 &cBL,
    const glm::dvec3 &cBR);
  std::pair<glm::dvec3, int> aaARecurseWithCount(double x, double y, double w,
    double h, int depth, int maxDepth, const glm::dvec3 &cTL,
    const glm::dvec3 &cTR, const glm::dvec3 &cBL,
    const glm::dvec3 &cBR);

  glm::dvec3 computeIndirectIllumination(const ray &r, const isect &i,
                                         const BRDF &brdf,
                                         const glm::dvec3 &thresh, int depth,
                                         int numSamples);
  

  glm::dvec3 traceIndirectRay(const glm::dvec3 &hitPoint, 
                              const glm::dvec3 &direction,
                              const glm::dvec3 &thresh, int depth);

  glm::dvec3 tracePathRay(ray &r, int maxDepth);

  void setGIEnabled(bool enabled, int numSamples = 16, double intensity = 1.0) {
    giNumSamples = enabled ? numSamples : 0;
    giIntensity = intensity;
  }


  void setDepthOfField(double focalDist, double fNumber) {
    if (!dofLens)
      dofLens = std::make_unique<LensAperture>();
    dofLens->setFocalDistance(focalDist);
    dofLens->setFNumber(fNumber);
    // aperture radius
    // focal_length=1.0 (camera image plane at unit distance from eye)
    // aperture_radius = focal_length / (2 * f_number) = 1 / (2 * f_number)
    // f/2.0 -> radius=0.25, f/4.0 -> radius=0.125, f/8.0 -> radius=0.0625
    dofLens->setApertureRadius(1.0 / (2.0 * fNumber));
    dofLens->setEnabled(fNumber < 100.0);
  }

 
  void setMotionBlur(double shutterOpen, double shutterClose) {
    if (!temporalSampler)
      temporalSampler = std::make_unique<TemporalSampler>();
    temporalSampler->setShutterInterval(shutterOpen, shutterClose);
    temporalSampler->setEnabled(shutterClose > shutterOpen);
  }


  void setAreaLightSamples(int numSamples) {
    areaLightSamples = std::max(1, numSamples);
  }

  void emitCausticPhotons();

  std::unique_ptr<Scene> scene;
  std::vector<unsigned char> buffer;
  double rayTermThresh;    // adaptive ray termination (0=off)
  double blockInterpThresh; // block interpolation (0=off)
  int buffer_width, buffer_height;
  bool m_bBufferReady;

  int bufferSize;
  unsigned int threads;
  int block_size;
  double aaThresh;
  int samples;

  bool tir;           // total internal reflection
  bool handleOverlap; // for multiple overlapping transparent objects
  bool aaVisualize;   // output ray-count visualization for adaptive AA
  bool goopEnabled;   // translucent emissive: integrate emission along path

  int giNumSamples;       // number of samples for global illumination (0=disabled)
  double giIntensity;     // intensity/weight of global illumination contribution
  int areaLightSamples;   // number of samples for soft shadows from area lights
  bool usePathIntegrator;   // true = true path tracing (tracePathRay)
  bool useDistributedGI;    // true = distributed ray tracing with GI (computeIndirectIllumination)
  int pathSPP;              // samples per pixel for full path tracing
  int causticPhotons;       // number of caustic photons to emit (0=disabled)

  std::unique_ptr<LensAperture> dofLens;           // depth of field lens aperture
  std::unique_ptr<TemporalSampler> temporalSampler; // motion blur temporal sampling
  std::string sceneJsonContents;
  std::string sceneJsonDir;
  bool hasSceneJsonTemplate = false;
  bool skipNextPostAAPass = false; // motion-blur path integrated AA already applied

  std::vector<unsigned char> aaVisBuffer;
};
#endif // __RAYTRACER_H__
