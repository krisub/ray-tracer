#ifndef __RAYTRACER_H__
#define __RAYTRACER_H__

#define MAX_THREADS 32

// The main ray tracer.

#include "scene/cubeMap.h"
#include "scene/ray.h"
#include <atomic>
#include <glm/vec3.hpp>
#include <mutex>
#include <queue>
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
  double aspectRatio();

  void traceImage(int w, int h);
  int aaImage();
  int aaAdaptiveImage();
  int aaFastApproxImage();
  bool checkRender();
  void waitRender();

  void traceSetup(int w, int h);

  bool loadScene(const char *fn);
  bool sceneLoaded() { return scene != 0; }

  void setReady(bool ready) { m_bBufferReady = ready; }
  bool isReady() const { return m_bBufferReady; }

  const Scene &getScene() { return *scene; }

  std::atomic<bool> stopTrace{false};

private:
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

  std::unique_ptr<Scene> scene;
  std::vector<unsigned char> buffer;
  double thresh;
  int buffer_width, buffer_height;
  bool m_bBufferReady;

  int bufferSize;
  unsigned int threads;
  int block_size;
  double aaThresh;
  int samples;

  bool tir;           // total internal reflection
  bool handleOverlap; // for multiple overlapping transparent objects


};

#endif // __RAYTRACER_H__
