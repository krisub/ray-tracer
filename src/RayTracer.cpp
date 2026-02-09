// The main ray tracer.

#pragma warning(disable : 4786)

#include "RayTracer.h"
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
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <string.h> // for memset

#include <fstream>
#include <iostream>

using namespace std;
extern TraceUI *traceUI;

// Use this variable to decide if you want to print out debugging messages. Gets
// set in the "trace single ray" mode in TraceGLWindow, for example.
bool debugMode = false;

glm::dvec3 RayTracer::computeReflectionDirection(
    const glm::dvec3 &incidentDir, const glm::dvec3 &surfaceNormal) {
  double incidentNormalDot = glm::dot(surfaceNormal, incidentDir);
  return incidentDir - 2.0 * incidentNormalDot * surfaceNormal;
}

glm::dvec3 RayTracer::computeRefractionDirection(
    const glm::dvec3 &incidentDir, const glm::dvec3 &surfaceNormal,
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

void RayTracer::accumulateReflection(ray &r, const isect &i,
                                     const glm::dvec3 &Kr,
                                     const glm::dvec3 &thresh, int depth,
                                     glm::dvec3 &colorC) {
  if (Kr.x > 0 || Kr.y > 0 || Kr.z > 0) {
    glm::dvec3 rayDir = glm::normalize(r.getDirection());
    glm::dvec3 surfaceNormal = glm::normalize(i.getN());
    glm::dvec3 reflectDir =
        computeReflectionDirection(rayDir, surfaceNormal);

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
  if (Kt.x > 0 || Kt.y > 0 || Kt.z > 0) {
    glm::dvec3 rayDir = glm::normalize(r.getDirection());
    glm::dvec3 surfaceNormal = glm::normalize(i.getN());
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
    } else {
      // TODO: add flag to enable TIR
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
      }
    }
  }
}

double RayTracer::averageIorFromObjects(
    const std::vector<const SceneObject *> &objStack,
    const isect &i) const {
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
    const std::vector<const SceneObject *> &objStack,
    const isect &i) const {
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
    ray &r, const isect &i, const glm::dvec3 &Kr,
    const glm::dvec3 &thresh, int depth, glm::dvec3 &colorC,
    const std::vector<const SceneObject *> &objStack) {
  if (Kr.x > 0 || Kr.y > 0 || Kr.z > 0) {
    glm::dvec3 rayDir = glm::normalize(r.getDirection());
    glm::dvec3 surfaceNormal = glm::normalize(i.getN());
    glm::dvec3 reflectDir =
        computeReflectionDirection(rayDir, surfaceNormal);

    ray reflectedRay(r.at(i) + reflectDir * RAY_EPSILON, reflectDir,
                     glm::dvec3(1.0, 1.0, 1.0), ray::REFLECTION);

    double t_val;
    colorC += Kr * traceRayOverlap(reflectedRay, thresh, depth - 1, t_val,
                                   objStack);
  }
}

void RayTracer::accumulateRefractionOverlap(
    ray &r, const isect &i, const Material &m, const glm::dvec3 &Kt,
    const glm::dvec3 &thresh, int depth, glm::dvec3 &colorC,
    const std::vector<const SceneObject *> &objStack) {
  if (Kt.x > 0 || Kt.y > 0 || Kt.z > 0) {
    glm::dvec3 rayDir = glm::normalize(r.getDirection());
    glm::dvec3 surfaceNormal = glm::normalize(i.getN());
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
          traceRayOverlap(refractedRay, thresh, depth - 1, t_val,
                          nextObjStack);
      glm::dvec3 attenKt = averageKtFromObjects(nextObjStack, i);
      incomingLight *= glm::pow(attenKt, glm::dvec3(t_val, t_val, t_val));
      colorC += incomingLight;
    } else {
      if (tir) {
        glm::dvec3 reflectDir =
            computeReflectionDirection(rayDir, normalForRefraction);

        ray reflectedRay(r.at(i) + reflectDir * RAY_EPSILON, reflectDir,
                         glm::dvec3(1.0, 1.0, 1.0), ray::REFLECTION);

        double t_val;
        glm::dvec3 incomingLight =
            traceRayOverlap(reflectedRay, thresh, depth - 1, t_val,
                            objStack);
        glm::dvec3 attenKt = averageKtFromObjects(objStack, i);
        incomingLight *= glm::pow(attenKt, glm::dvec3(t_val, t_val, t_val));
        colorC += incomingLight;
      }
    }
  }
}

glm::dvec3 RayTracer::traceRayOverlap(
    ray &r, const glm::dvec3 &thresh, int depth, double &t,
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

    const Material &m = i.getMaterial();
    colorC = m.shade(scene.get(), r, i);

    glm::dvec3 Kr = m.kr(i);
    accumulateReflectionOverlap(r, i, Kr, thresh, depth, colorC, objStack);

    glm::dvec3 Kt = m.kt(i);
    accumulateRefractionOverlap(r, i, m, Kt, thresh, depth, colorC, objStack);

  } else {
    t = 0.0;
    colorC = backgroundColor();
  }
#if VERBOSE
  std::cerr << "== depth: " << depth + 1 << " done, returning: " << colorC
            << std::endl;
#endif
  return colorC;
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

  ray r(glm::dvec3(0, 0, 0), glm::dvec3(0, 0, 0), glm::dvec3(1, 1, 1),
        ray::VISIBILITY);
  scene->getCamera().rayThrough(x, y, r);
  double dummy;
  glm::dvec3 ret =
      traceRay(r, glm::dvec3(1.0, 1.0, 1.0), traceUI->getDepth(), dummy);
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
  /*
  double x = (double(i) + 0.5) / double(buffer_width);  // og code
  double y = (double(j) + 0.5) / double(buffer_height); // og code
  */

  col = trace(x, y); // og code
  col = glm::clamp(col, 0.0, 1.0);

  unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;
  pixel[0] = (int)(255.0 * col[0]);
  pixel[1] = (int)(255.0 * col[1]);
  pixel[2] = (int)(255.0 * col[2]);

  return col;
}

#define VERBOSE 0

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

    const Material &m = i.getMaterial();
    colorC = m.shade(scene.get(), r, i);


    glm::dvec3 Kr = m.kr(i);
    accumulateReflection(r, i, Kr, thresh, depth, colorC);

    glm::dvec3 Kt = m.kt(i);
    accumulateRefraction(r, i, m, Kt, thresh, depth, colorC);  
    

  } else {
    t = 0.0;
    // std::cout << "no intersection." << std::endl;
    // No intersection. This ray travels to infinity, so we color
    // it according to the background color, which in this (simple)
    // case is just black.
    //
    // FIXME: Add CubeMap support here.
    // TIPS: CubeMap object can be fetched from
    // traceUI->getCubeMap();
    //       Check traceUI->cubeMap() to see if cubeMap is loaded
    //       and enabled.
    colorC = backgroundColor();
  }
#if VERBOSE
  std::cerr << "== depth: " << depth + 1 << " done, returning: " << colorC
            << std::endl;
#endif
  return colorC;
}

RayTracer::RayTracer()
    : scene(nullptr), buffer(0), thresh(0), buffer_width(0), buffer_height(0),
      m_bBufferReady(false) {}

RayTracer::~RayTracer() {}

void RayTracer::getBuffer(unsigned char *&buf, int &w, int &h) {
  buf = buffer.data();
  w = buffer_width;
  h = buffer_height;
}

double RayTracer::aspectRatio() {
  return sceneLoaded() ? scene->getCamera().getAspectRatio() : 1;
}

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
      JsonParser parser(path, ifs);
      scene.reset(parser.parseScene());
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
  size_t newBufferSize = w * h * 3;
  if (newBufferSize != buffer.size()) {
    bufferSize = newBufferSize;
    buffer.resize(bufferSize);
  }
  buffer_width = w;
  buffer_height = h;
  std::fill(buffer.begin(), buffer.end(), 0);
  m_bBufferReady = true;

  /*
   * Sync with TraceUI
   */

  threads = traceUI->getThreads();
  block_size = traceUI->getBlockSize();
  thresh = traceUI->getThreshold();
  samples = traceUI->getSuperSamples();
  aaThresh = traceUI->getAaThreshold();
  tir = traceUI->getTotalInternalReflection();
  handleOverlap = traceUI->getHandleOverlap();

  // YOUR CODE HERE
  // FIXME: Additional initializations
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
void RayTracer::traceImage(int w, int h) {
  // Always call traceSetup before rendering anything.
  traceSetup(w, h);

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

#pragma omp parallel for schedule(dynamic)
  for (int j = 0; j < h; ++j) {
    for (int i = 0; i < w; ++i) {
      tracePixel(i, j);
    }
  }
}

int RayTracer::aaImage() {
  if (threads > 0)
    omp_set_num_threads(threads);

#pragma omp parallel for schedule(dynamic)
  for (int j = 0; j < buffer_height; ++j) {
    for (int i = 0; i < buffer_width; ++i) {

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

int RayTracer::aaAdaptiveImage() {
  if (threads > 0)
    omp_set_num_threads(threads);

#pragma omp parallel for schedule(dynamic)
  for (int j = 0; j < buffer_height; ++j) {
    for (int i = 0; i < buffer_width; ++i) {

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

      glm::dvec3 col = aaARecurse(x, y, w, h, 0, maxDepth, cTL, cTR, cBL, cBR);

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
  return 0;
}

int RayTracer::aaFastApproxImage() {
  if (threads > 0)
    omp_set_num_threads(threads);

  const std::vector<unsigned char> srcBuffer = buffer;

  auto fetchColor = [&](int x, int y) -> glm::dvec3 {
    x = std::max(0, std::min(buffer_width - 1, x));
    y = std::max(0, std::min(buffer_height - 1, y));
    const unsigned char *pixel =
        srcBuffer.data() + (x + y * buffer_width) * 3;
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

    glm::dvec3 c0 = c00 * (1.0 - tx) + c10 * tx;
    glm::dvec3 c1 = c01 * (1.0 - tx) + c11 * tx;
    return c0 * (1.0 - ty) + c1 * ty;
  };

  // https://www.itu.int/dms_pubrec/itu-r/rec/bt/R-REC-BT.601-7-201103-I!!PDF-E.pdf, section 2.5.1
  auto luminance = [](const glm::dvec3 &c) -> double {
    return 0.299 * c.r + 0.587 * c.g + 0.114 * c.b;
  };

  double contrastThreshold = aaThresh;
  if (contrastThreshold <= 0.0)
    contrastThreshold = 1.0 / 255.0;

#pragma omp parallel for schedule(dynamic)
  for (int j = 0; j < buffer_height; ++j) {
    for (int i = 0; i < buffer_width; ++i) {
      glm::dvec3 c = fetchColor(i, j);

      glm::dvec3 cN = fetchColor(i, j - 1);
      glm::dvec3 cS = fetchColor(i, j + 1);
      glm::dvec3 cE = fetchColor(i + 1, j);
      glm::dvec3 cW = fetchColor(i - 1, j);

      double lC = luminance(c);
      double lN = luminance(cN);
      double lS = luminance(cS);
      double lE = luminance(cE);
      double lW = luminance(cW);

      double lumaMin = std::min(lC, std::min(std::min(lN, lS), std::min(lE, lW)));
      double lumaMax = std::max(lC, std::max(std::max(lN, lS), std::max(lE, lW)));
      double contrast = lumaMax - lumaMin;

      if (contrast < contrastThreshold) {
        setPixel(i, j, c);
        continue;
      }

      bool edgeHorizontal = std::abs(lN - lS) >= std::abs(lE - lW);

      double stepX = 0.0;
      double stepY = 0.0;
      if (edgeHorizontal) {
        stepY = (lN < lS) ? -1.0 : 1.0;
      } else {
        stepX = (lW < lE) ? -1.0 : 1.0;
      }

      double blend = std::clamp(contrast / (contrast + contrastThreshold), 0.0, 1.0);
      blend *= 0.5;

      glm::dvec3 shifted =
          sampleColor(double(i) + stepX * 0.5, double(j) + stepY * 0.5);
      glm::dvec3 filtered = (c + shifted) * 0.5;
      glm::dvec3 finalColor = c * (1.0 - blend) + filtered * blend;

      finalColor = glm::clamp(finalColor, 0.0, 1.0);
      setPixel(i, j, finalColor);
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
