// The main ray tracer.

#pragma warning(disable : 4786)

#include "RayTracer.h"
#include "scene/light.h"
#include "scene/material.h"
#include "scene/ray.h"
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
  double x = (double(i) + 0.5) / double(buffer_width);  // og code
  double y = (double(j) + 0.5) / double(buffer_height); // og code

  col = trace(x, y); // og code
  col = glm::clamp(col, 0.0, 1.0);

  unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;
  pixel[0] = (int)(255.0 * col[0]);
  pixel[1] = (int)(255.0 * col[1]);
  pixel[2] = (int)(255.0 * col[2]);

  return col;

  // if (i == buffer_width / 2 && j == buffer_height / 2) {
  //   debugMode = true;
  //   std::cout << "tracing center pixel (" << i << "," << j << ")" <<
  //   std::endl;
  // } else {
  //   debugMode = false;
  // }

  // brute force anti aliasing by supersampling
  // int gridSize = int(sqrt(samples));
  // if (gridSize < 1)
  //   gridSize = 1;

  // for (int p = 0; p < gridSize; ++p) {
  //   for (int q = 0; q < gridSize; ++q) {
  //     // offset based on grid pos
  //     double offsetX = (double(p) + 0.5) / double(gridSize);
  //     double offsetY = (double(q) + 0.5) / double(gridSize);
  //     // pixel coord + offset normalized
  //     double x = (double(i) + offsetX) / double(buffer_width);
  //     double y = (double(j) + offsetY) / double(buffer_height);

  //     col += trace(x, y); // trace subpixel
  //   }
  // }

  // col /= double(gridSize * gridSize); // average

  // unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3; // og
  // code

  // if (debugMode) {
  //   std::cout << "center pixel result: " << col.x << ", " << col.y << ", "
  //             << col.z << std::endl;
  // }

  // col = glm::clamp(col, 0.0, 1.0);
  // pixel[0] =
  //     (int)(255.0 * col[0] + 0.5); // og code maybe add +0.5 to round to
  //     nearest int?
  // pixel[1] = (int)(255.0 * col[1] + 0.5); // og code
  // pixel[2] = (int)(255.0 * col[2] + 0.5); // og code
  // return col;
}

#define VERBOSE 0

// Do recursive ray tracing! You'll want to insert a lot of code here (or places
// called from here) to handle reflection, refraction, etc etc.
glm::dvec3 RayTracer::traceRay(ray &r, const glm::dvec3 &thresh, int depth,
                               double &t) {

  // const double TEST_EPSILON = 1e-3;

  if (depth <= 0) {
    return glm::dvec3(0.0, 0.0, 0.0); // stop when depth is 0
  }

  isect i;
  glm::dvec3 colorC;
#if VERBOSE
  std::cerr << "== current depth: " << depth << std::endl;
#endif

  if (scene->intersect(r, i)) {
    t = i.getT();
    // YOUR CODE HERE
    // std::cout << "hit object at t=" << i.getT() << std::endl;
    // An intersection occurred!  We've got work to do. For now, this code gets
    // the material for the surface that was intersected, and asks that material
    // to provide a color for the ray.x

    // This is a great place to insert code for recursive ray tracing. Instead
    // of just returning the result of shade(), add some more steps: add in the
    // contributions from reflected and refracted rays.

    const Material &m = i.getMaterial();
    colorC = m.shade(scene.get(), r, i);

    // https://www.opengl.org/archives/resources/code/samples/advanced/advanced97/notes/node88.html
    // reflection: angle of incidence = angle of reflection
    // incident direction can be decomposed into two components, one parallel to
    // surface and one perpendicular to surface normal:
    //  refraction: snell's law

    // what is atten in ray.h
    glm::dvec3 Kr = m.kr(i); // reflective component
    if (Kr.x > 0 || Kr.y > 0 || Kr.z > 0) {
      glm::dvec3 rayDir = glm::normalize(r.getDirection());
      glm::dvec3 surfaceNormal = glm::normalize(i.getN());
      glm::dvec3 reflectDir = glm::reflect(rayDir, surfaceNormal);

      ray reflectedRay(r.at(i) + reflectDir * RAY_EPSILON, reflectDir,
                       glm::dvec3(1.0, 1.0, 1.0), ray::REFLECTION);

      double t_val;
      colorC += Kr * traceRay(reflectedRay, thresh, depth - 1, t_val);
    }

    glm::dvec3 Kt = m.kt(i); // refractive (transmissive) component
    if (Kt.x > 0 || Kt.y > 0 || Kt.z > 0) {
      glm::dvec3 rayDir = glm::normalize(r.getDirection());
      glm::dvec3 surfaceNormal = glm::normalize(i.getN());
      double indexOfRefraction = m.index(i);

      /*
      if dot product (cosine of angle of incidence) < 0 we are going into the
      material (hitting the front) otherwise we are leaving the material
      (hitting the back)
      */

      double cosIncidence = glm::dot(surfaceNormal, rayDir);
      // double cosIncidence = glm::dot(rayDir, surfaceNormal);
      double iorRatio;
      glm::dvec3 normalForRefraction;

      // TODO: can we assume that we are always going into/out of air? what
      // about from material to material?
      if (cosIncidence < 0) { // air to material
        iorRatio = 1.0 / indexOfRefraction;
        normalForRefraction = surfaceNormal;
      } else { // material to air.
        iorRatio = indexOfRefraction / 1.0;
        normalForRefraction = -surfaceNormal;
      }

      glm::dvec3 refractionDir =
          glm::refract(rayDir, normalForRefraction, iorRatio);

      if (glm::length(refractionDir) > 0.0) {
        // std::cout << "refraction at t=" << i.getT() << std::endl;
        ray refractedRay(r.at(i) + refractionDir * RAY_EPSILON, refractionDir,
                         glm::dvec3(1.0, 1.0, 1.0), ray::REFRACTION);

        double t_val;
        colorC += Kt * traceRay(refractedRay, thresh, depth - 1, t_val);
      } else { // should reflect back inside?
        // std::cout << "total internal reflection at t=" << i.getT() <<
        // std::endl;
        glm::dvec3 reflectDir = glm::reflect(rayDir, normalForRefraction);

        ray reflectedRay(r.at(i) + reflectDir * RAY_EPSILON, reflectDir,
                         glm::dvec3(1.0, 1.0, 1.0), ray::REFLECTION);

        double t_val;
        colorC += traceRay(reflectedRay, thresh, depth - 1, t_val);
      }
    }

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

    colorC = glm::dvec3(0.0, 0.0, 0.0);
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
  // YOUR CODE HERE
  // FIXME: Implement Anti-aliasing here
  //
  // TIP: samples and aaThresh have been synchronized with TraceUI by
  //      RayTracer::traceSetup() function

  /*
  pixel coordinates (i,j)/(x.y)

  (0, 0) (1, 0) (2, 0) ... (w-1, 0)
  (0, 1) (1, 1) (2, 1) ... (w-1, 1)
  ...
  (0, h-1) (1, h-1) (2, h-1) ... (w-1, h-1)


  */

  if (threads > 0)
    omp_set_num_threads(threads);

#pragma omp parallel for schedule(dynamic)
  for (int j = 0; j < buffer_height; ++j) {
    for (int i = 0; i < buffer_width; ++i) {

      // say j = 1, i = 0
      // and image is 3x3 pixels

      glm::dvec3 col(0, 0, 0);
      int gridSize = samples; // = 3

      if (gridSize < 1)
        gridSize = 1;

      for (int p = 0; p < gridSize; ++p) {
        for (int q = 0; q < gridSize; ++q) {

          // at p-q-0

          // offsetX = 0.1666...
          // offsetY = 0.1666...
          double offsetX = (double(p) + 0.5) / double(gridSize);
          double offsetY = (double(q) + 0.5) / double(gridSize);

          // pixel coord + offset normalized
          // x = (0 + 0.1666...) / 3 = 0.05555...
          // y = (1 + 0.1666...) / 3 = 0.38888...
          // normalizing by image dimensions to pass to trace function
          double x = (double(i) + offsetX) / double(buffer_width);
          double y = (double(j) + offsetY) / double(buffer_height);

          col += trace(x, y);
        }
      }

      col /= double(gridSize * gridSize);
      col = glm::clamp(col, 0.0, 1.0);

      // overwrite pixel in buffer
      unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;
      pixel[0] = (int)(255.0 * col[0] + 0.5); // do i need the +0.5
      pixel[1] = (int)(255.0 * col[1] + 0.5);
      pixel[2] = (int)(255.0 * col[2] + 0.5);
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
