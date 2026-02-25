#include "cubeMap.h"
#include "../scene/material.h"
#include "../ui/TraceUI.h"
#include "ray.h"
#include <algorithm>
#include <cmath>
extern TraceUI *traceUI;

glm::dvec3 CubeMap::getColor(ray r) const {
  // for each face
  //   - face normal n
  //   - "right" axis r (increasing u)
  //   - "up" axis    u (increasing v)
  // then compute:
  //   s = dot(d, right) / dot(d, n)
  //   t = dot(d, up)    / dot(d, n)
  // then map (s,t) from [-1,1] to (u,v) in [0,1].

  glm::dvec3 d = r.getDirection();
  double len = glm::length(d);
  if (len <= 0.0) {
    return glm::dvec3(0.0);
  }
  d /= len;

  const double ax = std::abs(d.x);
  const double ay = std::abs(d.y);
  const double az = std::abs(d.z);

  int face = -1; // 0:+X 1:-X 2:+Y 3:-Y 4:+Z 5:-Z
  double u = 0.5;
  double v = 0.5;

  glm::dvec3 n(0.0), right(0.0), up(0.0);
  double denom = 0.0;

  if (ax >= ay && ax >= az) {
    // X major
    if (d.x >= 0.0) {
      face = 0;                // +X
      n = glm::dvec3(1, 0, 0); // right-handed, +Y up
      right = glm::dvec3(0, 0, -1);
      up = glm::dvec3(0, 1, 0);
      denom = d.x;
    } else {
      face = 1;                 // -X
      n = glm::dvec3(-1, 0, 0); // looking toward -X
      right = glm::dvec3(0, 0, 1);
      up = glm::dvec3(0, 1, 0);
      denom = -d.x;
    }
  } else if (ay >= ax && ay >= az) {
    // Y major
    if (d.y >= 0.0) {
      face = 2;                // +Y
      n = glm::dvec3(0, 1, 0);
      // right = glm::dvec3(1, 0, 0);
      // up = glm::dvec3(0, 0, -1);
      right = glm::dvec3(-1, 0, 0);  // was (1, 0, 0)
      up = glm::dvec3(0, 0, 1);      // was (0, 0, -1)
      denom = d.y;
    } else {
      face = 3;                 // -Y
      n = glm::dvec3(0, -1, 0);
      // right = glm::dvec3(1, 0, 0);
      // up = glm::dvec3(0, 0, 1);
      right = glm::dvec3(-1, 0, 0);  // was (1, 0, 0)
      up = glm::dvec3(0, 0, -1);     // was (0, 0, 1)
      denom = -d.y;
    }
  } else {
    // Z major
    if (d.z >= 0.0) {
      face = 5;                // +Z
      n = glm::dvec3(0, 0, 1);
      right = glm::dvec3(1, 0, 0);
      up = glm::dvec3(0, 1, 0);
      denom = d.z;
    } else {
      face = 4;                 // -Z
      n = glm::dvec3(0, 0, -1);
      right = glm::dvec3(-1, 0, 0);
      up = glm::dvec3(0, 1, 0);
      denom = -d.z;
    }
  }

  if (denom <= 0.0) {
    return glm::dvec3(0.0);
  }

  const double s = glm::dot(d, right) / denom; // [-1,1]
  const double t = glm::dot(d, up) / denom;    // [-1,1]
  u = (s + 1.0) * 0.5;
  v = (t + 1.0) * 0.5;

  u = 1.0 - u;

  if (face < 0 || face > 5 || !tMap[face]) {
    return glm::dvec3(0.0);
  }

  auto clamp01 = [](double x) { return std::max(0.0, std::min(1.0, x)); };
  u = clamp01(u);
  v = clamp01(v);

  const int fw = traceUI ? std::max(1, traceUI->getFilterWidth()) : 1;
  if (fw <= 1) {
    return tMap[face]->getMappedValue(glm::dvec2(u, v));
  }

  const int texW = std::max(1, tMap[face]->getWidth());
  const int texH = std::max(1, tMap[face]->getHeight());
  const double du = 1.0 / static_cast<double>(texW);
  const double dv = 1.0 / static_cast<double>(texH);

  const int half = fw / 2;
  glm::dvec3 acc(0.0);
  int samples = 0;
  for (int j = -half; j <= half; ++j) {
    for (int i = -half; i <= half; ++i) {
      double su = clamp01(u + static_cast<double>(i) * du);
      double sv = clamp01(v + static_cast<double>(j) * dv);
      acc += tMap[face]->getMappedValue(glm::dvec2(su, sv));
      samples++;
    }
  }
  return (samples > 0) ? (acc / static_cast<double>(samples)) : acc;
}

CubeMap::CubeMap() {}

CubeMap::~CubeMap() {}

void CubeMap::setNthMap(int n, TextureMap *m) {
  if (m != tMap[n].get())
    tMap[n].reset(m);
}
