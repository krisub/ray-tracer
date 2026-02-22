#include "PhotonMap.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void PhotonMap::balance() {
  if (photons_.empty()) return;
  buildKdTree(0, static_cast<int>(photons_.size()));
}

void PhotonMap::buildKdTree(int start, int end) {
  if (end - start <= 1) {
    if (start < end) photons_[start].splitAxis = 0;
    return;
  }

  glm::dvec3 lo(1e30), hi(-1e30);
  for (int i = start; i < end; ++i) {
    lo = glm::min(lo, photons_[i].position);
    hi = glm::max(hi, photons_[i].position);
  }
  glm::dvec3 extent = hi - lo;
  int axis = 0;
  if (extent.y > extent.x) axis = 1;
  if (extent.z > extent[axis]) axis = 2;

  int median = (start + end) / 2;

  std::nth_element(
      photons_.begin() + start,
      photons_.begin() + median,
      photons_.begin() + end,
      [axis](const Photon &a, const Photon &b) {
        return a.position[axis] < b.position[axis];
      });

  photons_[median].splitAxis = static_cast<short>(axis);

  buildKdTree(start, median);
  buildKdTree(median + 1, end);
}


void PhotonMap::locatePhotons(int start, int end,
                              const glm::dvec3 &pos,
                              double &maxDistSq, int maxPhotons,
                              std::vector<NearestPhoton> &heap) const {
  if (start >= end) return;

  int median = (start + end) / 2;
  const Photon &p = photons_[median];
  int axis = p.splitAxis;
  double delta = pos[axis] - p.position[axis];


  if (delta < 0.0) {
    locatePhotons(start, median, pos, maxDistSq, maxPhotons, heap);
    if (delta * delta < maxDistSq)
      locatePhotons(median + 1, end, pos, maxDistSq, maxPhotons, heap);
  } else {
    locatePhotons(median + 1, end, pos, maxDistSq, maxPhotons, heap);
    if (delta * delta < maxDistSq)
      locatePhotons(start, median, pos, maxDistSq, maxPhotons, heap);
  }

  glm::dvec3 diff = p.position - pos;
  double distSq = glm::dot(diff, diff);
  if (distSq < maxDistSq) {
    NearestPhoton np{distSq, &p};
    if (static_cast<int>(heap.size()) < maxPhotons) {
      heap.push_back(np);
      std::push_heap(heap.begin(), heap.end());
    } else {
      if (distSq < heap.front().distSq) {
        std::pop_heap(heap.begin(), heap.end());
        heap.back() = np;
        std::push_heap(heap.begin(), heap.end());
        maxDistSq = heap.front().distSq;
      }
    }
  }
}

glm::dvec3 PhotonMap::irradianceEstimate(const glm::dvec3 &pos,
                                         const glm::dvec3 &normal,
                                         double maxDist,
                                         int maxPhotons) const {
  if (empty()) return glm::dvec3(0.0);

  double maxDistSq = maxDist * maxDist;
  std::vector<NearestPhoton> heap;
  heap.reserve(maxPhotons + 1);

  locatePhotons(0, static_cast<int>(photons_.size()),
                pos, maxDistSq, maxPhotons, heap);

  if (heap.empty()) return glm::dvec3(0.0);

  glm::dvec3 flux(0.0);
  for (const auto &np : heap) {
    if (glm::dot(np.photon->incomingDir, normal) < 0.0) {
      flux += np.photon->power;
    }
  }

  double r2 = heap.front().distSq; // max heap
  if (r2 < 1e-20) return glm::dvec3(0.0);

  return flux / (M_PI * r2);
}
