#ifndef __PHOTONMAP_H__
#define __PHOTONMAP_H__

/* emitCausticPhotons to balance to irradiance */

#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

struct Photon {
  glm::dvec3 position;       // world-space hit point
  glm::dvec3 power;          // RGB flux carried by this photon
  glm::dvec3 incomingDir;    // unit direction the photon was travelling
  short splitAxis;           // 0/1/2 = x/y/z, set during balancing
};

struct NearestPhoton {
  double distSq;
  const Photon *photon;
  bool operator<(const NearestPhoton &rhs) const {
    return distSq < rhs.distSq;
  }
};

class PhotonMap {
public:
  PhotonMap() = default;

  void reserve(size_t n) { photons_.reserve(n); }
  void store(const Photon &p) { photons_.push_back(p); }

  void balance();

  glm::dvec3 irradianceEstimate(const glm::dvec3 &pos,
                                const glm::dvec3 &normal,
                                double maxDist,
                                int maxPhotons) const;

  size_t size() const { return photons_.size(); }
  bool empty() const { return photons_.empty(); }

private:
  std::vector<Photon> photons_;  // flat kd-tree after balance()

  void buildKdTree(int start, int end);

  // knn
  void locatePhotons(int start, int end,
                     const glm::dvec3 &pos,
                     double &maxDistSq, int maxPhotons,
                     std::vector<NearestPhoton> &heap) const;
};

#endif // __PHOTONMAP_H__
