#include <cmath>
#include <cstdint>

#include "../SceneObjects/trimesh.h"
#include "../ui/TraceUI.h"
#include "bvh.h"
#include "kdTree.h"
#include "light.h"
#include "ReactionDiffusionTexture.h"
#include "scene.h"
#include <glm/gtx/extended_min_max.hpp>
#include <glm/gtx/io.hpp>
#include <iostream>

using namespace std;
extern TraceUI *traceUI;

namespace {
bool hasValidBounds(const BoundingBox &bounds) {
  const glm::dvec3 min = bounds.getMin();
  const glm::dvec3 max = bounds.getMax();
  return std::isfinite(min.x) && std::isfinite(min.y) && std::isfinite(min.z) &&
         std::isfinite(max.x) && std::isfinite(max.y) && std::isfinite(max.z) &&
         (min.x <= max.x) && (min.y <= max.y) && (min.z <= max.z);
}
} // namespace

bool Geometry::intersect(ray &r, isect &i) const {
  double tmin, tmax;
  if (hasBoundingBoxCapability() && !(bounds.intersect(r, tmin, tmax)))
    return false;
  // Transform the ray into the object's local coordinate space
  glm::dvec3 pos = transform.globalToLocalCoords(r.getPosition());
  glm::dvec3 dir =
      transform.globalToLocalCoords(r.getPosition() + r.getDirection()) - pos;
  double length = glm::length(dir);
  dir = glm::normalize(dir);
  // Backup World pos/dir, and switch to local pos/dir
  glm::dvec3 Wpos = r.getPosition();
  glm::dvec3 Wdir = r.getDirection();
  r.setPosition(pos);
  r.setDirection(dir);
  bool rtrn = false;
  if (intersectLocal(r, i)) {
    // Transform the intersection point & normal returned back into
    // global space.
    i.setN(transform.localToGlobalCoordsNormal(i.getN()));
    if (i.hasTangentFrame()) {
      i.setTangentFrame(transform.localToGlobalCoordsTangent(i.getDpDu()),
                        transform.localToGlobalCoordsTangent(i.getDpDv()));
    }
    i.setT(i.getT() / length);
    rtrn = true;
  }
  // Restore World pos/dir
  r.setPosition(Wpos);
  r.setDirection(Wdir);
  return rtrn;
}

bool Geometry::hasBoundingBoxCapability() const {
  // by default, primitives do not have to specify a bounding box. If this
  // method returns true for a primitive, then either the ComputeBoundingBox()
  // or the ComputeLocalBoundingBox() method must be implemented.

  // If no bounding box capability is supported for an object, that object will
  // be checked against every single ray drawn. This should be avoided whenever
  // possible, but this possibility exists so that new primitives will not have
  // to have bounding boxes implemented for them.
  return false;
}

void Geometry::ComputeBoundingBox() {
  // take the object's local bounding box, transform all 8 points on it,
  // and use those to find a new bounding box.

  BoundingBox localBounds = ComputeLocalBoundingBox();

  glm::dvec3 min = localBounds.getMin();
  glm::dvec3 max = localBounds.getMax();

  glm::dvec4 v, newMax, newMin;

  v = transform.localToGlobalCoords(glm::dvec4(min[0], min[1], min[2], 1));
  newMax = v;
  newMin = v;
  v = transform.localToGlobalCoords(glm::dvec4(max[0], min[1], min[2], 1));
  newMax = glm::max(newMax, v);
  newMin = glm::min(newMin, v);
  v = transform.localToGlobalCoords(glm::dvec4(min[0], max[1], min[2], 1));
  newMax = glm::max(newMax, v);
  newMin = glm::min(newMin, v);
  v = transform.localToGlobalCoords(glm::dvec4(max[0], max[1], min[2], 1));
  newMax = glm::max(newMax, v);
  newMin = glm::min(newMin, v);
  v = transform.localToGlobalCoords(glm::dvec4(min[0], min[1], max[2], 1));
  newMax = glm::max(newMax, v);
  newMin = glm::min(newMin, v);
  v = transform.localToGlobalCoords(glm::dvec4(max[0], min[1], max[2], 1));
  newMax = glm::max(newMax, v);
  newMin = glm::min(newMin, v);
  v = transform.localToGlobalCoords(glm::dvec4(min[0], max[1], max[2], 1));
  newMax = glm::max(newMax, v);
  newMin = glm::min(newMin, v);
  v = transform.localToGlobalCoords(glm::dvec4(max[0], max[1], max[2], 1));
  newMax = glm::max(newMax, v);
  newMin = glm::min(newMin, v);

  bounds.setMax(glm::dvec3(newMax));
  bounds.setMin(glm::dvec3(newMin));
}

Scene::Scene() : kdtree(nullptr), bvhDirty(true) {
  ambientIntensity = glm::dvec3(0, 0, 0);
}

Scene::~Scene() {
  for (auto &obj : objects)
    delete obj;
  for (auto &light : lights)
    delete light;
}

void Scene::add(Geometry *obj) {
  obj->ComputeBoundingBox();
  const bool useInBvh =
      obj->hasBoundingBoxCapability() && hasValidBounds(obj->getBoundingBox());
  if (useInBvh) {
    sceneBounds.merge(obj->getBoundingBox());
    bvhObjects.emplace_back(obj);
  } else {
    nonBvhObjects.emplace_back(obj);
  }
  objects.emplace_back(obj);

  Instance instance;
  instance.meshId = UINT32_MAX;
  instance.object = obj;
  instance.bounds = obj->getBoundingBox();
  if (auto *mesh = dynamic_cast<Trimesh *>(obj)) {
    auto it = meshToBlasId.find(mesh);
    if (it == meshToBlasId.end()) {
      const uint32_t newId = static_cast<uint32_t>(meshBlas.size());
      meshBlas.emplace_back(mesh);
      meshToBlasId.emplace(mesh, newId);
      instance.meshId = newId;
    } else {
      instance.meshId = it->second;
    }
  }
  instances.emplace_back(instance);
  bvhDirty = true;
}

void Scene::add(Light *light) { lights.emplace_back(light); }


void Scene::rebuildBvhIfNeeded() const {
  if (!bvhDirty) {
    return;
  }

  std::lock_guard<std::mutex> guard(bvhBuildMutex);
  if (!bvhDirty) {
    return;
  }

  if (!bvh) {
    bvh = std::make_unique<Bvh>();
  }
  if (traceUI && traceUI->tlasAccelMode() == TraceUI::AccelMode::Wide8) {
    bvh->setLayoutMode(Bvh::LayoutMode::Wide8);
  } else {
    bvh->setLayoutMode(Bvh::LayoutMode::Binary);
  }
  bvh->build(bvhObjects);
  bvhDirty = false;
}

// Get any intersection with an object.  Return information about the
// intersection through the reference parameter.
bool Scene::intersect(ray &r, isect &i) const {
  if (traceUI && !traceUI->kdSwitch()) {
    bool have_one = false;
    for (const auto &obj : objects) {
      isect cur;
      if (obj->intersect(r, cur)) {
        if (!have_one || (cur.getT() < i.getT())) {
          i = cur;
          have_one = true;
        }
      }
    }
    if (!have_one)
      i.setT(1000.0);
    if (TraceUI::m_debug) {
      addToIntersectCache(std::make_pair(new ray(r), new isect(i)));
    }
    return have_one;
  }

  rebuildBvhIfNeeded();

  bool have_one = false;
  if (bvh) {
    have_one = bvh->intersect(r, i);
  }

  for (const auto &obj : nonBvhObjects) {
    isect cur;
    if (obj->intersect(r, cur)) {
      if (!have_one || (cur.getT() < i.getT())) {
        i = cur;
        have_one = true;
      }
    }
  }
  if (!have_one)
    i.setT(1000.0);
  // if debugging,
  if (TraceUI::m_debug) {
    addToIntersectCache(std::make_pair(new ray(r), new isect(i)));
  }
  return have_one;
}

TextureMap *Scene::getTexture(string name) {
  auto itr = textureCache.find(name);
  if (itr == textureCache.end()) {
    textureCache[name].reset(new TextureMap(name));
    return textureCache[name].get();
  }
  return itr->second.get();
}

ReactionDiffusionTexture *Scene::getReactionDiffusionTexture(
    int width, int height, const std::string &pattern, int iterations) {
  std::string key = "rd_" + std::to_string(width) + "_" +
                    std::to_string(height) + "_" + pattern + "_" +
                    std::to_string(iterations);
  auto itr = rdTextureCache.find(key);
  if (itr == rdTextureCache.end()) {
    ReactionDiffusionTexture::Pattern p = ReactionDiffusionTexture::Pattern::Spots;
    if (pattern == "stripes")
      p = ReactionDiffusionTexture::Pattern::Stripes;
    else if (pattern == "wood")
      p = ReactionDiffusionTexture::Pattern::Wood;
    else if (pattern == "marble")
      p = ReactionDiffusionTexture::Pattern::Marble;

    rdTextureCache[key].reset(
        new ReactionDiffusionTexture(width, height, p, iterations));
    return rdTextureCache[key].get();
  }
  return itr->second.get();
}

std::vector<SceneObject *> Scene::getEmissiveObjects() const {
  std::vector<SceneObject *> out;
  isect dummy;
  for (Geometry *g : objects) {
    SceneObject *so = dynamic_cast<SceneObject *>(g);
    if (!so)
      continue;
    const glm::dvec3 ke = so->getMaterial().ke(dummy);
    if (ke.x > 1e-9 || ke.y > 1e-9 || ke.z > 1e-9)
      out.push_back(so);
  }
  return out;
}
