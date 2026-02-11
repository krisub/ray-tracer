#pragma once

#include "bbox.h"
#include "ray.h"
#include <cstdint>
#include <vector>

class Geometry;

struct BvhPrimRef {
  Geometry *obj = nullptr;
  BoundingBox bounds;
  glm::dvec3 centroid = glm::dvec3(0.0, 0.0, 0.0);
};

struct BvhNode {
  BoundingBox bounds;
  uint32_t left = 0;
  uint32_t right = 0;
  uint32_t firstPrim = 0;
  uint32_t primCount = 0;
  uint8_t axis = 0;
};

class Bvh {
public:
  void build(const std::vector<Geometry *> &objects, int maxLeafSize = 8,
             int maxDepth = 64);
  bool intersect(ray &r, isect &outIsect) const;

private:
  uint32_t buildRecursive(uint32_t begin, uint32_t end, int depth);

  int leafSize = 8;
  int treeMaxDepth = 64;
  std::vector<BvhPrimRef> prims;
  std::vector<uint32_t> primIndices;
  std::vector<BvhNode> nodes;
};
