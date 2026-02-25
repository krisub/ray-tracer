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
  enum class LayoutMode { Binary, Wide8 };

  void build(const std::vector<Geometry *> &objects, int maxLeafSize = 8,
             int maxDepth = 64);
  bool intersect(ray &r, isect &outIsect) const;
  void setLayoutMode(LayoutMode mode) { layoutMode = mode; }
  LayoutMode getLayoutMode() const { return layoutMode; }

private:
  struct alignas(64) Bvh8Node {
    float minX[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    float minY[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    float minZ[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    float maxX[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    float maxY[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    float maxZ[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t childNode[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t firstPrim[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t primCount[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t childCount = 0;
    uint8_t leafMask = 0; // bit i = lane i is leaf payload
  };
  static_assert((sizeof(Bvh8Node) % 64) == 0,
                "Bvh8Node should stay cache-line aligned");

  uint32_t buildRecursive(const std::vector<uint32_t> &refs, int depth);
  bool isBinaryLeaf(uint32_t idx) const;
  uint32_t buildWide8FromBinary(uint32_t binaryRoot);
  void buildWide8();
  bool intersectWide8(ray &r, isect &outIsect) const;

  int leafSize = 8;
  int treeMaxDepth = 64;
  uint32_t basePrimCount = 0;
  uint32_t spatialDupCount = 0;
  std::vector<BvhPrimRef> prims;
  std::vector<uint32_t> primIndices;
  std::vector<BvhNode> nodes;
  std::vector<Bvh8Node> wide8Nodes;
  LayoutMode layoutMode = LayoutMode::Binary;
};
