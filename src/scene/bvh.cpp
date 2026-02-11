#include "bvh.h"
#include "scene.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

namespace {
constexpr int kNumBins = 16;
constexpr double kTraversalCost = 1.0;
constexpr double kIntersectionCost = 1.0;
constexpr double kAxisEpsilon = 1e-12;

double surfaceArea(const BoundingBox &b) {
  const glm::dvec3 min = b.getMin();
  const glm::dvec3 max = b.getMax();
  const glm::dvec3 d = glm::max(max - min, glm::dvec3(0.0, 0.0, 0.0));
  return 2.0 * (d.x * d.y + d.y * d.z + d.z * d.x);
}

int largestExtentAxis(const BoundingBox &b) {
  const glm::dvec3 d = b.getMax() - b.getMin();
  if (d.x >= d.y && d.x >= d.z) {
    return 0;
  }
  if (d.y >= d.z) {
    return 1;
  }
  return 2;
}

int binForCentroid(double c, double cmin, double cmax) {
  const double extent = cmax - cmin;
  if (extent <= kAxisEpsilon) {
    return 0;
  }
  double t = (c - cmin) / extent;
  t = std::min(0.999999, std::max(0.0, t));
  return static_cast<int>(t * kNumBins);
}
} // namespace

void Bvh::build(const std::vector<Geometry *> &objects, int maxLeafSize,
                int maxDepth) {
  leafSize = std::max(1, maxLeafSize);
  treeMaxDepth = std::max(1, maxDepth);

  prims.clear();
  primIndices.clear();
  nodes.clear();
  prims.reserve(objects.size());

  for (Geometry *obj : objects) {
    if (!obj) {
      continue;
    }
    const BoundingBox &bounds = obj->getBoundingBox();
    const glm::dvec3 min = bounds.getMin();
    const glm::dvec3 max = bounds.getMax();
    const bool validBounds = std::isfinite(min.x) && std::isfinite(min.y) &&
                             std::isfinite(min.z) && std::isfinite(max.x) &&
                             std::isfinite(max.y) && std::isfinite(max.z) &&
                             (min.x <= max.x) && (min.y <= max.y) && (min.z <= max.z);
    if (!validBounds) {
      continue;
    }
    BvhPrimRef prim;
    prim.obj = obj;
    prim.bounds = bounds;
    prim.centroid = 0.5 * (bounds.getMin() + bounds.getMax());
    prims.emplace_back(prim);
  }

  if (prims.empty()) {
    return;
  }

  primIndices.resize(prims.size());
  std::iota(primIndices.begin(), primIndices.end(), 0u);
  nodes.reserve(prims.size() * 2u);
  (void)buildRecursive(0, static_cast<uint32_t>(primIndices.size()), 0);
}

uint32_t Bvh::buildRecursive(uint32_t begin, uint32_t end, int depth) {
  const uint32_t nodeIndex = static_cast<uint32_t>(nodes.size());
  nodes.emplace_back();
  BvhNode &node = nodes.back();

  BoundingBox nodeBounds;
  BoundingBox centroidBounds;
  for (uint32_t i = begin; i < end; ++i) {
    const BvhPrimRef &prim = prims[primIndices[i]];
    nodeBounds.merge(prim.bounds);
    centroidBounds.merge(BoundingBox(prim.centroid, prim.centroid));
  }

  const uint32_t count = end - begin;
  node.bounds = nodeBounds;

  const glm::dvec3 cmin = centroidBounds.getMin();
  const glm::dvec3 cmax = centroidBounds.getMax();
  const glm::dvec3 cext = cmax - cmin;
  const bool noCentroidSpread =
      (cext.x <= kAxisEpsilon && cext.y <= kAxisEpsilon && cext.z <= kAxisEpsilon);
  if (count <= static_cast<uint32_t>(leafSize) || depth >= treeMaxDepth ||
      noCentroidSpread) {
    node.firstPrim = begin;
    node.primCount = count;
    return nodeIndex;
  }

  const double parentArea = surfaceArea(nodeBounds);
  if (parentArea <= kAxisEpsilon) {
    node.firstPrim = begin;
    node.primCount = count;
    return nodeIndex;
  }

  double bestCost = std::numeric_limits<double>::infinity();
  int bestAxis = -1;
  int bestSplit = -1;

  struct Bin {
    BoundingBox bounds;
    uint32_t count = 0;
  };

  for (int axis = 0; axis < 3; ++axis) {
    if (cext[axis] <= kAxisEpsilon) {
      continue;
    }

    std::array<Bin, kNumBins> bins;
    for (uint32_t i = begin; i < end; ++i) {
      const BvhPrimRef &prim = prims[primIndices[i]];
      const int b = binForCentroid(prim.centroid[axis], cmin[axis], cmax[axis]);
      bins[b].count++;
      bins[b].bounds.merge(prim.bounds);
    }

    std::array<double, kNumBins - 1> leftArea;
    std::array<double, kNumBins - 1> rightArea;
    std::array<uint32_t, kNumBins - 1> leftCount;
    std::array<uint32_t, kNumBins - 1> rightCount;

    BoundingBox runningLeft;
    uint32_t runningLeftCount = 0;
    for (int i = 0; i < kNumBins - 1; ++i) {
      runningLeftCount += bins[i].count;
      runningLeft.merge(bins[i].bounds);
      leftCount[i] = runningLeftCount;
      leftArea[i] = surfaceArea(runningLeft);
    }

    BoundingBox runningRight;
    uint32_t runningRightCount = 0;
    for (int i = kNumBins - 1; i > 0; --i) {
      runningRightCount += bins[i].count;
      runningRight.merge(bins[i].bounds);
      rightCount[i - 1] = runningRightCount;
      rightArea[i - 1] = surfaceArea(runningRight);
    }

    for (int i = 0; i < kNumBins - 1; ++i) {
      if (leftCount[i] == 0 || rightCount[i] == 0) {
        continue;
      }
      const double cost = kTraversalCost +
                          (leftArea[i] / parentArea) * leftCount[i] * kIntersectionCost +
                          (rightArea[i] / parentArea) * rightCount[i] * kIntersectionCost;
      if (cost < bestCost) {
        bestCost = cost;
        bestAxis = axis;
        bestSplit = i;
      }
    }
  }

  const double leafCost = static_cast<double>(count) * kIntersectionCost;
  if (bestAxis < 0 || bestCost >= leafCost) {
    node.firstPrim = begin;
    node.primCount = count;
    return nodeIndex;
  }

  auto first = primIndices.begin() + begin;
  auto last = primIndices.begin() + end;
  auto midIt = std::partition(first, last, [&](uint32_t primIndex) {
    const BvhPrimRef &prim = prims[primIndex];
    const int b = binForCentroid(prim.centroid[bestAxis], cmin[bestAxis], cmax[bestAxis]);
    return b <= bestSplit;
  });

  uint32_t mid = static_cast<uint32_t>(midIt - primIndices.begin());
  if (mid == begin || mid == end) {
    const uint32_t median = begin + (count / 2);
    std::nth_element(first, primIndices.begin() + median, last, [&](uint32_t a, uint32_t b) {
      return prims[a].centroid[bestAxis] < prims[b].centroid[bestAxis];
    });
    mid = median;
  }

  if (mid == begin || mid == end) {
    const int axis = largestExtentAxis(centroidBounds);
    const uint32_t median = begin + (count / 2);
    std::nth_element(first, primIndices.begin() + median, last, [&](uint32_t a, uint32_t b) {
      return prims[a].centroid[axis] < prims[b].centroid[axis];
    });
    mid = median;
  }

  if (mid == begin || mid == end) {
    node.firstPrim = begin;
    node.primCount = count;
    return nodeIndex;
  }

  node.axis = static_cast<uint8_t>(bestAxis);
  node.left = buildRecursive(begin, mid, depth + 1);
  node.right = buildRecursive(mid, end, depth + 1);
  node.primCount = 0;
  return nodeIndex;
}

bool Bvh::intersect(ray &r, isect &outIsect) const {
  if (nodes.empty()) {
    return false;
  }

  bool hitAnything = false;
  double bestT = std::numeric_limits<double>::infinity();
  std::vector<uint32_t> stack;
  stack.reserve(64);
  stack.push_back(0u);

  while (!stack.empty()) {
    const uint32_t nodeIndex = stack.back();
    stack.pop_back();
    const BvhNode &node = nodes[nodeIndex];

    double nodeTMin = 0.0;
    double nodeTMax = 0.0;
    if (!node.bounds.intersect(r, nodeTMin, nodeTMax)) {
      continue;
    }
    if (nodeTMin > bestT) {
      continue;
    }

    if (node.primCount > 0) {
      const uint32_t end = node.firstPrim + node.primCount;
      for (uint32_t i = node.firstPrim; i < end; ++i) {
        Geometry *obj = prims[primIndices[i]].obj;
        isect cur;
        if (obj->intersect(r, cur) && cur.getT() > RAY_EPSILON && cur.getT() < bestT) {
          bestT = cur.getT();
          outIsect = cur;
          hitAnything = true;
        }
      }
      continue;
    }

    const BvhNode &leftNode = nodes[node.left];
    const BvhNode &rightNode = nodes[node.right];
    double lmin = 0.0, lmax = 0.0;
    double rmin = 0.0, rmax = 0.0;
    const bool hitLeft = leftNode.bounds.intersect(r, lmin, lmax);
    const bool hitRight = rightNode.bounds.intersect(r, rmin, rmax);

    if (hitLeft && hitRight) {
      if (lmin <= rmin) {
        stack.push_back(node.right);
        stack.push_back(node.left);
      } else {
        stack.push_back(node.left);
        stack.push_back(node.right);
      }
    } else if (hitLeft) {
      stack.push_back(node.left);
    } else if (hitRight) {
      stack.push_back(node.right);
    }
  }

  return hitAnything;
}
