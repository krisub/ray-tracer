#include "bvh.h"
#include "scene.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <limits>
#include <numeric>
#if defined(__AVX2__)
#include <immintrin.h>
#endif

namespace {
constexpr int kNumBins = 16;
constexpr double kTraversalCost = 1.0;
constexpr double kIntersectionCost = 1.0;
constexpr double kAxisEpsilon = 1e-12;
constexpr double kSpatialOverlapReductionMin = 0.25;
constexpr double kSpatialCostImproveMin = 0.03;
constexpr double kSpatialDupFracMax = 0.20;

double surfaceArea(const BoundingBox &b) {
  const glm::dvec3 min = b.getMin();
  const glm::dvec3 max = b.getMax();
  const glm::dvec3 d = glm::max(max - min, glm::dvec3(0.0, 0.0, 0.0));
  return 2.0 * (d.x * d.y + d.y * d.z + d.z * d.x);
}

BoundingBox clippedBounds(const BoundingBox &b, int axis, double split,
                         bool keepLeft) {
  BoundingBox out = b;
  glm::dvec3 mn = out.getMin();
  glm::dvec3 mx = out.getMax();
  if (keepLeft) {
    mx[axis] = std::min(mx[axis], split);
  } else {
    mn[axis] = std::max(mn[axis], split);
  }
  if (mn[axis] > mx[axis]) {
    mn[axis] = mx[axis];
  }
  out.setMin(mn);
  out.setMax(mx);
  return out;
}

double overlapArea(const BoundingBox &a, const BoundingBox &b) {
  const glm::dvec3 mn = glm::max(a.getMin(), b.getMin());
  const glm::dvec3 mx = glm::min(a.getMax(), b.getMax());
  const glm::dvec3 d = mx - mn;
  if (d.x <= 0.0 || d.y <= 0.0 || d.z <= 0.0) {
    return 0.0;
  }
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

bool intersectAabb(const glm::dvec3 &origin, const glm::dvec3 &invDir,
                   const glm::dvec3 &bmin, const glm::dvec3 &bmax,
                   double &tminOut, double &tmaxOut) {
  double tmin = -std::numeric_limits<double>::infinity();
  double tmax = std::numeric_limits<double>::infinity();

  for (int axis = 0; axis < 3; ++axis) {
    double t0 = (bmin[axis] - origin[axis]) * invDir[axis];
    double t1 = (bmax[axis] - origin[axis]) * invDir[axis];
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    tmin = std::max(tmin, t0);
    tmax = std::min(tmax, t1);
    if (tmax < tmin) {
      return false;
    }
  }

  tminOut = tmin;
  tmaxOut = tmax;
  return tmax >= std::max(tmin, RAY_EPSILON);
}

double boundsArea(const BvhNode &n) {
  const glm::dvec3 d = glm::max(n.bounds.getMax() - n.bounds.getMin(),
                                glm::dvec3(0.0, 0.0, 0.0));
  return 2.0 * (d.x * d.y + d.y * d.z + d.z * d.x);
}

inline void sortHitLanesByTNear(uint8_t *lanes, float *tnear, uint8_t count) {
  if (count < 2u) {
    return;
  }
  if (count == 2u) {
    if (tnear[0] > tnear[1]) {
      std::swap(tnear[0], tnear[1]);
      std::swap(lanes[0], lanes[1]);
    }
    return;
  }
  for (uint8_t i = 1; i < count; ++i) {
    const uint8_t lane = lanes[i];
    const float t = tnear[i];
    int j = static_cast<int>(i) - 1;
    while (j >= 0 && tnear[static_cast<size_t>(j)] > t) {
      lanes[static_cast<size_t>(j) + 1] = lanes[static_cast<size_t>(j)];
      tnear[static_cast<size_t>(j) + 1] = tnear[static_cast<size_t>(j)];
      --j;
    }
    lanes[static_cast<size_t>(j) + 1] = lane;
    tnear[static_cast<size_t>(j) + 1] = t;
  }
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

  basePrimCount = static_cast<uint32_t>(prims.size());
  spatialDupCount = 0u;

  std::vector<uint32_t> refs(basePrimCount);
  std::iota(refs.begin(), refs.end(), 0u);
  nodes.reserve(basePrimCount * 2u);
  (void)buildRecursive(refs, 0);
  if (layoutMode == LayoutMode::Wide8) {
    buildWide8();
  } else {
    wide8Nodes.clear();
  }
}

uint32_t Bvh::buildRecursive(const std::vector<uint32_t> &refs, int depth) {
  const uint32_t nodeIndex = static_cast<uint32_t>(nodes.size());
  nodes.emplace_back();
  BvhNode &node = nodes.back();

  BoundingBox nodeBounds;
  BoundingBox centroidBounds;
  for (uint32_t refId : refs) {
    const BvhPrimRef &prim = prims[refId];
    nodeBounds.merge(prim.bounds);
    centroidBounds.merge(BoundingBox(prim.centroid, prim.centroid));
  }

  const uint32_t count = static_cast<uint32_t>(refs.size());
  node.bounds = nodeBounds;

  const glm::dvec3 cmin = centroidBounds.getMin();
  const glm::dvec3 cmax = centroidBounds.getMax();
  const glm::dvec3 cext = cmax - cmin;
  const bool noCentroidSpread =
      (cext.x <= kAxisEpsilon && cext.y <= kAxisEpsilon && cext.z <= kAxisEpsilon);
  if (count <= static_cast<uint32_t>(leafSize) || depth >= treeMaxDepth ||
      noCentroidSpread) {
    node.firstPrim = static_cast<uint32_t>(primIndices.size());
    node.primCount = count;
    primIndices.insert(primIndices.end(), refs.begin(), refs.end());
    return nodeIndex;
  }

  const double parentArea = surfaceArea(nodeBounds);
  if (parentArea <= kAxisEpsilon) {
    node.firstPrim = static_cast<uint32_t>(primIndices.size());
    node.primCount = count;
    primIndices.insert(primIndices.end(), refs.begin(), refs.end());
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
    for (uint32_t refId : refs) {
      const BvhPrimRef &prim = prims[refId];
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
    node.firstPrim = static_cast<uint32_t>(primIndices.size());
    node.primCount = count;
    primIndices.insert(primIndices.end(), refs.begin(), refs.end());
    return nodeIndex;
  }

  std::vector<uint32_t> objectLeft;
  std::vector<uint32_t> objectRight;
  objectLeft.reserve(refs.size());
  objectRight.reserve(refs.size());
  for (uint32_t refId : refs) {
    const BvhPrimRef &prim = prims[refId];
    const int b = binForCentroid(prim.centroid[bestAxis], cmin[bestAxis], cmax[bestAxis]);
    if (b <= bestSplit) {
      objectLeft.push_back(refId);
    } else {
      objectRight.push_back(refId);
    }
  }
  if (objectLeft.empty() || objectRight.empty()) {
    std::vector<uint32_t> sorted = refs;
    const int axis = largestExtentAxis(centroidBounds);
    std::sort(sorted.begin(), sorted.end(), [&](uint32_t a, uint32_t b) {
      return prims[a].centroid[axis] < prims[b].centroid[axis];
    });
    const size_t mid = sorted.size() / 2u;
    objectLeft.assign(sorted.begin(), sorted.begin() + mid);
    objectRight.assign(sorted.begin() + mid, sorted.end());
  }
  if (objectLeft.empty() || objectRight.empty()) {
    node.firstPrim = static_cast<uint32_t>(primIndices.size());
    node.primCount = count;
    primIndices.insert(primIndices.end(), refs.begin(), refs.end());
    return nodeIndex;
  }

  BoundingBox objectLeftBounds;
  BoundingBox objectRightBounds;
  for (uint32_t refId : objectLeft) {
    objectLeftBounds.merge(prims[refId].bounds);
  }
  for (uint32_t refId : objectRight) {
    objectRightBounds.merge(prims[refId].bounds);
  }
  const double objectOverlap = overlapArea(objectLeftBounds, objectRightBounds);
  const double maxDupAllowed = static_cast<double>(count) * kSpatialDupFracMax;
  bool useSpatial = false;
  int spatialAxis = -1;
  double spatialPlane = 0.0;
  double spatialCost = std::numeric_limits<double>::infinity();
  double spatialOverlap = objectOverlap;

  // keep track of best spatial split
  for (int axis = 0; axis < 3; ++axis) {
    const double minA = nodeBounds.getMin()[axis];
    const double maxA = nodeBounds.getMax()[axis];
    const double extent = maxA - minA;
    if (extent <= kAxisEpsilon) {
      continue;
    }
    for (int s = 1; s < kNumBins; ++s) {
      const double plane = minA + extent * (static_cast<double>(s) / kNumBins);
      BoundingBox lb, rb;
      uint32_t lc = 0u, rc = 0u, dups = 0u;
      for (uint32_t refId : refs) {
        const BoundingBox &b = prims[refId].bounds;
        if (b.getMax()[axis] <= plane) {
          lb.merge(b);
          ++lc;
        } else if (b.getMin()[axis] >= plane) {
          rb.merge(b);
          ++rc;
        } else {
          BoundingBox bl = clippedBounds(b, axis, plane, true);
          BoundingBox br = clippedBounds(b, axis, plane, false);
          lb.merge(bl);
          rb.merge(br);
          ++lc;
          ++rc;
          ++dups;
        }
      }
      if (lc == 0u || rc == 0u) {
        continue;
      }
      if (static_cast<double>(dups) > maxDupAllowed) {
        continue;
      }
      const double la = surfaceArea(lb);
      const double ra = surfaceArea(rb);
      const double cost = kTraversalCost +
                          (la / parentArea) * static_cast<double>(lc) * kIntersectionCost +
                          (ra / parentArea) * static_cast<double>(rc) * kIntersectionCost;
      const double ov = overlapArea(lb, rb);
      if (cost < spatialCost) {
        spatialCost = cost;
        spatialOverlap = ov;
        spatialAxis = axis;
        spatialPlane = plane;
      }
    }
  }

  if (spatialAxis >= 0) {
    const bool overlapReducedEnough =
        spatialOverlap <= objectOverlap * (1.0 - kSpatialOverlapReductionMin);
    const bool costImprovedEnough =
        spatialCost <= bestCost * (1.0 - kSpatialCostImproveMin);
    useSpatial = overlapReducedEnough && costImprovedEnough &&
                 (spatialDupCount < basePrimCount);
  }

  std::vector<uint32_t> leftRefs;
  std::vector<uint32_t> rightRefs;
  if (useSpatial) {
    leftRefs.reserve(refs.size());
    rightRefs.reserve(refs.size());
    for (uint32_t refId : refs) {
      const BoundingBox &b = prims[refId].bounds;
      if (b.getMax()[spatialAxis] <= spatialPlane) {
        leftRefs.push_back(refId);
      } else if (b.getMin()[spatialAxis] >= spatialPlane) {
        rightRefs.push_back(refId);
      } else if (spatialDupCount < basePrimCount) {
        BvhPrimRef leftRef = prims[refId];
        leftRef.bounds = clippedBounds(b, spatialAxis, spatialPlane, true);
        leftRef.centroid = 0.5 * (leftRef.bounds.getMin() + leftRef.bounds.getMax());
        prims.push_back(leftRef);
        leftRefs.push_back(static_cast<uint32_t>(prims.size() - 1u));

        BvhPrimRef rightRef = prims[refId];
        rightRef.bounds = clippedBounds(b, spatialAxis, spatialPlane, false);
        rightRef.centroid = 0.5 * (rightRef.bounds.getMin() + rightRef.bounds.getMax());
        prims.push_back(rightRef);
        rightRefs.push_back(static_cast<uint32_t>(prims.size() - 1u));
        ++spatialDupCount;
      } else {
        if (prims[refId].centroid[spatialAxis] <= spatialPlane) {
          leftRefs.push_back(refId);
        } else {
          rightRefs.push_back(refId);
        }
      }
    }
    if (leftRefs.empty() || rightRefs.empty()) {
      useSpatial = false;
    }
  }
  if (!useSpatial) {
    leftRefs.swap(objectLeft);
    rightRefs.swap(objectRight);
  }

  if (leftRefs.empty() || rightRefs.empty()) {
    node.firstPrim = static_cast<uint32_t>(primIndices.size());
    node.primCount = count;
    primIndices.insert(primIndices.end(), refs.begin(), refs.end());
    return nodeIndex;
  }

  node.axis = static_cast<uint8_t>(useSpatial ? spatialAxis : bestAxis);
  node.left = buildRecursive(leftRefs, depth + 1);
  node.right = buildRecursive(rightRefs, depth + 1);
  node.primCount = 0;
  return nodeIndex;
}

bool Bvh::isBinaryLeaf(uint32_t idx) const { return nodes[idx].primCount > 0; }

uint32_t Bvh::buildWide8FromBinary(uint32_t binaryRoot) {
  const uint32_t wideIndex = static_cast<uint32_t>(wide8Nodes.size());
  wide8Nodes.emplace_back();
  Bvh8Node &wn = wide8Nodes.back();

  if (isBinaryLeaf(binaryRoot)) {
    const BvhNode &leaf = nodes[binaryRoot];
    wn.childCount = 1;
    wn.leafMask = 0x1u;
    wn.minX[0] = static_cast<float>(leaf.bounds.getMin().x);
    wn.minY[0] = static_cast<float>(leaf.bounds.getMin().y);
    wn.minZ[0] = static_cast<float>(leaf.bounds.getMin().z);
    wn.maxX[0] = static_cast<float>(leaf.bounds.getMax().x);
    wn.maxY[0] = static_cast<float>(leaf.bounds.getMax().y);
    wn.maxZ[0] = static_cast<float>(leaf.bounds.getMax().z);
    wn.firstPrim[0] = leaf.firstPrim;
    wn.primCount[0] = leaf.primCount;
    return wideIndex;
  }

  std::vector<uint32_t> children;
  children.reserve(8);
  const BvhNode &root = nodes[binaryRoot];
  children.push_back(root.left);
  children.push_back(root.right);

  while (children.size() < 8u) {
    int splitPos = -1;
    double bestSplitScore = -1.0;
    for (int i = 0; i < static_cast<int>(children.size()); ++i) {
      const uint32_t c = children[static_cast<size_t>(i)];
      if (isBinaryLeaf(c)) {
        continue;
      }
      const double score = boundsArea(nodes[c]);
      if (score > bestSplitScore) {
        bestSplitScore = score;
        splitPos = i;
      }
    }
    if (splitPos < 0) {
      break;
    }
    const uint32_t splitNode = children[static_cast<size_t>(splitPos)];
    const BvhNode &sn = nodes[splitNode];
    children[static_cast<size_t>(splitPos)] = sn.left;
    children.insert(children.begin() + splitPos + 1, sn.right);
  }

  wn.childCount = static_cast<uint8_t>(children.size());
  for (uint8_t lane = 0; lane < wn.childCount; ++lane) {
    const uint32_t childRoot = children[lane];
    const BvhNode &bn = nodes[childRoot];
    wn.minX[lane] = static_cast<float>(bn.bounds.getMin().x);
    wn.minY[lane] = static_cast<float>(bn.bounds.getMin().y);
    wn.minZ[lane] = static_cast<float>(bn.bounds.getMin().z);
    wn.maxX[lane] = static_cast<float>(bn.bounds.getMax().x);
    wn.maxY[lane] = static_cast<float>(bn.bounds.getMax().y);
    wn.maxZ[lane] = static_cast<float>(bn.bounds.getMax().z);

    if (isBinaryLeaf(childRoot)) {
      wn.leafMask |= static_cast<uint8_t>(1u << lane);
      wn.firstPrim[lane] = bn.firstPrim;
      wn.primCount[lane] = bn.primCount;
    } else {
      wn.childNode[lane] = buildWide8FromBinary(childRoot);
    }
  }
  return wideIndex;
}

void Bvh::buildWide8() {
  wide8Nodes.clear();
  if (nodes.empty()) {
    return;
  }
  wide8Nodes.reserve(nodes.size());
  (void)buildWide8FromBinary(0u);
}

bool Bvh::intersectWide8(ray &r, isect &outIsect) const {
  if (wide8Nodes.empty()) {
    return false;
  }

  struct Task {
    uint8_t kind = 0; // 0=node, 1=leaf
    uint32_t node = 0;
    uint32_t first = 0;
    uint32_t count = 0;
    float tmin = 0.0f;
  };

  std::array<Task, 128> stack{};
  uint32_t top = 0u;

  bool hitAnything = false;
  double bestT = std::numeric_limits<double>::infinity();
  stack[top++] = Task{0u, 0u, 0u, 0u, -FLT_MAX};

  // Precompute origin and inverse direction once per ray.
  const glm::dvec3 origin = r.getPosition();
  const glm::dvec3 dir = r.getDirection();
  const glm::dvec3 invDir(
      std::abs(dir.x) > 0.0 ? 1.0 / dir.x : std::copysign(1.0e308, dir.x),
      std::abs(dir.y) > 0.0 ? 1.0 / dir.y : std::copysign(1.0e308, dir.y),
      std::abs(dir.z) > 0.0 ? 1.0 / dir.z : std::copysign(1.0e308, dir.z));

#if defined(__AVX2__)
  const bool canUseAvx = (std::abs(dir.x) > kAxisEpsilon &&
                          std::abs(dir.y) > kAxisEpsilon &&
                          std::abs(dir.z) > kAxisEpsilon);
  const __m256 ox = _mm256_set1_ps(static_cast<float>(origin.x));
  const __m256 oy = _mm256_set1_ps(static_cast<float>(origin.y));
  const __m256 oz = _mm256_set1_ps(static_cast<float>(origin.z));
  const __m256 invx = _mm256_set1_ps(static_cast<float>(1.0 / dir.x));
  const __m256 invy = _mm256_set1_ps(static_cast<float>(1.0 / dir.y));
  const __m256 invz = _mm256_set1_ps(static_cast<float>(1.0 / dir.z));
  const __m256 eps = _mm256_set1_ps(static_cast<float>(RAY_EPSILON));
#endif

  while (top > 0u) {
    const Task task = stack[--top];
    if (task.tmin > static_cast<float>(bestT)) {
      continue;
    }

    if (task.kind == 1u) {
      const uint32_t end = task.first + task.count;
      for (uint32_t i = task.first; i < end; ++i) {
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

    const Bvh8Node &wn = wide8Nodes[task.node];
    if (wn.childCount == 0u) {
      continue;
    }

    float childTMin[8] = {FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX,
                          FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX};
    uint8_t hitMask = 0u;
    bool didAvxTest = false;

#if defined(__AVX2__)
    if (canUseAvx) {
      didAvxTest = true;
      const __m256 bminx = _mm256_loadu_ps(wn.minX);
      const __m256 bminy = _mm256_loadu_ps(wn.minY);
      const __m256 bminz = _mm256_loadu_ps(wn.minZ);
      const __m256 bmaxx = _mm256_loadu_ps(wn.maxX);
      const __m256 bmaxy = _mm256_loadu_ps(wn.maxY);
      const __m256 bmaxz = _mm256_loadu_ps(wn.maxZ);

      const __m256 tx1 = _mm256_mul_ps(_mm256_sub_ps(bminx, ox), invx);
      const __m256 tx2 = _mm256_mul_ps(_mm256_sub_ps(bmaxx, ox), invx);
      const __m256 ty1 = _mm256_mul_ps(_mm256_sub_ps(bminy, oy), invy);
      const __m256 ty2 = _mm256_mul_ps(_mm256_sub_ps(bmaxy, oy), invy);
      const __m256 tz1 = _mm256_mul_ps(_mm256_sub_ps(bminz, oz), invz);
      const __m256 tz2 = _mm256_mul_ps(_mm256_sub_ps(bmaxz, oz), invz);

      const __m256 txmin = _mm256_min_ps(tx1, tx2);
      const __m256 txmax = _mm256_max_ps(tx1, tx2);
      const __m256 tymin = _mm256_min_ps(ty1, ty2);
      const __m256 tymax = _mm256_max_ps(ty1, ty2);
      const __m256 tzmin = _mm256_min_ps(tz1, tz2);
      const __m256 tzmax = _mm256_max_ps(tz1, tz2);

      const __m256 tmin = _mm256_max_ps(_mm256_max_ps(txmin, tymin), tzmin);
      const __m256 tmax = _mm256_min_ps(_mm256_min_ps(txmax, tymax), tzmax);
      const __m256 best = _mm256_set1_ps(static_cast<float>(bestT));
      const __m256 validA = _mm256_cmp_ps(tmax, _mm256_max_ps(tmin, eps), _CMP_GE_OQ);
      const __m256 validB = _mm256_cmp_ps(tmin, best, _CMP_LE_OQ);
      const __m256 valid = _mm256_and_ps(validA, validB);
      uint32_t mask = static_cast<uint32_t>(_mm256_movemask_ps(valid));

      alignas(32) float tmins8[8];
      _mm256_store_ps(tmins8, tmin);
      for (uint8_t lane = 0; lane < wn.childCount; ++lane) {
        childTMin[lane] = tmins8[lane];
      }
      hitMask = static_cast<uint8_t>(mask & ((1u << wn.childCount) - 1u));
    }
#endif

    if (!didAvxTest) {
      for (uint8_t lane = 0; lane < wn.childCount; ++lane) {
        const glm::dvec3 cmin(wn.minX[lane], wn.minY[lane], wn.minZ[lane]);
        const glm::dvec3 cmax(wn.maxX[lane], wn.maxY[lane], wn.maxZ[lane]);
        double tmin = 0.0, tmax = 0.0;
        if (intersectAabb(origin, invDir, cmin, cmax, tmin, tmax) && tmin <= bestT) {
          hitMask |= static_cast<uint8_t>(1u << lane);
          childTMin[lane] = static_cast<float>(tmin);
        }
      }
    }

    uint8_t hitLanes[8];
    float hitTMin[8];
    uint8_t hitCount = 0u;
    for (uint8_t lane = 0; lane < wn.childCount; ++lane) {
      if ((hitMask & static_cast<uint8_t>(1u << lane)) == 0u) {
        continue;
      }
      hitLanes[hitCount] = lane;
      hitTMin[hitCount] = childTMin[lane];
      ++hitCount;
    }
    sortHitLanesByTNear(hitLanes, hitTMin, hitCount);

    for (int i = static_cast<int>(hitCount) - 1; i >= 0; --i) {
      const uint8_t lane = hitLanes[static_cast<size_t>(i)];
      const float tmin = hitTMin[static_cast<size_t>(i)];
      if (tmin > static_cast<float>(bestT)) {
        continue;
      }
      const bool isLeaf = (wn.leafMask & static_cast<uint8_t>(1u << lane)) != 0u;
      if (isLeaf) {
        stack[top++] = Task{1u, 0u, wn.firstPrim[lane], wn.primCount[lane], tmin};
      } else {
        stack[top++] = Task{0u, wn.childNode[lane], 0u, 0u, tmin};
      }
    }
  }

  return hitAnything;
}

bool Bvh::intersect(ray &r, isect &outIsect) const {
  if (layoutMode == LayoutMode::Wide8 && !wide8Nodes.empty()) {
    return intersectWide8(r, outIsect);
  }

  if (nodes.empty()) {
    return false;
  }

  const glm::dvec3 origin = r.getPosition();
  const glm::dvec3 dir = r.getDirection();
  const glm::dvec3 invDir(
      std::abs(dir.x) > 0.0 ? 1.0 / dir.x : std::copysign(1.0e308, dir.x),
      std::abs(dir.y) > 0.0 ? 1.0 / dir.y : std::copysign(1.0e308, dir.y),
      std::abs(dir.z) > 0.0 ? 1.0 / dir.z : std::copysign(1.0e308, dir.z));

  bool hitAnything = false;
  double bestT = std::numeric_limits<double>::infinity();
  std::array<uint32_t, 128> stack{};
  uint32_t top = 0u;
  stack[top++] = 0u;

  while (top > 0u) {
    const uint32_t nodeIndex = stack[--top];
    const BvhNode &node = nodes[nodeIndex];

    if (node.primCount > 0) {
      // leaf: test primitives
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

    // interior: test children bounds once, cull against bestT, push in order
    double lmin = 0.0, lmax = 0.0;
    double rmin = 0.0, rmax = 0.0;
    bool hitLeft = nodes[node.left].bounds.intersect(origin, invDir, lmin, lmax);
    bool hitRight = nodes[node.right].bounds.intersect(origin, invDir, rmin, rmax);

    if (hitLeft && lmin > bestT) hitLeft = false;
    if (hitRight && rmin > bestT) hitRight = false;

    if (hitLeft && hitRight) {
      if (lmin <= rmin) {
        stack[top++] = node.right;
        stack[top++] = node.left;
      } else {
        stack[top++] = node.left;
        stack[top++] = node.right;
      }
    } else if (hitLeft) {
      stack[top++] = node.left;
    } else if (hitRight) {
      stack[top++] = node.right;
    }
  }

  return hitAnything;
}
