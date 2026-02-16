#include "trimesh.h"
#include "../ui/TraceUI.h"
#include <algorithm>
#include <array>
#include <assert.h>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <float.h>
#include <limits>
#include <numeric>
#include <string.h>
#if defined(__AVX2__)
#include <immintrin.h>
#endif
extern TraceUI *traceUI;

using namespace std;
namespace {
constexpr int kFaceBvhBins = 16;
constexpr double kFaceBvhTraversalCost = 1.0;
constexpr double kFaceBvhIntersectCost = 1.0;
constexpr double kFaceBvhAxisEps = 1e-12;

double bboxSurfaceArea(const BoundingBox &b) {
  const glm::dvec3 mn = b.getMin();
  const glm::dvec3 mx = b.getMax();
  const glm::dvec3 d = glm::max(mx - mn, glm::dvec3(0.0, 0.0, 0.0));
  return 2.0 * (d.x * d.y + d.y * d.z + d.z * d.x);
}

int centroidBin(double c, double cmin, double cmax) {
  const double e = cmax - cmin;
  if (e <= kFaceBvhAxisEps) {
    return 0;
  }
  double t = (c - cmin) / e;
  t = std::min(0.999999, std::max(0.0, t));
  return static_cast<int>(t * kFaceBvhBins);
}

int maxExtentAxis(const BoundingBox &b) {
  const glm::dvec3 d = b.getMax() - b.getMin();
  if (d.x >= d.y && d.x >= d.z)
    return 0;
  if (d.y >= d.z)
    return 1;
  return 2;
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
} // namespace

Trimesh::~Trimesh() { // has a list of triangles (TrimeshFaces)
  for (auto f : faces)
    delete f;
}

// must add vertices, normals, and materials IN ORDER
void Trimesh::addVertex(const glm::dvec3 &v) { vertices.emplace_back(v); }

void Trimesh::addNormal(const glm::dvec3 &n) { normals.emplace_back(n); }

void Trimesh::addColor(const glm::dvec3 &c) { vertColors.emplace_back(c); }

void Trimesh::addUV(const glm::dvec2 &uv) { uvCoords.emplace_back(uv); }

// Returns false if the vertices a,b,c don't all exist
bool Trimesh::addFace(int a, int b, int c) {
  int vcnt = vertices.size();

  if (a >= vcnt || b >= vcnt || c >= vcnt)
    return false;

  TrimeshFace *newFace = new TrimeshFace(this, a, b, c);
  if (!newFace->degen)
    faces.push_back(newFace);
  else
    delete newFace;
  faceBvhDirty = true;

  // Don't add faces to the scene's object list so we can cull by bounding
  // box
  return true;
}

// Check to make sure that if we have per-vertex materials or normals
// they are the right number.
const char *Trimesh::doubleCheck() {
  if (!vertColors.empty() && vertColors.size() != vertices.size())
    return "Bad Trimesh: Wrong number of vertex colors.";
  if (!uvCoords.empty() && uvCoords.size() != vertices.size())
    return "Bad Trimesh: Wrong number of UV coordinates.";
  if (!normals.empty() && normals.size() != vertices.size())
    return "Bad Trimesh: Wrong number of normals.";

  return 0;
}

// shoots a ray at all triangles in the mesh and keeps whatever pt is closest
// returns true if there was a hit false otherwise
bool Trimesh::intersectLocal(ray &r, isect &i) const {
  rebuildFaceBvhIfNeeded();

  if (faceBvhNodes.empty()) {
    i.setT(1000.0);
    return false;
  }

  const bool useWide8 =
      traceUI && traceUI->blasAccelMode() == TraceUI::AccelMode::Wide8;
  if (useWide8 && !faceBvh8Nodes.empty()) {
    return intersectWide8(r, i);
  }
  return intersectBinary(r, i);
}

bool Trimesh::intersectBinary(ray &r, isect &i) const {
  const glm::dvec3 origin = r.getPosition();
  const glm::dvec3 dir = r.getDirection();
  const glm::dvec3 invDir(
      std::abs(dir.x) > 0.0 ? 1.0 / dir.x : std::copysign(1.0e308, dir.x),
      std::abs(dir.y) > 0.0 ? 1.0 / dir.y : std::copysign(1.0e308, dir.y),
      std::abs(dir.z) > 0.0 ? 1.0 / dir.z : std::copysign(1.0e308, dir.z));

  std::array<uint32_t, 128> stack{};
  uint32_t top = 0u;
  stack[top++] = 0u;
  bool have_one = false;
  double bestT = std::numeric_limits<double>::infinity();

  while (top > 0u) {
    const uint32_t nodeIndex = stack[--top];
    const FaceBvhNode &node = faceBvhNodes[nodeIndex];

    if (node.count > 0) {
      // Leaf: test faces.
      const uint32_t end = node.first + node.count;
      for (uint32_t idx = node.first; idx < end; ++idx) {
        TrimeshFace *face = faces[faceIndices[idx]];
        isect cur;
        if (face->intersectLocal(r, cur, bestT)) {
          if (cur.getT() < bestT) {
            i = cur;
            bestT = cur.getT();
            have_one = true;
          }
        }
      }
      continue;
    }

    // test children bounds once, cull against bestT, push in order.
    double lmin = 0.0, lmax = 0.0;
    double rmin = 0.0, rmax = 0.0;
    bool hitL = faceBvhNodes[node.left].bounds.intersect(origin, invDir, lmin, lmax);
    bool hitR = faceBvhNodes[node.right].bounds.intersect(origin, invDir, rmin, rmax);

    if (hitL && lmin > bestT) hitL = false;
    if (hitR && rmin > bestT) hitR = false;

    if (hitL && hitR) {
      if (lmin <= rmin) {
        stack[top++] = node.right;
        stack[top++] = node.left;
      } else {
        stack[top++] = node.left;
        stack[top++] = node.right;
      }
    } else if (hitL) {
      stack[top++] = node.left;
    } else if (hitR) {
      stack[top++] = node.right;
    }
  }

  if (!have_one)
    i.setT(1000.0);
  return have_one;
}

bool Trimesh::isFaceBinaryLeaf(uint32_t idx) const {
  return faceBvhNodes[idx].count > 0;
}

uint32_t Trimesh::buildWide8FromBinary(uint32_t binaryRoot) const {
  const uint32_t wideIndex = static_cast<uint32_t>(faceBvh8Nodes.size());
  faceBvh8Nodes.emplace_back();

  const FaceBvhNode &root = faceBvhNodes[binaryRoot];
  if (isFaceBinaryLeaf(binaryRoot)) {
    faceBvh8Nodes[wideIndex].childCount = 1;
    faceBvh8Nodes[wideIndex].leafMask = 0x1u;
    faceBvh8Nodes[wideIndex].minX[0] = static_cast<float>(root.bounds.getMin().x);
    faceBvh8Nodes[wideIndex].minY[0] = static_cast<float>(root.bounds.getMin().y);
    faceBvh8Nodes[wideIndex].minZ[0] = static_cast<float>(root.bounds.getMin().z);
    faceBvh8Nodes[wideIndex].maxX[0] = static_cast<float>(root.bounds.getMax().x);
    faceBvh8Nodes[wideIndex].maxY[0] = static_cast<float>(root.bounds.getMax().y);
    faceBvh8Nodes[wideIndex].maxZ[0] = static_cast<float>(root.bounds.getMax().z);
    faceBvh8Nodes[wideIndex].first[0] = root.first;
    faceBvh8Nodes[wideIndex].count[0] = root.count;
    return wideIndex;
  }

  std::vector<uint32_t> children;
  children.reserve(8);
  children.push_back(root.left);
  children.push_back(root.right);

  while (children.size() < 8u) {
    int splitPos = -1;
    double bestArea = -1.0;
    for (int i = 0; i < static_cast<int>(children.size()); ++i) {
      const uint32_t c = children[static_cast<size_t>(i)];
      if (isFaceBinaryLeaf(c)) {
        continue;
      }
      const FaceBvhNode &cn = faceBvhNodes[c];
      const glm::dvec3 d = glm::max(cn.bounds.getMax() - cn.bounds.getMin(),
                                    glm::dvec3(0.0, 0.0, 0.0));
      const double area = 2.0 * (d.x * d.y + d.y * d.z + d.z * d.x);
      if (area > bestArea) {
        bestArea = area;
        splitPos = i;
      }
    }
    if (splitPos < 0) {
      break;
    }
    const uint32_t splitNode = children[static_cast<size_t>(splitPos)];
    const FaceBvhNode &sn = faceBvhNodes[splitNode];
    children[static_cast<size_t>(splitPos)] = sn.left;
    children.insert(children.begin() + splitPos + 1, sn.right);
  }

  faceBvh8Nodes[wideIndex].childCount = static_cast<uint8_t>(children.size());
  for (uint8_t lane = 0; lane < faceBvh8Nodes[wideIndex].childCount; ++lane) {
    const uint32_t childRoot = children[lane];
    const FaceBvhNode &bn = faceBvhNodes[childRoot];
    faceBvh8Nodes[wideIndex].minX[lane] = static_cast<float>(bn.bounds.getMin().x);
    faceBvh8Nodes[wideIndex].minY[lane] = static_cast<float>(bn.bounds.getMin().y);
    faceBvh8Nodes[wideIndex].minZ[lane] = static_cast<float>(bn.bounds.getMin().z);
    faceBvh8Nodes[wideIndex].maxX[lane] = static_cast<float>(bn.bounds.getMax().x);
    faceBvh8Nodes[wideIndex].maxY[lane] = static_cast<float>(bn.bounds.getMax().y);
    faceBvh8Nodes[wideIndex].maxZ[lane] = static_cast<float>(bn.bounds.getMax().z);

    if (isFaceBinaryLeaf(childRoot)) {
      faceBvh8Nodes[wideIndex].leafMask |= static_cast<uint8_t>(1u << lane);
      faceBvh8Nodes[wideIndex].first[lane] = bn.first;
      faceBvh8Nodes[wideIndex].count[lane] = bn.count;
    } else {
      faceBvh8Nodes[wideIndex].childNode[lane] = buildWide8FromBinary(childRoot);
    }
  }

  return wideIndex;
}

void Trimesh::buildWide8() const {
  faceBvh8Nodes.clear();
  if (faceBvhNodes.empty()) {
    return;
  }
  faceBvh8Nodes.reserve(faceBvhNodes.size());
  (void)buildWide8FromBinary(0u);
}

bool Trimesh::intersectWide8(ray &r, isect &i) const {
  if (faceBvh8Nodes.empty()) {
    return false;
  }

  struct Task {
    uint8_t kind = 0; // 0=node, 1=leaf range
    uint32_t node = 0;
    uint32_t first = 0;
    uint32_t count = 0;
    float tmin = 0.0f;
  };

  std::array<Task, 128> stack{};
  uint32_t top = 0u;

  bool haveOne = false;
  double bestT = std::numeric_limits<double>::infinity();
  stack[top++] = Task{0u, 0u, 0u, 0u, -FLT_MAX};

  // precompute origin and inverse direction once per ray.
  const glm::dvec3 origin = r.getPosition();
  const glm::dvec3 dir = r.getDirection();
  const glm::dvec3 invDir(
      std::abs(dir.x) > 0.0 ? 1.0 / dir.x : std::copysign(1.0e308, dir.x),
      std::abs(dir.y) > 0.0 ? 1.0 / dir.y : std::copysign(1.0e308, dir.y),
      std::abs(dir.z) > 0.0 ? 1.0 / dir.z : std::copysign(1.0e308, dir.z));

#if defined(__AVX2__)
  const bool canUseAvx = (std::abs(dir.x) > kFaceBvhAxisEps &&
                          std::abs(dir.y) > kFaceBvhAxisEps &&
                          std::abs(dir.z) > kFaceBvhAxisEps);
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
      for (uint32_t idx = task.first; idx < end; ++idx) {
        TrimeshFace *face = faces[faceIndices[idx]];
        isect cur;
        if (face->intersectLocal(r, cur, bestT) && cur.getT() < bestT) {
          i = cur;
          bestT = cur.getT();
          haveOne = true;
        }
      }
      continue;
    }

    const FaceBvh8Node &wn = faceBvh8Nodes[task.node];
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

    for (int k = static_cast<int>(hitCount) - 1; k >= 0; --k) {
      const uint8_t lane = hitLanes[static_cast<size_t>(k)];
      const float tmin = hitTMin[static_cast<size_t>(k)];
      if (tmin > static_cast<float>(bestT)) {
        continue;
      }
      const bool isLeaf = (wn.leafMask & static_cast<uint8_t>(1u << lane)) != 0u;
      if (isLeaf) {
        stack[top++] = Task{1u, 0u, wn.first[lane], wn.count[lane], tmin};
      } else {
        stack[top++] = Task{0u, wn.childNode[lane], 0u, 0u, tmin};
      }
    }
  }

  if (!haveOne) {
    i.setT(1000.0);
  }
  return haveOne;
}

void Trimesh::rebuildFaceBvhIfNeeded() const {
  if (!faceBvhDirty) {
    return;
  }

  std::lock_guard<std::mutex> guard(faceBvhBuildMutex);
  if (!faceBvhDirty) {
    return;
  }

  faceIndices.clear();
  faceBvhNodes.clear();
  faceBvh8Nodes.clear();

  const uint32_t nFaces = static_cast<uint32_t>(faces.size());
  if (nFaces == 0) {
    faceBvhDirty = false;
    return;
  }

  faceIndices.resize(nFaces);
  std::iota(faceIndices.begin(), faceIndices.end(), 0u);
  faceBvhNodes.reserve(nFaces * 2u);
  (void)buildFaceBvhRecursive(0u, nFaces);
  if (traceUI && traceUI->blasAccelMode() == TraceUI::AccelMode::Wide8) {
    buildWide8();
  }
  faceBvhDirty = false;
}

uint32_t Trimesh::buildFaceBvhRecursive(uint32_t begin, uint32_t end) const {
  const uint32_t nodeIndex = static_cast<uint32_t>(faceBvhNodes.size());
  faceBvhNodes.emplace_back();
  FaceBvhNode &node = faceBvhNodes.back();

  BoundingBox nodeBounds;
  BoundingBox centroidBounds;
  for (uint32_t i = begin; i < end; ++i) {
    const BoundingBox &b = faces[faceIndices[i]]->getBoundingBox();
    nodeBounds.merge(b);
    const glm::dvec3 c = 0.5 * (b.getMin() + b.getMax());
    centroidBounds.merge(BoundingBox(c, c));
  }
  node.bounds = nodeBounds;

  const uint32_t count = end - begin;
  const glm::dvec3 cmin = centroidBounds.getMin();
  const glm::dvec3 cmax = centroidBounds.getMax();
  const glm::dvec3 cext = cmax - cmin;
  const bool degenerateCentroids =
      (cext.x <= kFaceBvhAxisEps && cext.y <= kFaceBvhAxisEps &&
       cext.z <= kFaceBvhAxisEps);

  constexpr uint32_t kLeafSize = 8u;
  if (count <= kLeafSize || degenerateCentroids) {
    node.first = begin;
    node.count = count;
    return nodeIndex;
  }

  const double parentArea = bboxSurfaceArea(nodeBounds);
  if (parentArea <= kFaceBvhAxisEps) {
    node.first = begin;
    node.count = count;
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
    if (cext[axis] <= kFaceBvhAxisEps) {
      continue;
    }
    std::array<Bin, kFaceBvhBins> bins;
    for (uint32_t i = begin; i < end; ++i) {
      const BoundingBox &b = faces[faceIndices[i]]->getBoundingBox();
      const glm::dvec3 c = 0.5 * (b.getMin() + b.getMax());
      const int bin = centroidBin(c[axis], cmin[axis], cmax[axis]);
      bins[bin].count++;
      bins[bin].bounds.merge(b);
    }

    std::array<double, kFaceBvhBins - 1> leftArea;
    std::array<double, kFaceBvhBins - 1> rightArea;
    std::array<uint32_t, kFaceBvhBins - 1> leftCount;
    std::array<uint32_t, kFaceBvhBins - 1> rightCount;

    BoundingBox leftBounds;
    uint32_t lcount = 0;
    for (int i = 0; i < kFaceBvhBins - 1; ++i) {
      lcount += bins[i].count;
      leftBounds.merge(bins[i].bounds);
      leftCount[i] = lcount;
      leftArea[i] = bboxSurfaceArea(leftBounds);
    }

    BoundingBox rightBounds;
    uint32_t rcount = 0;
    for (int i = kFaceBvhBins - 1; i > 0; --i) {
      rcount += bins[i].count;
      rightBounds.merge(bins[i].bounds);
      rightCount[i - 1] = rcount;
      rightArea[i - 1] = bboxSurfaceArea(rightBounds);
    }

    for (int i = 0; i < kFaceBvhBins - 1; ++i) {
      if (leftCount[i] == 0 || rightCount[i] == 0)
        continue;
      const double cost =
          kFaceBvhTraversalCost +
          (leftArea[i] / parentArea) * leftCount[i] * kFaceBvhIntersectCost +
          (rightArea[i] / parentArea) * rightCount[i] * kFaceBvhIntersectCost;
      if (cost < bestCost) {
        bestCost = cost;
        bestAxis = axis;
        bestSplit = i;
      }
    }
  }

  const double leafCost = static_cast<double>(count) * kFaceBvhIntersectCost;
  if (bestAxis < 0 || bestCost >= leafCost) {
    node.first = begin;
    node.count = count;
    return nodeIndex;
  }

  auto first = faceIndices.begin() + begin;
  auto last = faceIndices.begin() + end;
  auto midIt = std::partition(first, last, [&](uint32_t faceId) {
    const BoundingBox &b = faces[faceId]->getBoundingBox();
    const glm::dvec3 c = 0.5 * (b.getMin() + b.getMax());
    const int bin = centroidBin(c[bestAxis], cmin[bestAxis], cmax[bestAxis]);
    return bin <= bestSplit;
  });

  uint32_t mid = static_cast<uint32_t>(midIt - faceIndices.begin());
  if (mid == begin || mid == end) {
    const uint32_t median = begin + (count / 2);
    std::nth_element(first, faceIndices.begin() + median, last,
                     [&](uint32_t a, uint32_t b) {
                       const BoundingBox &ab = faces[a]->getBoundingBox();
                       const BoundingBox &bb = faces[b]->getBoundingBox();
                       const glm::dvec3 ac = 0.5 * (ab.getMin() + ab.getMax());
                       const glm::dvec3 bc = 0.5 * (bb.getMin() + bb.getMax());
                       return ac[bestAxis] < bc[bestAxis];
                     });
    mid = median;
  }

  if (mid == begin || mid == end) {
    const int axis = maxExtentAxis(centroidBounds);
    const uint32_t median = begin + (count / 2);
    std::nth_element(first, faceIndices.begin() + median, last,
                     [&](uint32_t a, uint32_t b) {
                       const BoundingBox &ab = faces[a]->getBoundingBox();
                       const BoundingBox &bb = faces[b]->getBoundingBox();
                       const glm::dvec3 ac = 0.5 * (ab.getMin() + ab.getMax());
                       const glm::dvec3 bc = 0.5 * (bb.getMin() + bb.getMax());
                       return ac[axis] < bc[axis];
                     });
    mid = median;
  }

  if (mid == begin || mid == end) {
    node.first = begin;
    node.count = count;
    return nodeIndex;
  }

  node.left = buildFaceBvhRecursive(begin, mid);
  node.right = buildFaceBvhRecursive(mid, end);
  node.count = 0;
  return nodeIndex;
}

bool TrimeshFace::intersect(ray &r, isect &i) const {
  return intersectLocal(r, i);
}

/*
in class we used v as direction but it overlaps with barycentric coord v so its
d here. ray equation: o+vt (o- origin, d - direction, t - distance along the
ray, unknown) triangle equation: a + u(b-a) + v(c-a) (a,b,c - triangle vertices,
u,v - barycentric coords, unknown) so we want to solve for t,u,v such that: o +
dt = a + u(b-a) + v(c-a) = a + uAB + vAC rearranging: o - a = -dt + uAB + vAC
or: dt - uAB - vAC = a - o
which we can write as a matrix equation:
[d -AB -AC] [t u v]^T = a - o
*/

// Intersect ray r with the triangle abc.  If it hits returns true,
// and put the parameter in t and the barycentric coordinates of the
// intersection in u (alpha) and v (beta).
bool TrimeshFace::intersectLocal(ray &r, isect &i, double bestT) const {
  // YOUR CODE HERE
  //
  // FIXME: Add ray-trimesh intersection

  /* To determine the color of an intersection, use the following rules:
     - If the parent mesh has non-empty `uvCoords`, barycentrically interpolate
       the UV coordinates of the three vertices of the face, then assign it to
       the intersection using i.setUVCoordinates().
     - Otherwise, if the parent mesh has non-empty `vertexColors`,
       barycentrically interpolate the colors from the three vertices of the
       face. Create a new material by copying the parent's material, set the
       diffuse color of this material to the interpolated color, and then
       assign this material to the intersection.
     - If neither is true, assign the parent's material to the intersection.
  */

  const glm::dvec3 &ABedge = e1;
  const glm::dvec3 &ACedge = e2;
  /*
  a point is inside the triangle if all of its barycentric coordinates
  sum to 1 and are all positive. if a point is outside the triangle then
  one of its barycentric coordinates is negative. but they always sum to 1.
  vertices of a triangle have barycentric coordinates: (0,0,1), (0,1,0), (1,0,0)
  */
  /*
  can calculate if the ray is parallel to the triangle by finding the
  determinant of the [d AB AC] matrix from above. if det is 0 then
  the ray is either parallel or runs along the edge.
  */
  glm::dvec3 o = r.getPosition();
  glm::dvec3 d = r.getDirection();
  glm::dvec3 AO = o - v0; // vector from v0 to ray origin

  glm::dvec3 dCrossAC = glm::cross(d, ACedge);
  double det = glm::dot(ABedge, dCrossAC);

  const double EPS = 1e-8;

  // const glm::dvec3 kt = parent->getMaterial().kt(isect());
  // const bool hasTransmission = (kt.x > 1e-6 || kt.y > 1e-6 || kt.z > 1e-6);
  const bool trans = parent->getMaterial().Trans();
  const bool cullBackface = traceUI && traceUI->bkFaceSw() && !trans &&
                            r.type() != ray::SHADOW;
  if (cullBackface) {
    if (det < EPS) {
      return false;
    }
  } else if (fabs(det) < EPS) {
    return false;
  }

  // u, v, t

  double u = glm::dot(AO, dCrossAC) / det;
  if (u < 0.0 || u > 1.0)
    return false;

  const glm::dvec3 aoCrossAb = glm::cross(AO, ABedge);
  double v = glm::dot(d, aoCrossAb) / det;
  if (v < 0.0 || (u + v) > 1.0)
    return false;

  double t = glm::dot(ACedge, aoCrossAb) / det;
  if (t < EPS || t > bestT)
    return false; // intersection is behind ray or farther than a previous hit

  // setting stuff
  i.setT(t);
  i.setBary(1.0 - u - v, u, v);
  // OLD PRE PHONG INTERP
  // i.setN(this->normal); // OR UNCOMMENT THIS AND GET RID OF ELSE BELOW

  // TODO: if have texture coords need to interpolate it here.
  // POST PHONG INTERP

  if (parent->vertNorms) { // if parent mesh has vertex norms -> get norms for 3
                           // verteices
    glm::dvec3 norm1 = parent->normals[ids[0]];
    glm::dvec3 norm2 = parent->normals[ids[1]];
    glm::dvec3 norm3 = parent->normals[ids[2]];

    // interpolate  (1-u-v) for norm1, u for norm 2, v for norm 2
    glm::dvec3 interpolatedNorm =
        ((1.0 - u - v) * norm1) + (u * norm2) + (v * norm3);
    i.setN(glm::normalize(interpolatedNorm));
  } else {
    i.setN(this->normal);
  }

  i.setObject(this->parent);

  // i.setMaterial(this->parent->getMaterial()); // using parent

  if (!parent->uvCoords.empty()) {
    glm::dvec2 uv0 = parent->uvCoords[ids[0]];
    glm::dvec2 uv1 = parent->uvCoords[ids[1]];
    glm::dvec2 uv2 = parent->uvCoords[ids[2]];
    glm::dvec2 uv = (1.0 - u - v) * uv0 + u * uv1 + v * uv2;
    i.setUVCoordinates(uv);
    // tangent frame for normal mapping: T and B from UV derivatives
    glm::dvec2 deltaUV1 = uv1 - uv0;
    glm::dvec2 deltaUV2 = uv2 - uv0;
    double f = deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y;
    if (std::abs(f) > 1e-10) {
      f = 1.0 / f;
      glm::dvec3 dPdu = f * (deltaUV2.y * e1 - deltaUV1.y * e2);
      glm::dvec3 dPdv = f * (-deltaUV2.x * e1 + deltaUV1.x * e2);
      i.setTangentFrame(dPdu, dPdv);
    }
  } else if (!parent->vertColors.empty()) { // getting vertex colors
    glm::dvec3 color1 = parent->vertColors[ids[0]];
    glm::dvec3 color2 = parent->vertColors[ids[1]];
    glm::dvec3 color3 = parent->vertColors[ids[2]];

    glm::dvec3 interpolatedColor =
        ((1.0 - u - v) * color1) + (u * color2) + (v * color3);

    Material m = this->parent->getMaterial();
    m.setDiffuse(interpolatedColor);
    i.setMaterial(m);
  }

  return true;
}

// Once all the verts and faces are loaded, per vertex normals can be
// generated by averaging the normals of the neighboring faces.
void Trimesh::generateNormals() {
  int cnt = vertices.size();
  normals.resize(cnt);
  std::vector<int> numFaces(cnt, 0);

  for (auto face : faces) {
    glm::dvec3 faceNormal = face->getNormal();

    for (int i = 0; i < 3; ++i) {
      normals[(*face)[i]] += faceNormal;
      ++numFaces[(*face)[i]];
    }
  }

  for (int i = 0; i < cnt; ++i) {
    if (numFaces[i])
      normals[i] /= numFaces[i];
  }

  vertNorms = true;
}
