#include "trimesh.h"
#include "../ui/TraceUI.h"
#include <algorithm>
#include <array>
#include <assert.h>
#include <cmath>
#include <float.h>
#include <limits>
#include <numeric>
#include <string.h>
extern TraceUI *traceUI;
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

  // Iterative traversal to avoid recursion overhead.
  std::vector<uint32_t> stack;
  stack.reserve(64);
  stack.push_back(0u);
  bool have_one = false;

  while (!stack.empty()) {
    const uint32_t nodeIndex = stack.back();
    stack.pop_back();
    const FaceBvhNode &node = faceBvhNodes[nodeIndex];

    double nmin = 0.0;
    double nmax = 0.0;
    if (!node.bounds.intersect(r, nmin, nmax)) {
      continue;
    }
    if (have_one && nmin > i.getT()) {
      continue;
    }

    if (node.count > 0) {
      const uint32_t end = node.first + node.count;
      for (uint32_t idx = node.first; idx < end; ++idx) {
        TrimeshFace *face = faces[faceIndices[idx]];
        isect cur;
        if (face->intersectLocal(r, cur)) {
          if (!have_one || (cur.getT() < i.getT())) {
            i = cur;
            have_one = true;
          }
        }
      }
      continue;
    }

    const FaceBvhNode &left = faceBvhNodes[node.left];
    const FaceBvhNode &right = faceBvhNodes[node.right];
    double lmin = 0.0, lmax = 0.0;
    double rmin = 0.0, rmax = 0.0;
    const bool hitL = left.bounds.intersect(r, lmin, lmax);
    const bool hitR = right.bounds.intersect(r, rmin, rmax);

    if (hitL && hitR) {
      if (lmin <= rmin) {
        stack.push_back(node.right);
        stack.push_back(node.left);
      } else {
        stack.push_back(node.left);
        stack.push_back(node.right);
      }
    } else if (hitL) {
      stack.push_back(node.left);
    } else if (hitR) {
      stack.push_back(node.right);
    }
  }

  if (!have_one)
    i.setT(1000.0);
  return have_one;
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

  const uint32_t nFaces = static_cast<uint32_t>(faces.size());
  if (nFaces == 0) {
    faceBvhDirty = false;
    return;
  }

  faceIndices.resize(nFaces);
  std::iota(faceIndices.begin(), faceIndices.end(), 0u);
  faceBvhNodes.reserve(nFaces * 2u);
  (void)buildFaceBvhRecursive(0u, nFaces);
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
bool TrimeshFace::intersectLocal(ray &r, isect &i) const {
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

  const glm::dvec3 &a = parent->vertices[ids[0]]; // TrimeshFace constructor
  const glm::dvec3 &b = parent->vertices[ids[1]];
  const glm::dvec3 &c = parent->vertices[ids[2]];

  glm::dvec3 ABedge = b - a; // starting at a, the vector that gets to b. AB
  glm::dvec3 ACedge = c - a; // AC
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
  glm::dvec3 AO = o - a; // vector from a to ray origin. AO

  glm::dvec3 dCrossAC = glm::cross(d, ACedge);
  double det = glm::dot(ABedge, dCrossAC);

  const double EPS = 1e-8;
  // no bkface culling
  // if (fabs(det) < EPS)
  //   return false; // ray is parallel. (TODO: or runs along edge? is that
  // //   valid?)

  // backface culling

  const Material &mat = parent->getMaterial();
  // TODO: change when texture mapping
  bool isTransmissive = (glm::length(mat.kt(isect())) > 1e-6);

  if (traceUI->bkFaceSw() && !isTransmissive) {
    if (det < EPS) {
      return false; // ray is parallel or backfacing
    }
  } else {
    if (fabs(det) < EPS) {
      return false; // ray is parallel, may or may not be backfacing
    }
  }

  // u, v, t

  double u = glm::dot(AO, dCrossAC) / det;
  if (u < 0.0 || u > 1.0)
    return false;

  double v = glm::dot(d, glm::cross(AO, ABedge)) / det;
  if (v < 0.0 || (u + v) > 1.0)
    return false;

  double t = glm::dot(ACedge, glm::cross(AO, ABedge)) / det;
  if (t < EPS /*|| t > i.getT()*/)
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
    glm::dvec2 uv = (1.0 - u - v) * parent->uvCoords[ids[0]] +
                    u * parent->uvCoords[ids[1]] + v * parent->uvCoords[ids[2]];
    i.setUVCoordinates(uv);
  } else if (!parent->vertColors.empty()) { // getting vertex colors
    glm::dvec3 color1 = parent->vertColors[ids[0]];
    glm::dvec3 color2 = parent->vertColors[ids[1]];
    glm::dvec3 color3 = parent->vertColors[ids[2]];

    glm::dvec3 interpolatedColor =
        ((1.0 - u - v) * color1) + (u * color2) + (v * color3);

    Material m = this->parent->getMaterial();
    m.setDiffuse(interpolatedColor);
    i.setMaterial(m);
  } else {
    i.setMaterial(this->parent->getMaterial());
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
