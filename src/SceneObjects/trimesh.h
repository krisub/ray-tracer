#ifndef TRIMESH_H__
#define TRIMESH_H__

#include <list>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

#include "../scene/kdTree.h"
#include "../scene/material.h"
#include "../scene/ray.h"
#include "../scene/scene.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/vec3.hpp>

class TrimeshFace;

class Trimesh : public SceneObject {
  friend class TrimeshFace;
  typedef std::vector<glm::dvec3> Normals;
  typedef std::vector<glm::dvec3> Vertices;
  typedef std::vector<TrimeshFace *> Faces;
  typedef std::vector<glm::dvec3> VertColors;
  typedef std::vector<glm::dvec2> UVCoords;

  Vertices vertices;
  Faces faces;
  Normals normals;
  VertColors vertColors;
  UVCoords uvCoords;
  BoundingBox localBounds;

  struct FaceBvhNode {
    BoundingBox bounds;
    uint32_t left = 0;
    uint32_t right = 0;
    uint32_t first = 0;
    uint32_t count = 0;
  };

  struct alignas(64) FaceBvh8Node {
    float minX[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    float minY[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    float minZ[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    float maxX[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    float maxY[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    float maxZ[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t childNode[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t first[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t count[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t childCount = 0;
    uint8_t leafMask = 0;
  };

  mutable std::vector<uint32_t> faceIndices;
  mutable std::vector<FaceBvhNode> faceBvhNodes;
  mutable std::vector<FaceBvh8Node> faceBvh8Nodes;
  mutable bool faceBvhDirty = true;
  mutable std::mutex faceBvhBuildMutex;

  void rebuildFaceBvhIfNeeded() const;
  uint32_t buildFaceBvhRecursive(uint32_t begin, uint32_t end) const;
  bool isFaceBinaryLeaf(uint32_t idx) const;
  uint32_t buildWide8FromBinary(uint32_t binaryRoot) const;
  void buildWide8() const;
  bool intersectWide8(ray &r, isect &i) const;
  bool intersectBinary(ray &r, isect &i) const;

public:
  Trimesh(Scene *scene, Material *mat, MatrixTransform transform)
      : SceneObject(scene, mat), displayListWithMaterials(0),
        displayListWithoutMaterials(0) {
    this->transform = transform;
    vertNorms = false;
  }

  bool vertNorms;

  bool intersectLocal(ray &r, isect &i) const;

  ~Trimesh();

  // must add vertices, normals, and materials IN ORDER
  void addVertex(const glm::dvec3 &);
  void addNormal(const glm::dvec3 &);
  void addColor(const glm::dvec3 &);
  void addUV(const glm::dvec2 &);
  bool addFace(int a, int b, int c);

  const char *doubleCheck();

  void generateNormals();

  bool hasBoundingBoxCapability() const { return true; }

  BoundingBox ComputeLocalBoundingBox() {
    BoundingBox localbounds;
    if (vertices.size() == 0)
      return localbounds;
    localbounds.setMax(vertices[0]);
    localbounds.setMin(vertices[0]);
    Vertices::const_iterator viter;
    for (viter = vertices.begin(); viter != vertices.end(); ++viter) {
      localbounds.setMax(glm::max(localbounds.getMax(), *viter));
      localbounds.setMin(glm::min(localbounds.getMin(), *viter));
    }
    localBounds = localbounds;
    return localbounds;
  }

protected:
  void glDrawLocal(int quality, bool actualMaterials,
                   bool actualTextures) const;
  mutable int displayListWithMaterials;
  mutable int displayListWithoutMaterials;
};

/* A triangle in a mesh. This class looks and behaves a lot like other
SceneObjects (e.g. Trimesh, Sphere, etc.) and has many of the same members
like intersectLocal() and a BoundingBox.

However, SceneObjects must have a MatrixTransform and a Material, and storing
these in every single TrimeshFace would explode memory usage. Because of this,
TrimeshFace is treated as an implementation detail of Trimesh and is not within
the SceneObject hierarchy.

Access to materials and transform are provided by referencing the parent
Trimesh object. */
class TrimeshFace {
  Trimesh *parent;
  int ids[3];
  glm::dvec3 v0;
  glm::dvec3 e1;
  glm::dvec3 e2;
  glm::dvec3 normal;
  double dist;
  BoundingBox bounds;

public:
  TrimeshFace(Trimesh *parent, int a, int b, int c) {
    this->parent = parent;
    ids[0] = a;
    ids[1] = b;
    ids[2] = c;

    // Compute the face normal here, not on the fly
    const glm::dvec3 a_coords = parent->vertices[a];
    const glm::dvec3 b_coords = parent->vertices[b];
    const glm::dvec3 c_coords = parent->vertices[c];

    v0 = a_coords;
    e1 = b_coords - a_coords;
    e2 = c_coords - a_coords;
    glm::dvec3 vab = e1;
    glm::dvec3 vac = e2;
    glm::dvec3 vcb = (b_coords - c_coords);

    if (glm::length(vab) == 0.0 || glm::length(vac) == 0.0 ||
        glm::length(vcb) == 0.0)
      degen = true;
    else {
      degen = false;
      normal = glm::cross(e1, e2);
      normal = glm::normalize(normal);
      dist = glm::dot(normal, a_coords);
    }
    localbounds = ComputeLocalBoundingBox();
    bounds = localbounds;
  }

  BoundingBox localbounds;
  bool degen;

  int operator[](int i) const { return ids[i]; }

  glm::dvec3 getNormal() { return normal; }

  bool intersect(ray &r, isect &i) const;
  bool intersectLocal(ray &r, isect &i, double bestT = std::numeric_limits<double>::infinity()) const;
  Trimesh *getParent() const { return parent; }

  bool hasBoundingBoxCapability() const { return true; }

  BoundingBox ComputeLocalBoundingBox() {
    BoundingBox localbounds;
    localbounds.setMax(
        glm::max(parent->vertices[ids[0]], parent->vertices[ids[1]]));
    localbounds.setMin(
        glm::min(parent->vertices[ids[0]], parent->vertices[ids[1]]));

    localbounds.setMax(
        glm::max(parent->vertices[ids[2]], localbounds.getMax()));
    localbounds.setMin(
        glm::min(parent->vertices[ids[2]], localbounds.getMin()));
    return localbounds;
  }

  const BoundingBox &getBoundingBox() const { return localbounds; }
};

#endif // TRIMESH_H__
