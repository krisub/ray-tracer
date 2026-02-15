#ifndef __PORTAL_H__
#define __PORTAL_H__

#include <GL/glu.h>
#include "../scene/scene.h"

class Portal : public SceneObject {
public:
  Portal(Scene *scene, Material *mat, double radius = 0.5)
      : SceneObject(scene, mat), radius_(radius) {}

  virtual bool intersectLocal(ray &r, isect &i) const;
  virtual bool hasBoundingBoxCapability() const { return true; }

  virtual BoundingBox ComputeLocalBoundingBox() {
    BoundingBox localbounds;
    localbounds.setMin(
        glm::dvec3(-radius_, -radius_, -0.001)); // thin disc
    localbounds.setMax(glm::dvec3(radius_, radius_, 0.001));
    return localbounds;
  }

  // set the paired portal (call on both portals to link them)
  void setPartner(Portal *other) { partner_ = other; }
  Portal *getPartner() const { return partner_; }

  // compute exit ray when entering this portal at hitPoint with rayDir
  // return true if partner exists; exitOrigin and exitDir are set
  bool computeExitRay(const glm::dvec3 &hitPoint, const glm::dvec3 &rayDir,
                     glm::dvec3 &exitOrigin, glm::dvec3 &exitDir) const;

protected:
  void glDrawLocal(int quality, bool actualMaterials,
                  bool actualTextures) const override;

private:
  double radius_;
  Portal *partner_ = nullptr;
};

#endif // __PORTAL_H__
