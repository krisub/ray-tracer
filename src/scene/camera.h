/**
 * @file camera.h
 * @brief defines the Camera class which generates viewing rays.
 *
 * the Camera class simulates a simple pinhole camera. maintains the position
 * ("eye"), orientation ("look" vector), and the image plane dimensions (derived
 * from FOV and aspect ratio). primary job is to take a 2D pixel coordinate
 * and produce a 3D ray into the scene.
 *
 * referenced by:
 * - `Scene`: the scene owns the camera.
 * - `RayTracer::trace`: calls `rayThrough` to generate the initial rays for
 * each pixel.
 * - `TraceUI`: updates camera parameters based on user input (mouse movement,
 * sliders).
 */

#ifndef CAMERA_H
#define CAMERA_H

#include "ray.h"
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

class Camera {
public:
  /**
   * @brief default camera looks down the -Z axis at (0,0,0).
   */
  Camera();
  /**
   * @brief generates a ray from the camera eye through pixel (x,y)
   *
   * maps the normalized screen coords to ap oint on the image plane in
   * 3D space then construct a ray from the eye to that point
   *
   * @param x normalized x coord in [0,1]
   * @param y normalized y coord in [0,1]
   * @param r output ray
   *
   * called by `RayTracer::trace` to generate primary rays for each pixel
   */
  void rayThrough(double x, double y, ray &r);

  /**
   * @brief sets the camera eye position
   * @param eye eye position as 3D vector
   */
  void setEye(const glm::dvec3 &eye);
  /**
   * @brief sets the camera look direction using angles
   *
   * rotates default view (down is -Z, up is +Y) by quaternion (r,i,j,k)
   * and internally updates the rotation matrix `m`.
   *
   * @param r real part of quaternion
   * @param i first imaginary part
   * @param j second imaginary part
   * @param k third imaginary part
   */
  void setLook(double, double, double, double);

  /**
   * @brief sets the camera look direction using view and up vectors
   *
   * computes the camera rotation matrix `m` based on the desired
   * view direction and up vector.
   *
   * @param viewDir desired view direction (look vector)
   * @param upDir desired up vector
   */
  void setLook(const glm::dvec3 &viewDir, const glm::dvec3 &upDir);

  /**
   * @brief sets the vertical field of view (in degrees). calculates physical
   * height of image plane a unit distance from eye.
   *
   * @param fov vertical field of view in degrees
   */
  void setFOV(double);

  /**
   * @brief sets the aspect ratio (width/height) of the image plane. scales the
   * horizontal direction.
   * @param ar aspect ratio
   */
  void setAspectRatio(double);
  /**
   * @brief gets the current aspect ratio
   * @return double aspect ratio (width/height)
   */
  double getAspectRatio() { return aspectRatio; }

  /** @return the camera's position in world space */
  const glm::dvec3 &getEye() const { return eye; }

  /** @return the forward looking direction vector (normalized) */
  const glm::dvec3 &getLook() const { return look; }

  /** @return the "Right" vector (camera's local X axis) */
  const glm::dvec3 &getU() const { return u; }

  /** @return the "Up" vector (camera's local Y axis) */
  const glm::dvec3 &getV() const { return v; }

private:
  glm::dmat3 m;            ///< rotation matrix
  double normalizedHeight; ///< dimensions of image plane at unit dist from eye
  double aspectRatio;      ///< image width / height ratio

  /**
   * @brief recalculates the camera basis vectors (u, v, look).
   *
   * called internally whenever the rotation matrix, FOV, or aspect ratio
   * changes. sets:
   * - `look`: forward vector
   * - `u`: horizontal vector (scaled by aspect ratio)
   * - `v`: vertical vector (scaled by FOV)
   */
  void update(); // using the above three values calculate look,u,v

  glm::dvec3 eye;  ///< Location of the camera eye
  glm::dvec3 look; ///< direction to look
  glm::dvec3 u, v; ///< u and v in the image coordinate system.
};

#endif
