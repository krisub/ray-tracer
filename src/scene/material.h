/**
 * @file material.h
 * @brief Material class and other structures for surface properties
 *
 * contains classes describing how light interacts with surface geomery:
 * - `TextureMap`: manages image data for texture mapping
 * - `MaterialParameter`: container that can hold either a solid
 * color or a texture map
 * - `Material`: main container for Phong shading properties (diffuse,
 * specular, ambient)
 *
 * these classes are referenced by `SceneObject`, `RayTracer`, `isect`, and
 * others
 */

//
// material.h
//
// The Material class: a description of the phsyical properties of a surface
// that are used to determine how that surface interacts with light.

#ifndef __MATERIAL_H__
#define __MATERIAL_H__

#include <glm/glm.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <stdint.h>
#include <string>
#include <vector>

class Scene;
class ray;
class isect;

using std::string;

class ReactionDiffusionTexture;

/**
 * @class TextureSource
 * @brief Abstract base for texture sources (image files or procedural)
 */
class TextureSource {
public:
  virtual ~TextureSource() {}
  virtual glm::dvec3 getMappedValue(const glm::dvec2 &coord) const = 0;
};

/* The TextureMap class can be used to store a texture map,
which consists of a bitmap and various accessors to it. To implement basic
texture mapping, you'll want to fill in the getMappedValue function to
implement basic texture mapping. */
/**
 * @class TextureMap
 * @brief manages a 2D bitmap image for texture mapping
 *
 * loads image files and provides color balues based on 2D parametric
 * coordinates (u, v)
 *
 * used by `MaterialParameter` when a surface property is defined by a texture
 * rather than a solid/constant color
 */
class TextureMap : public TextureSource {
public:
  /**
   * @brief constructs a TextureMap from an image file
   *
   * loads the file using `readImage` (from fileio/images.h)
   * @param filename path to the image file
   * @throws TextureMapException if the file cannot be loaded
   */
  TextureMap(string filename);

  // Return the mapped value; here the coordinate is assumed to be within
  // the parametrization space:
  // [0, 1] x [0, 1]
  // (i.e., {(u, v): 0 <= u <= 1 and 0 <= v <= 1}
  /**
   * @brief retrieves the color at the given uv coordinates
   *
   * this function maps the unit square [0,1]x[0,1] to the image dimensions
   * and performs bilinear interpolation to retrieve the color
   *
   * @param coord the 2D texture coordinates (u, v)
   * @return glm::dvec3 the interpolated color value
   */
  glm::dvec3 getMappedValue(const glm::dvec2 &coord) const override;

  // Retrieve the value stored in a physical location (with integer coordinates)
  // in the bitmap. Should be called from getMappedValue in order to do
  // bilinear interpolation.
  /**
   * @brief retrieves the raw pixel color at integer image coordinates
   *
   * helper function used by `getMappedValue` for interpolation
   *
   * @param x the x-coordinate (column) in the bitmap
   * @param y the y-coordinate (row) in the bitmap
   * @return glm::dvec3 the pixel color
   */
  glm::dvec3 getPixelAt(int x, int y) const;

  /** @brief returns the width of the texture image in pixels */
  int getWidth() const { return width; }
  /** @brief returns the height of the texture image in pixels */
  int getHeight() const { return height; }

  ~TextureMap() {}

protected:
  int width;                 ///< image width
  int height;                ///< image height
  std::vector<uint8_t> data; ///< raw image data (RGB)
};

class TextureMapException {
public:
  TextureMapException(string errorMsg) : _errorMsg(errorMsg) {}
  string message() { return _errorMsg; }

private:
  string _errorMsg;
};

/*
MaterialParameter is a helper class for a material; it stores either a constant
value (in a 3-vector) or else a link to a map of some type. If the pointer to
the TextureMap is NULL, the class simply returns whatever is stored in the
constant vector.

This is unabashedly a speed hack; we are replacing a potential use of
polymorphism with an old-school "if" statement. But raytracers are slow enough,
and the Material class is used often enough, that it is (somewhat) justifiable
to do this.
*/
/**
 * @class MaterialParameter
 * @brief wrapper that stores either a constant color or a TextureMap
 *
 * abstracts the difference between a solid material property (a red triangle)
 * and a textured one (a marble triangle). The `value()`
 * method handles the logic of checking if a map exists and returning the
 * appropriate color
 *
 * this is an optimization to avoid virtual function overhead for every
 * single material property
 */
class MaterialParameter {
public:
  /** @brief constructs a parameter with a constant 3D color/vector */
  explicit MaterialParameter(const glm::dvec3 &par)
      : _value(par), _textureSource(0) {}

  /** @brief constructs a parameter with a constant scalar (replicated to
   * x,y,z) */
  explicit MaterialParameter(const double par)
      : _value(par, par, par), _textureSource(0) {}

  /** @brief constructs a parameter linked to a texture map */
  explicit MaterialParameter(TextureMap *tex) : _textureSource(tex) {}

  /** @brief constructs a parameter linked to a reaction-diffusion texture */
  explicit MaterialParameter(ReactionDiffusionTexture *tex);

  /** @brief default constructor (black/zero) */
  MaterialParameter() : _value(0.0, 0.0, 0.0), _textureSource(0) {}

  // these are all operators for combining material parameters
  MaterialParameter &operator*=(const MaterialParameter &rhs) {
    (*this) *= rhs._value;
    return *this;
  }

  glm::dvec3 &operator*=(const glm::dvec3 &rhs) {
    _value[0] *= rhs[0];
    _value[1] *= rhs[1];
    _value[2] *= rhs[2];
    return _value;
  }

  glm::dvec3 &operator*=(const double rhs) {
    _value[0] *= rhs;
    _value[1] *= rhs;
    _value[2] *= rhs;
    return _value;
  }

  MaterialParameter &operator+=(const MaterialParameter &rhs) {
    _value += rhs._value;
    return *this;
  }

  /** @brief sets the constant value (removes texture map if present) */
  void setValue(const glm::dvec3 &rhs) {
    _value = rhs;
    _textureSource = 0;
  }

  /** @brief sets the constant scalar value (removes texture map if present) */
  void setValue(const double rhs) {
    _value[0] = rhs;
    _value[1] = rhs;
    _value[2] = rhs;
    _textureSource = 0;
  }

  /** @brief checks if the constant value is effectively zero (black) */
  bool isZero() { return glm::length(_value) == 0.0; }

  glm::dvec3 &operator+=(const glm::dvec3 &rhs) {
    _value += rhs;
    return _value;
  }

  /**
   * @brief retrieves the value of the parameter at a specific intersection
   *
   * if a TextureMap is present, it uses the uv coordinates from the
   * intersection `is`. otherwise it returns the constant `_value`.
   *
   * @param is the intersection details (containing uv coords)
   * @return glm::dvec3 The effective color/vector
   */
  glm::dvec3 value(const isect &is) const;

  /**
   * @brief retrieves the intensity (grayscale/scalar) value
   *
   * useful for scalar properties like shininess or refractive index.
   * uses NTSC conversion weights if deriving from a color texture: 0.299R +
   * 0.587G + 0.114B (?)
   *
   * @param is the intersection details
   * @return double The scalar intensity
   */
  double intensityValue(const isect &is) const;

  // Use this to determine if the particular parameter is
  // mapped; use this to determine if we need to somehow renormalize.
  /** @brief returns true if this param uses a texture map */
  bool mapped() const { return _textureSource != 0; }

private:
  glm::dvec3 _value;           ///< constant color/vector value
  TextureSource *_textureSource; ///< pointer to texture (image or procedural)
};

/**
 * @class Material
 * @brief defines the physical properties of a surface (Phong Model).
 *
 * a Material acts as a container for `MaterialParameter`s,
 * corresponding to the standard Phong reflection model components:
 * - Ke: Emissive
 * - Ka: Ambient
 * - Ks: Specular
 * - Kd: Diffuse
 * - Kr: Reflective
 * - Kt: Transmissive
 *
 * it also caches boolean flags (e.g., `_refl`, `_trans`) to quickly check if
 * complex shading (recursion) is required.
 */
class Material {
public:
  /** @brief default constructor. initializes all properties to black/zero. */
  Material()
      : _ke(glm::dvec3(0.0, 0.0, 0.0)), _ka(glm::dvec3(0.0, 0.0, 0.0)),
        _ks(glm::dvec3(0.0, 0.0, 0.0)), _kd(glm::dvec3(0.0, 0.0, 0.0)),
        _kr(glm::dvec3(0.0, 0.0, 0.0)), _kt(glm::dvec3(0.0, 0.0, 0.0)),
        _refl(0), _trans(0), _recur(0), _spec(0), _both(0), _shininess(0.0),
        _index(1.0) {}

  virtual ~Material();

  /**
   * @brief constructor with explicit values for all Phong components.
   *
   * calls `setBools()` to initialize optimization flags
   */
  Material(const glm::dvec3 &e, const glm::dvec3 &a, const glm::dvec3 &s,
           const glm::dvec3 &d, const glm::dvec3 &r, const glm::dvec3 &t,
           double sh, double in)
      : _ke(e), _ka(a), _ks(s), _kd(d), _kr(r), _kt(t),
        _shininess(glm::dvec3(sh, sh, sh)), _index(glm::dvec3(in, in, in)) {
    setBools();
  }

  /**
   * @brief calculates the color of the surface at a specific intersection
   *
   * this is the core shading function. implements the Phong Illumination
   * Model and sums contributions from Emissive, Ambient, Diffuse, and Specular
   * components for all lights in the scene
   *
   * @param scene pointer to the scene (to access lights and ambient light)
   * @param r the ray that hit the surface (provides view direction)
   * @param i the intersection details (provides position, normal, UVs)
   * @return glm::dvec3 the computed color
   */
  virtual glm::dvec3 shade(Scene *scene, const ray &r, const isect &i) const;

  /** @brief adds another material to this one */
  Material &operator+=(const Material &m) {
    _ke += m._ke;
    _ka += m._ka;
    _ks += m._ks;
    _kd += m._kd;
    _kr += m._kr;
    _kt += m._kt;
    _index += m._index;
    _shininess += m._shininess;
    return *this;
  }

  friend Material operator*(double d, Material m);

  // Accessor functions; we pass in an isect& for cases where the parameter is
  // dependent on, for example, world-space coordinates (i.e., solid textures)
  // or parametrized coordinates (i.e., mapped textures)

  /** @brief get Emissive component (self-glowing light) */
  glm::dvec3 ke(const isect &i) const { return _ke.value(i); }

  /** @brief get Ambient component (interaction with background light) */
  glm::dvec3 ka(const isect &i) const { return _ka.value(i); }

  /** @brief get Specular component (shiny highlights) */
  glm::dvec3 ks(const isect &i) const { return _ks.value(i); }

  /** @brief get Diffuse component (matte surface color) */
  glm::dvec3 kd(const isect &i) const { return _kd.value(i); }

  /** @brief get Reflective component (mirror-like reflection) */
  glm::dvec3 kr(const isect &i) const { return _kr.value(i); }

  /** @brief get Transmissive component (refraction/transparency) */
  glm::dvec3 kt(const isect &i) const { return _kt.value(i); }

  /**
   * @brief get Shininess exponent (controls size of specular highlight).
   * if texture mapped, scales the intensity by 128.0.
   */
  double shininess(const isect &i) const {
    // Have to renormalize into the range 0-128 if it's texture
    // mapped.
    return _shininess.mapped() ? 128.0 * _shininess.intensityValue(i)
                               : _shininess.intensityValue(i);
  }

  /** @brief get index of refraction for snell's law */
  double index(const isect &i) const { return _index.intensityValue(i); }

  // setting functions accepting primitives (glm::dvec3 and double)
  void setEmissive(const glm::dvec3 &ke) { _ke.setValue(ke); }
  void setAmbient(const glm::dvec3 &ka) { _ka.setValue(ka); }
  void setSpecular(const glm::dvec3 &ks) {
    _ks.setValue(ks);
    setBools();
  }
  void setDiffuse(const glm::dvec3 &kd) { _kd.setValue(kd); }
  void setReflective(const glm::dvec3 &kr) {
    _kr.setValue(kr);
    setBools();
  }
  void setTransmissive(const glm::dvec3 &kt) {
    _kt.setValue(kt);
    setBools();
  }
  void setShininess(double shininess) { _shininess.setValue(shininess); }
  void setIndex(double index) { _index.setValue(index); }

  // setting functions taking MaterialParameters
  void setEmissive(const MaterialParameter &ke) { _ke = ke; }
  void setAmbient(const MaterialParameter &ka) { _ka = ka; }
  void setSpecular(const MaterialParameter &ks) { _ks = ks; }
  void setDiffuse(const MaterialParameter &kd) { _kd = kd; }
  void setReflective(const MaterialParameter &kr) {
    _kr = kr;
    setBools();
  }
  void setTransmissive(const MaterialParameter &kt) {
    _kt = kt;
    setBools();
  }
  void setShininess(const MaterialParameter &shininess) {
    _shininess = shininess;
  }
  void setIndex(const MaterialParameter &index) { _index = index; }

  /** @brief set optional normal map texture for normal/bump mapping */
  void setNormalMap(TextureMap *tex) { _normalMap = tex; }
  /** @brief set strength multiplier for normal map (1.0 = as-is, >1 = more pronounced) */
  void setNormalMapStrength(double s) { _normalMapStrength = s; }
  /** @brief returns true if material has a normal map */
  bool hasNormalMap() const { return _normalMap != nullptr; }
  /** @brief get the normal map (may be nullptr) */
  TextureMap *getNormalMap() const { return _normalMap; }

  /** @brief set optional height map for parallax mapping */
  void setHeightMap(TextureMap *tex) { _heightMap = tex; }
  /** @brief set parallax scale (height multiplier) */
  void setParallaxScale(double s) { _parallaxScale = s; }
  /** @brief returns true if material has a height map */
  bool hasHeightMap() const { return _heightMap != nullptr; }

  /** @brief set metallic factor (0=dielectric, 1=metal) */
  void setMetallic(double m) { _metallic = m; }
  /** @brief get metallic factor */
  double metallic() const { return _metallic; }

  /** @brief set anisotropy factor (-1..1, 0=isotropic) */
  void setAnisotropy(double a) { _anisotropy = a; }
  /** @brief get anisotropy factor */
  double anisotropy() const { return _anisotropy; }

  
  bool isVolumetric() const { return _volumetric; }
  void setVolumetric(bool v) { _volumetric = v; }

  glm::dvec3 volumeAbsorption() const { return _volAbsorption; }
  void setVolumeAbsorption(const glm::dvec3 &a) { _volAbsorption = a; }

  glm::dvec3 volumeScattering() const { return _volScattering; }
  void setVolumeScattering(const glm::dvec3 &s) { _volScattering = s; }

  glm::dvec3 volumeExtinction() const { return _volAbsorption + _volScattering; }

  double volumePhaseG() const { return _volPhaseG; }
  void setVolumePhaseG(double g) { _volPhaseG = g; }

  glm::dvec3 volumeEmission() const { return _volEmission; }
  void setVolumeEmission(const glm::dvec3 &e) { _volEmission = e; }

  double volumeDensity() const { return _volDensity; }
  void setVolumeDensity(double d) { _volDensity = d; }

  double volumeStepSize() const { return _volStepSize; }
  void setVolumeStepSize(double s) { _volStepSize = s; }

  double volumeNoiseScale() const { return _volNoiseScale; }
  void setVolumeNoiseScale(double s) { _volNoiseScale = s; }

  int volumeNoiseOctaves() const { return _volNoiseOctaves; }
  void setVolumeNoiseOctaves(int o) { _volNoiseOctaves = o; }

  double volumeNoiseGain() const { return _volNoiseGain; }
  void setVolumeNoiseGain(double g) { _volNoiseGain = g; }

  double volumeFalloff() const { return _volFalloff; }
  void setVolumeFalloff(double f) { _volFalloff = f; }

  // get booleans for reflection and refraction
  bool Refl() const { return _refl; }   ///< is the material reflective?
  bool Trans() const { return _trans; } ///< is the material transmissive?
  bool Recur() const {
    return _recur;
  } ///< is it either reflective or transmissive?
  bool Spec() const {
    return _spec;
  } ///< does it have any specular or reflective component?
  bool Both() const {
    return _both;
  } ///< is it both reflective and transmissive?

private:
  MaterialParameter _ke; ///< emissive
  MaterialParameter _ka; ///< ambient
  MaterialParameter _ks; ///< specular
  MaterialParameter _kd; ///< diffuse
  MaterialParameter _kr; ///< reflective
  MaterialParameter _kt; ///< transmissive

  bool _refl;                   ///< specular reflector?
  bool _trans;                  ///< specular transmitter?
  bool _recur;                  ///< either one
  bool _spec;                   ///< any kind of specular?
  bool _both;                   ///< reflection and transmission
  MaterialParameter _shininess; ///< shininess exponent
  MaterialParameter _index;     ///< index of refraction
  TextureMap *_normalMap = nullptr; ///< optional normal map for normal mapping
  double _normalMapStrength = 3.0;  ///< strength multiplier for normal map bumps
  TextureMap *_heightMap = nullptr; ///< optional height map for parallax mapping
  double _parallaxScale = 0.1;      ///< parallax height scale (LearnOpenGL uses ~0.1)
  double _metallic = 0.0;            ///< metallic factor (0=dielectric, 1=metal)
  double _anisotropy = 0.0;          ///< anisotropy factor (-1..1, 0=isotropic)

  // volumetric/participating media properties for ray marching
  bool _volumetric = false;              ///< is this a volumetric object?
  glm::dvec3 _volAbsorption{0,0,0};      ///< absorption coefficient sigma_a
  glm::dvec3 _volScattering{0,0,0};      ///< scattering coefficient sigma_s
  double _volPhaseG = 0.0;               ///< Henyey-Greenstein asymmetry (-1..1, 0=isotropic)
  glm::dvec3 _volEmission{0,0,0};        ///< volume emission (radiance per unit distance)
  double _volDensity = 1.0;              ///< density multiplier
  double _volStepSize = 0.05;            ///< ray march step size (world units)
  double _volNoiseScale = 0.0;           ///< noise spatial frequency (0 = homogeneous)
  int    _volNoiseOctaves = 4;           ///< fBm octave count
  double _volNoiseGain = 0.5;            ///< fBm amplitude decay per octave
  double _volFalloff = 0.0;              ///< edge falloff exponent (0 = no falloff)

  void setBools() {
    _refl = !_kr.isZero();
    _trans = !_kt.isZero();
    _recur = _refl || _trans;
    _spec = _refl || !_ks.isZero();
    _both = _refl && _trans;
  }
};

// This doesn't necessarily make sense for mapped materials
inline Material operator*(double d, Material m) {
  m._ke *= d;
  m._ka *= d;
  m._ks *= d;
  m._kd *= d;
  m._kr *= d;
  m._kt *= d;
  m._index *= d;
  m._shininess *= d;
  return m;
}

#endif // __MATERIAL_H__
