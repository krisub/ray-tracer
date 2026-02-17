/**
 * @file ReactionDiffusionTexture.h
 * @brief Procedural texture based on reaction-diffusion (Turing patterns)
 *
 * implements reaction-diffusion textures as described in Witkin & Kass (SIGGRAPH '91)
 * and Foley et al. patterns arise from two morphogens evolving via diffusion + reaction.
 */

#ifndef __REACTION_DIFFUSION_TEXTURE_H__
#define __REACTION_DIFFUSION_TEXTURE_H__

#include "material.h"
#include <glm/glm.hpp>
#include <glm/vec2.hpp>
#include <string>
#include <vector>

/**
 * @class ReactionDiffusionTexture
 * @brief Generates Turing-pattern textures via reaction-diffusion simulation
 *
 * Precomputes a 2D pattern and provides getMappedValue(uv) compatible with
 * the texture sampling interface.
 */
class ReactionDiffusionTexture : public TextureSource {
public:
  /** Pattern presets: spots, stripes, wood, marble */
  enum class Pattern { Spots, Stripes, Wood, Marble };

  /**
   * @brief Construct and simulate RD texture
   * @param width  resolution in u
   * @param height resolution in v
   * @param pattern which preset to use
   * @param iterations number of simulation steps
   */
  ReactionDiffusionTexture(int width, int height, Pattern pattern = Pattern::Spots,
                           int iterations = 3000);

  /**
   * @brief Get color at UV coordinates [0,1]x[0,1]
   */
  glm::dvec3 getMappedValue(const glm::dvec2 &coord) const override;

  int getWidth() const { return width_; }
  int getHeight() const { return height_; }

private:
  void simulate();
  void simulateSpots();
  void simulateStripes();
  void simulateWood();
  void simulateMarble();
  double sampleConcentration(int x, int y) const;

  int width_;
  int height_;
  Pattern pattern_;
  int iterations_;

  /** Concentration difference C+ - C- after simulation (one channel is enough) */
  std::vector<double> concentration_;
};

#endif // __REACTION_DIFFUSION_TEXTURE_H__
