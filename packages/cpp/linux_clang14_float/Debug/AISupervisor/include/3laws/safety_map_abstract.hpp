/**
 * @file safety_map_abstract.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief Safety map interface
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#ifndef THREELAWS_SAFETY_MAP_ABSTRACT_HPP
#define THREELAWS_SAFETY_MAP_ABSTRACT_HPP

#include <memory>
#include <vector>

#include <3laws/span.hpp>

#include "3laws/common.hpp"

namespace lll {

/**
 * @brief Safety map probing result
 * @details Given an n_obstacle dimensional vector of safety measurements \f$ h(x)
 * \f$ evaluated at \f$ x \in \mathbb{R}^{\text{nx}} \f$ then:
 * \f{array}{ \\
 *    \text{val} = h(x) \\
 *    \text{dval_dx} = \frac{\partial{h(x)}}{\partial{x}}
 * \f}
 * States \f$ x \f$ for which a component of \f$ h(x) \f$ is negative are considered unsafe sates.
 */
struct SafetyMapProbingResult
{
  std::size_t nx          = 0;    ///< Number of system states
  std::size_t n_obstacles = 0;    ///< Number of obstacles in the map
  std::vector<scalar_t> val;      ///< Safety measurements
                                  /**< In column major order, size n_obstacles*/
  std::vector<scalar_t> dval_dx;  ///< Partial derivative of safety measurements w.r.t state
                                  /**< In column major order, size n_obstacles*nx,
                                       empty if not available. */
};

/**
 * @brief Safety map interface
 * @details  Abstract class for a generic description of the safety space of a dynamical system.
 */
class SafetyMapAbstract
{
public:
  SafetyMapAbstract()                                          = default;
  SafetyMapAbstract(const SafetyMapAbstract &)                 = default;
  SafetyMapAbstract(SafetyMapAbstract &&) noexcept             = default;
  SafetyMapAbstract & operator=(const SafetyMapAbstract &)     = default;
  SafetyMapAbstract & operator=(SafetyMapAbstract &&) noexcept = default;
  virtual ~SafetyMapAbstract()                                 = default;

  /**
   * @brief Check if map probing result contains gradient information. False by default.
   */
  virtual bool has_gradient() const { return false; }

  /**
   * @brief Number of map obstacles
   */
  virtual std::size_t n_obstacles() const = 0;

  /**
   * @brief Number of system states
   */
  virtual std::size_t nx() const = 0;

  /**
   * @brief Probe safety map for a given state value x
   *
   * @param x State at which to probe the map (size must match result of nx())
   */
  virtual std::shared_ptr<SafetyMapProbingResult> probe(
    const span<const scalar_t, dynamic_extent> x) const = 0;
};

}  // namespace lll

#endif  // THREELAWS_SAFETY_MAP_ABSTRACT_HPP
