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
  size_t nx          = 0;         ///< Number of system states
  size_t n_obstacles = 0;         ///< Number of obstacles in the map
  std::vector<scalar_t> val;      ///< Safety measurements
                                  /**< In column major order, size n_obstacles*1. */
  std::vector<scalar_t> dval_dt;  ///< Partial derivative of safety measurements w.r.t time
                                  /**< In column major order, size n_obstacles*1,
                                       empty if not available. */
  std::vector<scalar_t> dval_dx;  ///< Partial derivative of safety measurements w.r.t state
                                  /**< In column major order, size n_obstacles*nx,
                                       empty if not available. */

  SafetyMapProbingResult()                                           = default;
  SafetyMapProbingResult(const SafetyMapProbingResult &)             = default;
  SafetyMapProbingResult(SafetyMapProbingResult &&)                  = default;
  SafetyMapProbingResult & operator=(const SafetyMapProbingResult &) = default;
  SafetyMapProbingResult & operator=(SafetyMapProbingResult &&)      = default;
  ~SafetyMapProbingResult()                                          = default;

  SafetyMapProbingResult(
    const size_t nx_, const size_t n_obstacles_, const bool with_gradient = false)
      : nx{nx_}, n_obstacles{n_obstacles_}, val(n_obstacles, scalar_t(0.))
  {
    if (with_gradient) {
      dval_dt.assign(n_obstacles, scalar_t(0.));
      dval_dx.assign(n_obstacles * nx, scalar_t(0.));
    }
  }

  using SharedPtr = std::shared_ptr<SafetyMapProbingResult>;
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
  virtual size_t n_obstacles() const = 0;

  /**
   * @brief Number of system states
   */
  virtual size_t nx() const = 0;

  /**
   * @brief Probe safety map for a given state value x
   *
   * @param x State at which to probe the map (size must match result of nx())
   * @param t_nsec Time at which to evaluate the map (in nanoseconds, default 0)
   */
  virtual std::shared_ptr<SafetyMapProbingResult> probe(
    const span<const scalar_t, dynamic_extent> x, const t_t t_nsec = 0) const = 0;

  using SharedPtr = std::shared_ptr<SafetyMapAbstract>;
};

}  // namespace lll

#endif  // THREELAWS_SAFETY_MAP_ABSTRACT_HPP
