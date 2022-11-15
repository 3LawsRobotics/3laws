/**
 * @file dynamical_model_abstract.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief Dynamical model interface
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#ifndef THREELAWS_DYNAMICAL_MODEL_ABSTRACT_HPP
#define THREELAWS_DYNAMICAL_MODEL_ABSTRACT_HPP

#include <memory>
#include <vector>

#include <3laws/span.hpp>

#include "3laws/common.hpp"
#include "3laws/config.hpp"

namespace lll {

/**
 * @brief Dynamical model evaluation result
 * @details Given a dynamical model \f$ \dot{x} = f(x,u) \f$ evaluated at \f$ x \in
 * \mathbb{R}^{\text{nx}} \f$ and \f$ u \in \mathbb{R}^{\text{nu}} \f$ then:
 * \f{array}{ \\
 *    \text{val} = f(x,u) \\
 *    \text{dval_dx} = \frac{\partial{f(x,u)}}{\partial{x}} \\
 *    \text{dval_du} = \frac{\partial{f(x,u)}}{\partial{u}}
 * \f}
 */
struct DynamicalModelEvaluationResult
{
  std::size_t nx = 0;  ///< Number of states
  std::size_t nu = 0;  ///< Number of inputs

  std::vector<scalar_t> val;  ///< Value of dynamics evaluation
                              /**< In column major order, size nx.*/

  std::vector<scalar_t> dval_dx;  ///< Partial derivative w.r.t state
                                  /**< In column major order, size nx*nx,
                                       empty if not available. */

  std::vector<scalar_t> dval_du;  ///< Partial derivative w.r.t input
                                  /**< In column major order, size nx*nu,
                                        empty if not available. */

  DynamicalModelEvaluationResult()                                                   = default;
  DynamicalModelEvaluationResult(const DynamicalModelEvaluationResult &)             = default;
  DynamicalModelEvaluationResult(DynamicalModelEvaluationResult &&)                  = default;
  DynamicalModelEvaluationResult & operator=(const DynamicalModelEvaluationResult &) = default;
  DynamicalModelEvaluationResult & operator=(DynamicalModelEvaluationResult &&)      = default;
  ~DynamicalModelEvaluationResult()                                                  = default;

  DynamicalModelEvaluationResult(
    const std::size_t nx_, const std::size_t nu_, const bool with_gradient = false)
      : nx{nx_}, nu{nu_}, val(nx, scalar_t(0.))
  {
    if (with_gradient) {
      dval_dx.assign(nx, scalar_t(0.));
      dval_du.assign(nx * nu, scalar_t(0.));
    }
  }
};

/**
 * @brief Dynamical model interface
 * @details Abstract class for a generic non-linear dynamical model of the form
 * \f$ \dot{x} = f(x,u) \f$ with \f$ x \in \mathbb{R}^{\text{nx}} \f$ and \f$ u \in
 * \mathbb{R}^{\text{nu}} \f$.
 */
class DynamicalModelAbstract
{
public:
  DynamicalModelAbstract()                                               = default;
  DynamicalModelAbstract(const DynamicalModelAbstract &)                 = default;
  DynamicalModelAbstract(DynamicalModelAbstract &&) noexcept             = default;
  DynamicalModelAbstract & operator=(const DynamicalModelAbstract &)     = default;
  DynamicalModelAbstract & operator=(DynamicalModelAbstract &&) noexcept = default;
  virtual ~DynamicalModelAbstract()                                      = default;

  /**
   * @brief Check if model evaluation result contains gradient information. False by default.
   */
  virtual bool has_gradient() const { return false; }

  /**
   * @brief Number of model states
   */
  virtual std::size_t nx() const = 0;

  /**
   * @brief Number of model inputs
   */
  virtual std::size_t nu() const = 0;

  /**
   * @brief Evaluate the model dynamics for a given state value x and input value u
   *
   * @param x State at which to evaluate the dynamics (size must match result of nx())
   * @param u Input at which to evaluate the dynamics (size must match result of nu())
   */
  virtual std::shared_ptr<DynamicalModelEvaluationResult> evaluate(
    const span<const scalar_t, dynamic_extent> x,
    const span<const scalar_t, dynamic_extent> u) const = 0;
};

}  // namespace lll

#endif  // THREELAWS_DYNAMICAL_MODEL_ABSTRACT_HPP
