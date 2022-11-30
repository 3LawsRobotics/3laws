/**
 * @file affine_dynamical_model_abstract.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief Affine dynamical model interface
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#ifndef THREELAWS_AFFINE_DYNAMICAL_MODEL_ABSTRACT_HPP
#define THREELAWS_AFFINE_DYNAMICAL_MODEL_ABSTRACT_HPP

#include <memory>
#include <vector>

#include <3laws/span.hpp>

#include "3laws/common.hpp"
#include "3laws/dynamical_model_abstract.hpp"

namespace lll {

/**
 * @brief Affine Dynamical model evaluation result
 * @details Given an affine dynamical model \f$ \dot{x} = f(x) + g(x)u \f$ evaluated at
 * \f$ x \in \mathbb{R}^{\text{nx}} \f$ then:
 * \f{array}{ \\
 *    \text{f} = f(x) \\
 *    \text{df_dx} = \frac{\partial{f(x)}}{\partial{x}} \\
 *    \text{g} = g(x) \\
 *    \text{dg_dx} = \frac{\partial{g(x)}}{\partial{x}} \\
 * \f}
 */
struct AffineDynamicalModelEvaluationResult
{
  size_t nx = 0;  ///< Number of states
  size_t nu = 0;  ///< Number of inputs

  std::vector<scalar_t> f;  ///< Value of natural dynamics evaluation
                            /**< In column major order, size nx*1. */

  std::vector<scalar_t> df_dx;  ///< Partial derivative of natural dynamics w.r.t state
                                /**< In column major order, size nx*nx,
                                     empty if not available. */

  std::vector<scalar_t> g;  ///< Value of actuation dynamics evaluation
                            /**< In column major order, size nx*nu. */

  std::vector<scalar_t> dg_dx;  ///< Partial derivative of actuation dynamics w.r.t state
                                /**< In column major order, size nx*nu*nx,
                                     empty if not available. */

  AffineDynamicalModelEvaluationResult()                                             = default;
  AffineDynamicalModelEvaluationResult(const AffineDynamicalModelEvaluationResult &) = default;
  AffineDynamicalModelEvaluationResult(AffineDynamicalModelEvaluationResult &&)      = default;
  AffineDynamicalModelEvaluationResult & operator=(
    const AffineDynamicalModelEvaluationResult &) = default;
  AffineDynamicalModelEvaluationResult & operator=(
    AffineDynamicalModelEvaluationResult &&) = default;
  ~AffineDynamicalModelEvaluationResult()    = default;

  AffineDynamicalModelEvaluationResult(
    const size_t nx_, const size_t nu_, const bool with_gradient = false)
      : nx{nx_}, nu{nu_}, f(nx, scalar_t(0.)), g(nx * nu, scalar_t(0.))
  {
    if (with_gradient) {
      df_dx.assign(nx * nx, scalar_t(0.));
      dg_dx.assign(nx * nu * nx, scalar_t(0.));
    }
  }

  using SharedPtr = std::shared_ptr<AffineDynamicalModelEvaluationResult>;
};

/**
 * @brief Affine dynamical model interface
 * @details Abstract class for a generic affine non-linear dynamical model of the form
 * \f$ \dot{x} = f(x) + g(x)u \f$ with \f$ x \in \mathbb{R}^{\text{nx}} \f$ and \f$ u \in
 * \mathbb{R}^{\text{nu}} \f$.
 */
class AffineDynamicalModelAbstract : public DynamicalModelAbstract
{
public:
  /**
   * @brief Deleted default constructor
   * @details Use AffineDynamicalModelAbstract(const size_t, const
   * size_t) to construct an AffineDynamicalModelAbstract. This is done to guarantee
   * allocation on construction of heap allocated data members.
   */
  AffineDynamicalModelAbstract()                                                     = delete;
  AffineDynamicalModelAbstract(const AffineDynamicalModelAbstract &)                 = default;
  AffineDynamicalModelAbstract(AffineDynamicalModelAbstract &&) noexcept             = default;
  AffineDynamicalModelAbstract & operator=(const AffineDynamicalModelAbstract &)     = default;
  AffineDynamicalModelAbstract & operator=(AffineDynamicalModelAbstract &&) noexcept = default;
  virtual ~AffineDynamicalModelAbstract()                                            = default;

  /**
   * @brief Construct a new Affine Dynamical Model Abstract object
   *
   * @param nx Number of states
   * @param nu Number of inputs
   */
  AffineDynamicalModelAbstract(const size_t nx, const size_t nu);

  /**
   * @brief Evaluate the model dynamics for a given state value x and input value u.
   * @details Final implementation of DynamicalModelAbstract::evaluate function.
   *
   * @param x State at which to evaluate the dynamics (size must match result of nx())
   * @param u Input at which to evaluate the dynamics (size must match result of nu())
   */
  std::shared_ptr<DynamicalModelEvaluationResult> evaluate(
    const span<const scalar_t, dynamic_extent> x,
    const span<const scalar_t, dynamic_extent> u) const final;

  /**
   * @brief Evaluate the affine model dynamics for a given state value x.
   *
   * @param x State at which to evaluate the dynamics (size must match result of nx())
   */
  virtual std::shared_ptr<AffineDynamicalModelEvaluationResult> evaluate_affine(
    const span<const scalar_t, dynamic_extent> x) const = 0;

  // Functions of DynamicalModelAbstract left to implement:
  // virtual bool has_gradient() const { return false; }
  // virtual size_t nx() const = 0;
  // virtual size_t nu() const = 0;

  using SharedPtr = std::shared_ptr<AffineDynamicalModelAbstract>;

private:
  /**
   * @brief DynamicalModelEvaluationResult holder.
   * @details Allocated at construction and updated after each call to evaluate().
   */
  std::shared_ptr<DynamicalModelEvaluationResult> m_res;
};

}  // namespace lll

#endif  // THREELAWS_AFFINE_DYNAMICAL_MODEL_ABSTRACT__HPP
