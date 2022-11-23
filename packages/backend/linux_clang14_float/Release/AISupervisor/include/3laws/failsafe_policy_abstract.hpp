/**
 * @file failsafe_policy_abstract.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief Failsafe policy interface
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#ifndef THREELAWS_FAILSAFE_POLICY_ABSTRACT_HPP
#define THREELAWS_FAILSAFE_POLICY_ABSTRACT_HPP

#include <memory>
#include <vector>

#include <3laws/span.hpp>

#include "3laws/common.hpp"

namespace lll {

/**
 * @brief Failsafe policy evaluation result
 * @details Given a control policy \f$ u = k(x) \f$ with \f$ u \in \mathbb{R}^{\text{nu}} \f$
 * evaluated at \f$ x \in \mathbb{R}^{\text{nx}} \f$ then:
 * \f{array}{ \\
 *    \text{val} = k(x) \\
 *    \text{dval_dx} = \frac{\partial{k(x)}}{\partial{x}} \\
 * \f}
 */
struct FailsafePolicyEvaluationResult
{
  size_t nx = 0;  ///< Number of states
  size_t nu = 0;  ///< Number of inputs

  std::vector<scalar_t> val;  ///< Value of dynamics evaluation
                              /**< In column major order, size nu. */

  std::vector<scalar_t> dval_dx;  ///< Partial derivative w.r.t state
                                  /**< In column major order, size nu*nx,
                                        empty if not available. */

  FailsafePolicyEvaluationResult()                                                   = default;
  FailsafePolicyEvaluationResult(const FailsafePolicyEvaluationResult &)             = default;
  FailsafePolicyEvaluationResult(FailsafePolicyEvaluationResult &&)                  = default;
  FailsafePolicyEvaluationResult & operator=(const FailsafePolicyEvaluationResult &) = default;
  FailsafePolicyEvaluationResult & operator=(FailsafePolicyEvaluationResult &&)      = default;
  ~FailsafePolicyEvaluationResult()                                                  = default;

  FailsafePolicyEvaluationResult(
    const size_t nx_, const size_t nu_, const bool with_gradient = false)
      : nx{nx_}, nu{nu_}, val(nu, scalar_t(0.))
  {
    if (with_gradient) { dval_dx.assign(nu * nx, scalar_t(0.)); }
  }

  using SharedPtr = std::shared_ptr<FailsafePolicyEvaluationResult>;
};

/**
 * @brief Failsafe policy interface
 * @details Abstract class for generic feedback control policy of the form \f$ u = k(x) \f$
 * with \f$ x \in \mathbb{R}^{\text{nx}} \f$ and \f$ u \in \mathbb{R}^{\text{nu}} \f$.
 */
class FailsafePolicyAbstract
{
public:
  FailsafePolicyAbstract()                                               = default;
  FailsafePolicyAbstract(const FailsafePolicyAbstract &)                 = default;
  FailsafePolicyAbstract(FailsafePolicyAbstract &&) noexcept             = default;
  FailsafePolicyAbstract & operator=(const FailsafePolicyAbstract &)     = default;
  FailsafePolicyAbstract & operator=(FailsafePolicyAbstract &&) noexcept = default;
  virtual ~FailsafePolicyAbstract()                                      = default;

  /**
   * @brief Check if model evaluation result contains gradient information. False by default.
   */
  virtual bool has_gradient() const { return false; }

  /**
   * @brief Number of control states
   */
  virtual size_t nx() const = 0;

  /**
   * @brief Number of control inputs
   */
  virtual size_t nu() const = 0;

  /**
   * @brief Evaluate the failsafe policy for a given state value x
   *
   * @param x State at which to evaluate the policy (size must match result of nx())
   * @param t_nsec Time at which to evaluate the policy (in nanoseconds, default 0)
   */
  virtual std::shared_ptr<FailsafePolicyEvaluationResult> evaluate(
    const span<const scalar_t, dynamic_extent> x, const t_t t_nsec = 0) const = 0;

  using SharedPtr = std::shared_ptr<FailsafePolicyAbstract>;
};

}  // namespace lll

#endif  // THREELAWS_FAILSAFE_POLICY_ABSTRACT_HPP
