/**
 * @file qp_input_filter.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief Quadratic Program based input filter
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#ifndef THREELAWS_QP_INPUT_FILTER_HPP
#define THREELAWS_QP_INPUT_FILTER_HPP

#include <chrono>
#include <memory>

#include "3laws/common.hpp"
#include "3laws/input_filter_abstract.hpp"
#include "3laws/regulation_data.hpp"
#include <3laws/optional.hpp>

namespace lll {

/**
 * @brief QpInputFilter parameters
 */
struct QpInputFilterParams
{
  /**
   * @brief QPSolver parameters
   * @details
   *
   */
  struct SolverParams
  {
    bool verbose                        = false;  ///< Print solver info to stdout
    float alpha                         = 1.6f;   ///< Relaxation parameter
    float rho                           = 0.1f;   ///< First dual step size
    float sigma                         = 1e-6f;  ///< Second dual step length
    bool scaling                        = true;   ///< Scale problem
    float eps_abs                       = 1e-3f;  ///< Absolute threshold for convergence
    float eps_rel                       = 1e-3f;  ///< Relative threshold for convergence
    float eps_primal_inf                = 1e-4f;  ///< Threshold for primal infeasibility
    float eps_dual_inf                  = 1e-4f;  ///< Threshold for dual infeasibility
    lll::optional<std::size_t> max_iter = {};     ///< Max number of iterations (default no limit)
    lll::optional<std::chrono::nanoseconds> max_time = {};  ///< Max solving time (default no limit)
    std::size_t stop_check_iter = 25;     ///< Iterations between checking stopping criterion
    bool polish                 = true;   ///< Run solution polishing (uses dynamic memory)
    std::size_t polish_iter     = 5;      ///< Number of iterations to refine polish
    float delta                 = 1e-6f;  ///< Regularization parameter for polishing
  };

  SolverParams qp_params;           ///< QPSolver parameters
  std::size_t nu           = 0;     ///< Number of inputs (must be > 0)
  std::size_t n_safetyCstr = 0;     ///< Number of safety constraints
  std::size_t n_failsafes  = 0;     ///< Number of failsafe policies
  std::size_t n_inputCstr  = 0;     ///< Number of input constraints
  float alpha              = 1.;    ///< Safety constraints relaxation (must be >= 0)
  float relax_cost         = 100.;  ///< Safety constraints relaxation penalty (must be >= 0)
};

/**
 * @brief Quadratic Program based input filter
 * @details Implementation of an InputFilterAbstract. In particular:
\f{split}{
u_{\text{filt}}(x)=\underset{\substack{u\in U \\ \varepsilon \geq 0
}}{\text{argmin}} & \ \left\Vert u_{\text{des}}-u\right\Vert ^{2} +
\text{params.relax_cost} \ \varepsilon^2\\
\text{s.t.} & \ L_{f}h_{i}(x)+L_{g}h_{i}(x)u+\alpha h_{i}(x)\geq0 \\
 & \ \forall i \in \{1, \ldots, \text{params.n_safetyCstr}\}
\f} with \f$ U \f$ being the set described by regulationData.input_cstr.

This filter accepts regulation data from ExplicitKernelGenerator and ImplicitKernelGenerator.
 */
class QpInputFilter : public InputFilterAbstract
{
public:
  QpInputFilter()                                      = delete;
  QpInputFilter(const QpInputFilter &)                 = delete;
  QpInputFilter(QpInputFilter &&) noexcept             = default;
  QpInputFilter & operator=(const QpInputFilter &)     = delete;
  QpInputFilter & operator=(QpInputFilter &&) noexcept = default;
  ~QpInputFilter() override;
  explicit QpInputFilter(const QpInputFilterParams & params);

  bool ready_to_filter() const override;
  void set_input_desired(const std::chrono::duration<scalar_t> dt,
    const span<const scalar_t, dynamic_extent> u) override;
  void set_regulation(const std::chrono::duration<scalar_t> dt,
    const std::shared_ptr<RegulationData> & regulationData) override;
  std::shared_ptr<InputFilteringResult> filter() override;

private:
  std::unique_ptr<struct QpInputFilterImpl> m_pImpl;  ///< Pointer to implementation
};

}  // namespace lll

#endif  // THREELAWS_QP_INPUT_FILTER_HPP
