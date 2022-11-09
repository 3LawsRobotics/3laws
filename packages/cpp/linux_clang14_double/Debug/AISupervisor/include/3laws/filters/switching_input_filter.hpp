/**
 * @file switching_input_filter.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief Policy switching based input filter
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#ifndef THREELAWS_SWITCHING_INPUT_FILTER_HPP
#define THREELAWS_SWITCHING_INPUT_FILTER_HPP

#include <memory>
#include <vector>

#include "3laws/common.hpp"
#include "3laws/config.hpp"
#include "3laws/input_filter_abstract.hpp"
#include "3laws/regulation_data.hpp"

namespace lll {

/**
 * @brief SwitchingInputFilter parameters
 */
struct SwitchingInputFilterParams
{
  std::size_t nu           = 0;              ///< Number of inputs (must be > 0)
  std::size_t n_safetyCstr = 0;              ///< Number of safety constraints
  std::size_t n_failsafes  = 0;              ///< Number of failsafe policies
  std::size_t n_inputCstr  = 0;              ///< Number of input constraints
  scalar_t gain_d          = scalar_t(1.0);  ///< Derivative switching gain
  scalar_t filter_tau      = scalar_t(1.0);  ///< Derivative filter time constant
  scalar_t expo            = scalar_t(6.0);  ///< Exponent of switching function
};

/**
 * @brief  Policy switching based input filter
 * @details Implementation of an InputFilterAbstract. In particular:
 * \f[
 * u_{\text{filt}}(x)=\lambda u_\text{fail} + (1-\lambda) u_\text{des}
 * \f] with \f$ \lambda = \sigma_0^1\left(\lambda_p + (1-\lambda_p)\lambda_d\right) \f$
 * where \f$ \sigma_0^1 \f$ is a smooth saturation function between 0 and 1, \f$ \lambda_p =
 * (1-h_\text{min})^\text{params.expo}\f$, and \f$ \lambda_d =
 * \text{first_order_filter}\left(-\text{params.gain_d}\frac{d
 * h_\text{min}}{dt},\text{params.filter_tau}\right)\f$.
 *
 * This filter accepts regulation data from ImplicitKernelGenerator and SwitchingKernelGenerator.
 */
class SwitchingInputFilter : public InputFilterAbstract
{
public:
  SwitchingInputFilter()                                             = delete;
  SwitchingInputFilter(const SwitchingInputFilter &)                 = delete;
  SwitchingInputFilter(SwitchingInputFilter &&) noexcept             = default;
  SwitchingInputFilter & operator=(const SwitchingInputFilter &)     = delete;
  SwitchingInputFilter & operator=(SwitchingInputFilter &&) noexcept = default;
  ~SwitchingInputFilter() override;

  explicit SwitchingInputFilter(const SwitchingInputFilterParams & params);

  bool ready_to_filter() const override;
  void set_input_desired(const t_t t_nsec, const span<const scalar_t, dynamic_extent> u) override;
  void set_regulation(const t_t t_nsec, const RegulationData & regulationData) override;
  std::shared_ptr<InputFilteringResult> filter() override;

private:
  std::unique_ptr<struct SwitchingInputFilterImpl> m_pImpl;  ///< Pointer to implementation
};

}  // namespace lll

#endif  // THREELAWS_SWITCHING_INPUT_FILTER_HPP
