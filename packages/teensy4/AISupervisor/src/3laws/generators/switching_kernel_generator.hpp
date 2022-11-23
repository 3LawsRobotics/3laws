/**
 * @file switching_kernel_generator.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief Switching kernel generator
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#ifndef THREELAWS_SWITCHING_KERNEL_GENERATOR_HPP
#define THREELAWS_SWITCHING_KERNEL_GENERATOR_HPP

#include <memory>
#include <utility>
#include <vector>

#include "3laws/common.hpp"
#include "3laws/failsafe_policy_abstract.hpp"
#include "3laws/kernel_generator_abstract.hpp"

namespace lll {

/**
 * @brief SwitchingKernelGenerator parameters
 */
struct SwitchingKernelGeneratorParams
{
  scalar_t T  = scalar_t(1.);    ///< Time horizon
  size_t K    = 10;              ///< Number of uniformly distributed constraint instances
  scalar_t dt = scalar_t(0.01);  ///< Maximal integration time step
  InputConstraints input_cstr;   ///< Input constraints
};

/**
 * @brief  Switching kernel generator
 * @details Implementation of an KernelGeneratorAbstract.
 *
 * Multiple failsafe policies can be supplied to this kernel generator.
 *
 * This kernel generator can only be used in conjunction with a SwitchingInputFilter.
 */
class SwitchingKernelGenerator : public KernelGeneratorAbstract
{
public:
  SwitchingKernelGenerator()                                                 = delete;
  SwitchingKernelGenerator(const SwitchingKernelGenerator &)                 = delete;
  SwitchingKernelGenerator(SwitchingKernelGenerator &&) noexcept             = default;
  SwitchingKernelGenerator & operator=(const SwitchingKernelGenerator &)     = delete;
  SwitchingKernelGenerator & operator=(SwitchingKernelGenerator &&) noexcept = default;
  ~SwitchingKernelGenerator() override;

  SwitchingKernelGenerator(std::shared_ptr<DynamicalModelAbstract> dyn,
    std::shared_ptr<SafetyMapAbstract> map,
    std::vector<std::shared_ptr<FailsafePolicyAbstract>> failsafes,
    const SwitchingKernelGeneratorParams & params);

  SwitchingKernelGenerator(std::shared_ptr<DynamicalModelAbstract> dyn,
    std::shared_ptr<SafetyMapAbstract> map,
    std::shared_ptr<FailsafePolicyAbstract> failsafe,
    const SwitchingKernelGeneratorParams & params);

  std::shared_ptr<RegulationData> generate() override;

private:
  std::unique_ptr<struct SwitchingKernelGeneratorImpl> m_pImpl;  ///< Pointer to implementation
};

}  // namespace lll

#endif  // THREELAWS_SWITCHING_KERNEL_GENERATOR_HPP
