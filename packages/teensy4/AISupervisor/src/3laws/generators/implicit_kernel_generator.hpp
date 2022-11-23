/**
 * @file implicit_kernel_generator.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief Implicit kernel generator
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#ifndef THREELAWS_IMPLICIT_KERNEL_GENERATOR_HPP
#define THREELAWS_IMPLICIT_KERNEL_GENERATOR_HPP

#include <memory>
#include <utility>
#include <vector>

#include "3laws/affine_dynamical_model_abstract.hpp"
#include "3laws/common.hpp"
#include "3laws/failsafe_policy_abstract.hpp"
#include "3laws/kernel_generator_abstract.hpp"

namespace lll {

/**
 * @brief ImplicitKernelGenerator parameters
 */
struct ImplicitKernelGeneratorParams
{
  scalar_t T  = scalar_t(1.);    ///< Time horizon
  size_t K    = 10;              ///< Number of uniformly distributed constraint instances
  scalar_t dt = scalar_t(0.01);  ///< Maximal integration time step
  InputConstraints input_cstr;   ///< Input constraints
};

/**
 * @brief  Implicit kernel generator
 * @details Implementation of an KernelGeneratorAbstract.
 *
 *\attention Provided map, dynamical model, and failsafe must provide gradient information.
 *
 * This kernel generator can be used in conjunction with a QpInputFilter or a SwitchingInputFilter.
 */
class ImplicitKernelGenerator : public KernelGeneratorAbstract
{
public:
  ImplicitKernelGenerator()                                                = delete;
  ImplicitKernelGenerator(const ImplicitKernelGenerator &)                 = delete;
  ImplicitKernelGenerator(ImplicitKernelGenerator &&) noexcept             = default;
  ImplicitKernelGenerator & operator=(const ImplicitKernelGenerator &)     = delete;
  ImplicitKernelGenerator & operator=(ImplicitKernelGenerator &&) noexcept = default;
  ~ImplicitKernelGenerator() override;

  ImplicitKernelGenerator(std::shared_ptr<AffineDynamicalModelAbstract> dyn,
    std::shared_ptr<SafetyMapAbstract> map,
    std::shared_ptr<FailsafePolicyAbstract> failsafe,
    const ImplicitKernelGeneratorParams & params);

  std::shared_ptr<RegulationData> generate() override;

private:
  std::unique_ptr<struct ImplicitKernelGeneratorImpl> m_pImpl;  ///< Pointer to implementation
};

}  // namespace lll

#endif  // THREELAWS_IMPLICIT_KERNEL_GENERATOR_HPP
