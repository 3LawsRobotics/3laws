/**
 * @file explicit_kernel_generator.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief Explicit kernel generator
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#ifndef THREELAWS_EXPLICIT_KERNEL_GENERATOR_HPP
#define THREELAWS_EXPLICIT_KERNEL_GENERATOR_HPP

#include <memory>
#include <utility>

#include "3laws/affine_dynamical_model_abstract.hpp"
#include "3laws/common.hpp"
#include "3laws/kernel_generator_abstract.hpp"
#include "3laws/regulation_data.hpp"

namespace lll {

/**
 * @brief ExplicitKernelGenerator parameters
 */
struct ExplicitKernelGeneratorParams
{
  InputConstraints input_cstr;  ///< Input constraints
};

/**
 * @brief  Explicit kernel generator
 * @details Implementation of an KernelGeneratorAbstract.
 *
 *\attention Provided map must provide gradient information and describe a control invariant set for
 *to be a valid input.
 *
 * This kernel generator can only be used in conjunction with a QpInputFilter.
 */
class ExplicitKernelGenerator : public KernelGeneratorAbstract
{
public:
  ExplicitKernelGenerator()                                                = delete;
  ExplicitKernelGenerator(const ExplicitKernelGenerator &)                 = delete;
  ExplicitKernelGenerator(ExplicitKernelGenerator &&) noexcept             = default;
  ExplicitKernelGenerator & operator=(const ExplicitKernelGenerator &)     = delete;
  ExplicitKernelGenerator & operator=(ExplicitKernelGenerator &&) noexcept = default;
  ~ExplicitKernelGenerator() override;

  ExplicitKernelGenerator(std::shared_ptr<AffineDynamicalModelAbstract> dyn,
    std::shared_ptr<SafetyMapAbstract> map,
    const ExplicitKernelGeneratorParams & params);

  std::shared_ptr<RegulationData> generate() override;

private:
  std::unique_ptr<struct ExplicitKernelGeneratorImpl> m_pImpl;  ///< Pointer to implementation
};

}  // namespace lll

#endif  // THREELAWS_EXPLICIT_KERNEL_GENERATOR_HPP
