/**
 * @file input_filter_abstract.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief Input filter interface
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#ifndef THREELAWS_INPUT_FILTER_ABSTRACT_HPP
#define THREELAWS_INPUT_FILTER_ABSTRACT_HPP

#include <chrono>
#include <memory>
#include <vector>

#include <3laws/span.hpp>

#include "3laws/common.hpp"
#include "3laws/regulation_data.hpp"

namespace lll {

/**
 * @brief Input filter evaluation result
 * @details Given an input filtering \f$ u_{\text{filt}} = \text{filter}(u_{\text{des}},
 * u_{\text{fail}}) \f$ then:
 * \f{array}{ \\
 *    \text{input_desired} =  u_{\text{act}} \\
 *    \text{input_filtered} =  u_{\text{filt}} \\
 *    \text{input_failsafe} =  u_{\text{fail}} \\
 * \f}
 */
struct InputFilteringResult
{
  std::size_t nu      = 0;               ///< Number of inputs
  int32_t return_code = 0;               ///< Filtering return code
  std::vector<scalar_t> input_desired;   ///< Desired input
                                         /**< Size nu. */
  std::vector<scalar_t> input_filtered;  ///< Filtered input
                                         /**< Size nu. */
  std::vector<scalar_t> input_failsafe;  ///< Failsafe input
                                         /**< Size nu, empty if not available. */

  // Return code values
  static constexpr int32_t RC_OK = 0;
};

/**
 * @brief Input filter interface
 * @details Abstract class for generic input filter of the form \f$ u_{\text{filt}} =
 * \text{filter}(u_{\text{des}}, \text{RegulationData}) \f$.
 * Users can set a desired input and some regulation data, after which the ready_to_filter() method
 * should return true and they shoud be able to call the filter() method.
 */
class InputFilterAbstract
{
public:
  InputFilterAbstract()                                            = default;
  InputFilterAbstract(const InputFilterAbstract &)                 = default;
  InputFilterAbstract(InputFilterAbstract &&) noexcept             = default;
  InputFilterAbstract & operator=(const InputFilterAbstract &)     = default;
  InputFilterAbstract & operator=(InputFilterAbstract &&) noexcept = default;
  virtual ~InputFilterAbstract()                                   = default;

  /**
   * @brief Set the desired input
   *
   * @param dt Duration since last call to set_input_desired()
   * @param u Desired input
   */
  virtual void set_input_desired(
    const std::chrono::duration<scalar_t> dt, const span<const scalar_t, dynamic_extent> u) = 0;

  /**
   * @brief Set the regulation data
   *
   * @param dt Duration since last call to set_regulation()
   * @param regulationData Regulation data
   */
  virtual void set_regulation(const std::chrono::duration<scalar_t> dt,
    const std::shared_ptr<RegulationData> & regulationData) = 0;

  /**
   * @brief Check if filter is ready to be used
   */
  virtual bool ready_to_filter() const = 0;

  /**
   * @brief Filter desired input given regulation data
   */
  virtual std::shared_ptr<InputFilteringResult> filter() = 0;
};

}  // namespace lll

#endif  // THREELAWS_INPUT_FILTER_ABSTRACT_HPP
