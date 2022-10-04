/**
 * @file kernel_generator_abstract.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief Kernel generator interface
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#ifndef THREELAWS_KERNEL_GENERATOR_ABSTRACT_HPP
#define THREELAWS_KERNEL_GENERATOR_ABSTRACT_HPP

#include <memory>
#include <vector>

#include "3laws/common.hpp"
#include "3laws/dynamical_model_abstract.hpp"
#include "3laws/regulation_data.hpp"
#include "3laws/safety_map_abstract.hpp"

namespace lll {

/**
 * @brief Kernel generator interface
 * @details Abstract class for generic kernel generator. A kernel generator gets constructed with
 * some dynamics and a map. Users can set the current state, after which the ready_to_generate()
 * method should return true and they should be able to call the generate() method.
 */
class KernelGeneratorAbstract
{
public:
  /**
   * @brief Deleted default constructor
   * @details Use KernelGeneratorAbstract(const std::shared_ptr<DynamicalModelAbstract> &, const
   * std::shared_ptr<SafetyMapAbstract> &) to construct a KernelGeneratorAbstract object. This is
   * done to guarantee allocation on construction of heap allocated data members.
   */
  KernelGeneratorAbstract()                                                = delete;
  KernelGeneratorAbstract(const KernelGeneratorAbstract &)                 = default;
  KernelGeneratorAbstract(KernelGeneratorAbstract &&) noexcept             = default;
  KernelGeneratorAbstract & operator=(const KernelGeneratorAbstract &)     = default;
  KernelGeneratorAbstract & operator=(KernelGeneratorAbstract &&) noexcept = default;
  virtual ~KernelGeneratorAbstract()                                       = default;

  /**
   * @brief Construct a new Kernel Generator Abstract object
   *
   * @param dyn Dynamical model
   * @param map Safety map
   */
  KernelGeneratorAbstract(const std::shared_ptr<DynamicalModelAbstract> & dyn,
    const std::shared_ptr<SafetyMapAbstract> & map);

  /**
   * @brief Set the current state
   *
   * @param x Current state
   */
  virtual void set_state(const span<const scalar_t, dynamic_extent> x);

  /**
   * @brief Get the current state
   */
  virtual const std::vector<scalar_t> & get_state() const;

  /**
   * @brief Check if kernel generator is ready to be used. By default, returns true after
   * set_state() has been called once.
   */
  virtual bool ready_to_generate() const;

  /**
   * @brief Generate regulation data given current map, dynamics and state
   */
  virtual std::shared_ptr<RegulationData> generate() = 0;

protected:
  bool m_stateInit = false;       ///< State initialized flag
  std::vector<scalar_t> m_state;  ///< Current state holder
};

}  // namespace lll

#endif  // THREELAWS_KERNEL_GENERATOR_ABSTRACT_HPP
