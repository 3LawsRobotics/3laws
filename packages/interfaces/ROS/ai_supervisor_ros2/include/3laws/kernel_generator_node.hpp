/**
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */
#ifndef THREELAWS_KERNEL_GENERATOR_NODE_HPP
#define THREELAWS_KERNEL_GENERATOR_NODE_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <lll_msgs/msg/float64_vector_stamped.hpp>
#include <lll_msgs/msg/regulation_data.hpp>

#include <3laws/affine_dynamical_model_abstract.hpp>
#include <3laws/generators/explicit_kernel_generator.hpp>
#include <3laws/generators/implicit_kernel_generator.hpp>
#include <3laws/generators/switching_kernel_generator.hpp>
#include <3laws/safety_map_abstract.hpp>

#include <boost/hana/adapt_struct.hpp>

namespace lll {

struct KernelGeneratorNodeParams
{
  std::string kernel_type = "explicit";
  ExplicitKernelGeneratorParams explicit_params;
  ImplicitKernelGeneratorParams implicit_params;
  SwitchingKernelGeneratorParams switching_params;
  bool on_callback       = true;
  bool on_timer          = false;
  uint32_t timer_dt_usec = 10000;
};

// Component
class KernelGeneratorNode : public rclcpp::Node
{
public:
  explicit KernelGeneratorNode(const rclcpp::NodeOptions & options);

  void initialize(const KernelGeneratorNodeParams & prm,
    std::shared_ptr<AffineDynamicalModelAbstract> dyn,
    std::shared_ptr<SafetyMapAbstract> map,
    std::shared_ptr<FailsafePolicyAbstract> failsafe);
  void initialize(const KernelGeneratorNodeParams & prm,
    std::shared_ptr<AffineDynamicalModelAbstract> dyn,
    std::shared_ptr<SafetyMapAbstract> map,
    std::vector<std::shared_ptr<FailsafePolicyAbstract>> failsafes = {});

private:
  KernelGeneratorNodeParams m_prm;

  rclcpp::Publisher<lll_msgs::msg::RegulationData>::SharedPtr n_pubReg;
  rclcpp::Subscription<lll_msgs::msg::Float64VectorStamped>::SharedPtr m_subState;
  rclcpp::TimerBase::SharedPtr m_generationTimer;

  std::shared_ptr<AffineDynamicalModelAbstract> m_dyn;
  std::shared_ptr<SafetyMapAbstract> m_map;
  std::vector<std::shared_ptr<FailsafePolicyAbstract>> m_failsafes;
  std::unique_ptr<KernelGeneratorAbstract> m_kernelGen;

  bool sub_state(const lll_msgs::msg::Float64VectorStamped::SharedPtr & msg);
  void generate() const;
};

}  // namespace lll

BOOST_HANA_ADAPT_STRUCT(lll::InputConstraints, nu, n_cstr, lb, M, ub);
BOOST_HANA_ADAPT_STRUCT(lll::ExplicitKernelGeneratorParams, input_cstr);
BOOST_HANA_ADAPT_STRUCT(lll::ImplicitKernelGeneratorParams, T, K, dt, input_cstr);
BOOST_HANA_ADAPT_STRUCT(lll::SwitchingKernelGeneratorParams, T, K, dt, input_cstr);
BOOST_HANA_ADAPT_STRUCT(lll::KernelGeneratorNodeParams,
  kernel_type,
  explicit_params,
  implicit_params,
  switching_params,
  on_callback,
  on_timer,
  timer_dt_usec);

#endif  // THREELAWS_KERNEL_GENERATOR_NODE_HPP
