/**
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#include "3laws/kernel_generator_node.hpp"

#include <chrono>
#include <stdexcept>

#include <3laws/memory.hpp>

#include "3laws/msg_utils.hpp"
#include "external/parameters.hpp"

namespace lll {

KernelGeneratorNode::KernelGeneratorNode(const rclcpp::NodeOptions & options)
    : Node("KernelGeneratorNode", options)
{}

void KernelGeneratorNode::initialize(const KernelGeneratorNodeParams & prm,
  std::shared_ptr<AffineDynamicalModelAbstract> dyn,
  std::shared_ptr<SafetyMapAbstract> map,
  std::shared_ptr<FailsafePolicyAbstract> failsafe)
{
  initialize(prm, dyn, map, std::vector<std::shared_ptr<FailsafePolicyAbstract>>{failsafe});
}

void KernelGeneratorNode::initialize(const KernelGeneratorNodeParams & prm,
  std::shared_ptr<AffineDynamicalModelAbstract> dyn,
  std::shared_ptr<SafetyMapAbstract> map,
  std::vector<std::shared_ptr<FailsafePolicyAbstract>> failsafes)
{
  m_prm = prm;
  if (!dyn) {
    RCLCPP_FATAL(get_logger(), "Initialized with empty dynamics.");
    throw std::runtime_error("Initialized with empty dynamics.");
  }
  if (!map) {
    RCLCPP_FATAL(get_logger(), "Initialized with empty map.");
    throw std::runtime_error("Initialized with empty map.");
  }
  for (const auto & failsafe : failsafes) {
    if (!failsafe) {
      RCLCPP_FATAL(get_logger(), "Initialized with an empty failsafe.");
      throw std::runtime_error("Initialized with an empty failsafe.");
    }
  }
  m_dyn       = std::move(dyn);
  m_map       = std::move(map);
  m_failsafes = std::move(failsafes);

  if (m_prm.kernel_type == "explicit") {
    m_kernelGen = lll::make_unique<ExplicitKernelGenerator>(m_dyn, m_map, m_prm.explicit_params);
  } else {
    if (m_failsafes.empty()) {
      RCLCPP_FATAL(get_logger(), "Initialized with no failsafes.");
      throw std::runtime_error("Initialized with no failsafes.");
    }
    if (m_prm.kernel_type == "implicit") {
      m_kernelGen = lll::make_unique<ImplicitKernelGenerator>(
        m_dyn, m_map, m_failsafes[0], m_prm.implicit_params);
    } else if (m_prm.kernel_type == "switching") {
      m_kernelGen = lll::make_unique<SwitchingKernelGenerator>(
        m_dyn, m_map, m_failsafes, m_prm.switching_params);
    } else {
      RCLCPP_FATAL(
        get_logger(), "Unkown kernel type. Should be either explicit, implicit, or switching.");
      throw std::runtime_error("Unkown kernel type");
    }
  }

  n_pubReg =
    create_publisher<lll_msgs::msg::RegulationData>("regulation_data", rclcpp::SystemDefaultsQoS());

  const std::string stateTopicName = "state";
  const auto stateQoS              = rclcpp::SystemDefaultsQoS();
  if (m_prm.on_callback) {
    m_subState = create_subscription<lll_msgs::msg::Float64VectorStamped>(stateTopicName,
      stateQoS,
      [&](lll_msgs::msg::Float64VectorStamped::SharedPtr msg)
      {
        if (sub_state(std::move(msg))) { generate(); }
      });
  } else {
    m_subState = create_subscription<lll_msgs::msg::Float64VectorStamped>(stateTopicName,
      stateQoS,
      [&](lll_msgs::msg::Float64VectorStamped::SharedPtr msg) { sub_state(msg); });
  }

  if (m_prm.on_timer) {
    RCLCPP_INFO(get_logger(), "Kernel Generator ready");
    m_generationTimer = rclcpp::create_timer(
      this, get_clock(), std::chrono::microseconds(m_prm.timer_dt_usec), [this]() { generate(); });
  }

  RCLCPP_INFO(get_logger(), "Kernel Generator ready");
}

bool KernelGeneratorNode::sub_state(const lll_msgs::msg::Float64VectorStamped::SharedPtr & msg)
{
  RCLCPP_INFO(get_logger(), "Received state...");
  if (msg->data.size() != m_kernelGen->get_state().size()) {
    RCLCPP_ERROR(get_logger(),
      "Size of received state (%lu) inconsitent with expected size of state (%lu)",
      msg->data.size(),
      m_kernelGen->get_state().size());
    return false;
  }

  m_kernelGen->set_state(msg->data);
  return true;
}

void KernelGeneratorNode::generate() const
{
  RCLCPP_INFO(get_logger(), "Generating regulation data...");
  if (!m_kernelGen->ready_to_generate()) {
    RCLCPP_WARN(get_logger(), "Kernel generator not ready");
    return;
  }
  const auto regData       = m_kernelGen->generate();
  auto regDataMsg          = lll::make_unique<lll_msgs::msg::RegulationData>();
  *regDataMsg              = to_msg(*regData);
  regDataMsg->header.stamp = now();
  n_pubReg->publish(std::move(regDataMsg));
}

}  // namespace lll
