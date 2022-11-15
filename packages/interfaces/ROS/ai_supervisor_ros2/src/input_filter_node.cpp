/**
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#include "3laws/input_filter_node.hpp"

#include <3laws/memory.hpp>

#include "3laws/msg_utils.hpp"
#include "external/parameters.hpp"

namespace lll {

InputFilterNode::InputFilterNode(const rclcpp::NodeOptions & options)
    : Node("InputFilterNode", options)
{}

void InputFilterNode::init()
{
  cbr::initParams(*this, "", m_prm);

  if (m_prm.filter_type == "qp") {
    m_filter = lll::make_unique<QpInputFilter>(m_prm.qp_params);
  } else if (m_prm.filter_type == "switching") {
    m_filter = lll::make_unique<SwitchingInputFilter>(m_prm.switching_params);
  } else {
    RCLCPP_FATAL(get_logger(), "Unkown filter type. Should be either qp, or switching.");
    throw std::runtime_error("Unkown filter type");
  }

  n_pubFilterRes = create_publisher<lll_msgs::msg::InputFilteringResult>(
    "input_filtered", rclcpp::SystemDefaultsQoS());

  // Input desired subscriber
  const std::string inputDesTopicName = "input_desired";
  const auto inputDesQoS              = rclcpp::SystemDefaultsQoS();
  if (m_prm.on_input_callback) {
    m_subInputDes = create_subscription<lll_msgs::msg::Float64VectorStamped>(inputDesTopicName,
      inputDesQoS,
      [&](lll_msgs::msg::Float64VectorStamped::SharedPtr msg)
      {
        sub_input_des(msg);
        filter();
      });
  } else {
    m_subInputDes = create_subscription<lll_msgs::msg::Float64VectorStamped>(inputDesTopicName,
      inputDesQoS,
      [&](lll_msgs::msg::Float64VectorStamped::SharedPtr msg) { sub_input_des(msg); });
  }

  // Regulation data subscriber
  const std::string regDataTopicName = "regulation_data";
  const auto regDataQoS              = rclcpp::SystemDefaultsQoS();
  if (m_prm.on_regulation_data_callback) {
    m_subRegData = create_subscription<lll_msgs::msg::RegulationData>(regDataTopicName,
      regDataQoS,
      [&](lll_msgs::msg::RegulationData::SharedPtr msg)
      {
        sub_regulation_data(msg);
        filter();
      });
  } else {
    m_subRegData = create_subscription<lll_msgs::msg::RegulationData>(regDataTopicName,
      regDataQoS,
      [&](lll_msgs::msg::RegulationData::SharedPtr msg) { sub_regulation_data(msg); });
  }

  // Input filtering service
  m_srvFiltering = create_service<lll_msgs::srv::InputFiltering>("input_filtering",
    [this](const lll_msgs::srv::InputFiltering::Request::SharedPtr & req,
      const lll_msgs::srv::InputFiltering::Response::SharedPtr & resp)
    { srv_filtering(req, resp); });

  RCLCPP_INFO(get_logger(), "Input Filter ready");
}

void InputFilterNode::filter() const
{
  RCLCPP_INFO(get_logger(), "Filtering...");
  if (!m_filter->ready_to_filter()) {
    RCLCPP_WARN(get_logger(), "Input filter not ready");
    return;
  }
  const auto filterResult       = m_filter->filter();
  auto filterResultMsg          = lll::make_unique<lll_msgs::msg::InputFilteringResult>();
  *filterResultMsg              = to_msg(*filterResult);
  filterResultMsg->header.stamp = now();
  n_pubFilterRes->publish(std::move(filterResultMsg));

  if (filterResult->return_code != filterResult->RC_OK) {
    RCLCPP_WARN(get_logger(), "Filtering failed, return code: %i", filterResult->return_code);
  }
}

void InputFilterNode::sub_input_des(const lll_msgs::msg::Float64VectorStamped::SharedPtr & msg)
{
  RCLCPP_INFO(get_logger(), "Receiving input...");
  m_filter->set_input_desired(from_msg(msg->header.stamp), {msg->data.data(), msg->data.size()});
}

void InputFilterNode::sub_regulation_data(const lll_msgs::msg::RegulationData::SharedPtr & msg)
{
  RCLCPP_INFO(get_logger(), "Receiving regulation data...");
  m_filter->set_regulation(from_msg(msg->header.stamp), from_msg(*msg));
}
void InputFilterNode::srv_filtering(const lll_msgs::srv::InputFiltering::Request::SharedPtr & req,
  const lll_msgs::srv::InputFiltering::Response::SharedPtr & resp)
{
  RCLCPP_INFO(get_logger(), "Input filtering service called...");
  m_filter->set_input_desired(from_msg(req->input_desired.header.stamp),
    {req->input_desired.data.data(), req->input_desired.data.size()});

  if (!m_filter->ready_to_filter()) {
    RCLCPP_WARN(get_logger(), "Input filter not ready");
    resp->ready = false;
    return;
  }

  resp->ready             = true;
  const auto filterResult = m_filter->filter();
  resp->res               = to_msg(*filterResult);
  resp->res.header.stamp  = now();

  if (filterResult->return_code != filterResult->RC_OK) {
    RCLCPP_WARN(get_logger(), "Filtering failed, return code: %i", filterResult->return_code);
  }
}

}  // namespace lll
