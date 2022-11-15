/**
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */
#ifndef THREELAWS_INPUT_FILTER_NODE_HPP
#define THREELAWS_INPUT_FILTER_NODE_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <lll_msgs/msg/float64_vector_stamped.hpp>
#include <lll_msgs/msg/input_filtering_result.hpp>
#include <lll_msgs/msg/regulation_data.hpp>
#include <lll_msgs/srv/input_filtering.hpp>

#include <3laws/filters/qp_input_filter.hpp>
#include <3laws/filters/switching_input_filter.hpp>

#include <boost/hana/adapt_struct.hpp>

namespace lll {

struct InputFilterNodeParams
{
  std::string filter_type = "qp";
  QpInputFilterParams qp_params;
  SwitchingInputFilterParams switching_params;
  bool on_input_callback           = true;
  bool on_regulation_data_callback = true;
};

// Component
class InputFilterNode : public rclcpp::Node
{
public:
  explicit InputFilterNode(const rclcpp::NodeOptions & options);

  void init();

private:
  InputFilterNodeParams m_prm;

  rclcpp::Publisher<lll_msgs::msg::InputFilteringResult>::SharedPtr n_pubFilterRes;
  rclcpp::Subscription<lll_msgs::msg::Float64VectorStamped>::SharedPtr m_subInputDes;
  rclcpp::Subscription<lll_msgs::msg::RegulationData>::SharedPtr m_subRegData;
  rclcpp::Service<lll_msgs::srv::InputFiltering>::SharedPtr m_srvFiltering;

  std::unique_ptr<InputFilterAbstract> m_filter;

  void sub_input_des(const lll_msgs::msg::Float64VectorStamped::SharedPtr & msg);
  void sub_regulation_data(const lll_msgs::msg::RegulationData::SharedPtr & msg);
  void srv_filtering(const lll_msgs::srv::InputFiltering::Request::SharedPtr & req,
    const lll_msgs::srv::InputFiltering::Response::SharedPtr & resp);
  void filter() const;
};

}  // namespace lll

BOOST_HANA_ADAPT_STRUCT(lll::QpInputFilterParams::SolverParams,
  verbose,
  alpha,
  rho,
  sigma,
  scaling,
  eps_abs,
  eps_rel,
  eps_primal_inf,
  eps_dual_inf,
  max_iter,
  max_time,
  stop_check_iter,
  polish,
  polish_iter,
  delta);

BOOST_HANA_ADAPT_STRUCT(lll::QpInputFilterParams,
  qp_params,
  nu,
  n_safetyCstr,
  n_failsafes,
  n_inputCstr,
  alpha,
  relax_cost);

BOOST_HANA_ADAPT_STRUCT(lll::SwitchingInputFilterParams,
  nu,
  n_safetyCstr,
  n_failsafes,
  n_inputCstr,
  gain_d,
  filter_tau,
  expo);

BOOST_HANA_ADAPT_STRUCT(lll::InputFilterNodeParams,
  filter_type,
  qp_params,
  switching_params,
  on_input_callback,
  on_regulation_data_callback);

#endif  // THREELAWS_INPUT_FILTER_NODE_HPP
