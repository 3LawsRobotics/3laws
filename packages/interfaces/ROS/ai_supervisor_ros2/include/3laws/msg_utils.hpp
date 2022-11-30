/**
 * @file msg_utils.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */
#ifndef THREELAWS_MSG_UTILS_HPP
#define THREELAWS_MSG_UTILS_HPP

#include <rclcpp/timer.hpp>

#include <3laws/input_filter_abstract.hpp>
#include <3laws/regulation_data.hpp>

#include <lll_msgs/msg/input_filtering_result.hpp>
#include <lll_msgs/msg/regulation_data.hpp>

namespace lll {

inline t_t from_msg(const builtin_interfaces::msg::Time & in)
{
  return static_cast<t_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<decltype(in.sec)>(in.sec)
    + std::chrono::duration<decltype(in.nanosec), std::nano>(in.nanosec))
                            .count());
}

inline InputConstraints from_msg(const lll_msgs::msg::InputConstraints & in)
{
  InputConstraints out;
  out.nu     = in.nu;
  out.n_cstr = in.n_cstr;
  out.lb     = in.lb;
  out.M      = in.m;
  out.ub     = in.ub;
  return out;
}

inline RegulationData from_msg(const lll_msgs::msg::RegulationData & in)
{
  RegulationData out;
  out.nu             = in.nu;
  out.n_safetyCstr   = in.n_safety_cstr;
  out.n_failsafes    = in.n_failsafes;
  out.input_cstr     = from_msg(in.input_cstr);
  out.lfh            = in.lfh;
  out.lgh            = in.lgh;
  out.safety_val     = in.safety_val;
  out.failsafe_input = in.failsafe_input;
  return out;
}

inline InputFilteringResult from_msg(const lll_msgs::msg::InputFilteringResult & in)
{
  InputFilteringResult out;
  out.nu             = in.nu;
  out.return_code    = in.return_code;
  out.input_desired  = in.input_desired;
  out.input_filtered = in.input_filtered;
  out.input_failsafe = in.input_failsafe;
  return out;
}

inline builtin_interfaces::msg::Time to_msg(const t_t & in)
{
  const auto tIn = std::chrono::duration<t_t, std::nano>(in);
  builtin_interfaces::msg::Time out;
  auto t_sec  = std::chrono::duration_cast<std::chrono::duration<decltype(out.sec)>>(tIn);
  out.sec     = t_sec.count();
  out.nanosec = std::chrono::duration<decltype(out.nanosec), std::nano>(tIn - t_sec).count();
  return out;
}

inline lll_msgs::msg::InputConstraints to_msg(const InputConstraints & in)
{
  lll_msgs::msg::InputConstraints out;
  out.nu     = in.nu;
  out.n_cstr = in.n_cstr;
  out.lb     = in.lb;
  out.m      = in.M;
  out.ub     = in.ub;
  return out;
}

inline lll_msgs::msg::RegulationData to_msg(const RegulationData & in)
{
  lll_msgs::msg::RegulationData out;
  out.nu             = in.nu;
  out.n_safety_cstr  = in.n_safetyCstr;
  out.n_failsafes    = in.n_failsafes;
  out.input_cstr     = to_msg(in.input_cstr);
  out.lfh            = in.lfh;
  out.lgh            = in.lgh;
  out.safety_val     = in.safety_val;
  out.failsafe_input = in.failsafe_input;
  return out;
}

inline lll_msgs::msg::InputFilteringResult to_msg(const InputFilteringResult & in)
{
  lll_msgs::msg::InputFilteringResult out;
  out.nu             = in.nu;
  out.return_code    = in.return_code;
  out.input_desired  = in.input_desired;
  out.input_filtered = in.input_filtered;
  out.input_failsafe = in.input_failsafe;
  return out;
}

}  // namespace lll
#endif  // THREELAWS_MSG_UTILS_HPP
