/**
 * @file start_node.hpp
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */
#ifndef THREELAWS_START_NODE_HPP
#define THREELAWS_START_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>

template<typename node_t>
int start_node(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions{};

  auto node = std::make_shared<node_t>(options);
  node->init();

  rclcpp::executors::MultiThreadedExecutor exec{};
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}

#endif  // THREELAWS_START_NODE_HPP
