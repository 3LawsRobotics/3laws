/**
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */
#include "3laws/input_filter_node.hpp"

#include "./start_node.hpp"

int main(int argc, char ** argv) { return start_node<lll::InputFilterNode>(argc, argv); }
