/**
 * @author Thomas Gurriet (tgurriet@3laws.io)
 * @brief
 *
 * @copyright Copyright 2022 3Laws Robotics Inc.
 */

#include "3laws/kernel_generator_node_unicycle.hpp"

#include "./start_node.hpp"

int main(int argc, char ** argv)
{
  return start_node<lll::KernelGeneratorNodeUnicycle>(argc, argv);
}
