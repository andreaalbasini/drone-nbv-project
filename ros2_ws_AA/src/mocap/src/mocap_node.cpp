/**
 * @file mocap_node.cpp
 * @brief Entry point for mocap ROS2 node
 * @author Kota Kondo <kkondo@mit.edu>
 * @date Dec 17, 2024 (Updated for ROS2)
 */

#include <rclcpp/rclcpp.hpp>
#include "mocap/mocap.h"

int main(int argc, char* argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create the ROS2 node
  auto node = std::make_shared<acl::mocap::Mocap>("mocap");

  // Spin the node (run the main event loop)
  node->spin();

  // Cleanup and shutdown
  rclcpp::shutdown();
  return 0;
}
