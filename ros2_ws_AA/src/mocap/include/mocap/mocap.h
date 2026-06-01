/**
 * @file mocap.hpp
 * @brief Broadcasts mocap data onto ROS2 network
 * @date 11 Dec 2019 (Updated for ROS2)
 */

#pragma once

#include <algorithm>
#include <cctype>
#include <map>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <mocap/msg/markers.hpp>
#include <mocap/msg/cameras.hpp>

#include "mocap/body.h"
#include "mocap/rpc_client.h"
#include "mocap/client.h"
#include "mocap/client/vicon.h"
#include "mocap/client/optitrack.h"

namespace acl {
namespace mocap {

  class Mocap : public rclcpp::Node
  {
  public:
    /**
     * @brief      Constructor for the Mocap node
     *
     * @param[in]  node_name  Name of the ROS2 node
     */
    explicit Mocap(const std::string& node_name);
    ~Mocap() = default;

    /**
     * @brief      Main loop. Synchronously checks for data and publishes it.
     */
    void spin();
  
  private:
    // Publishers for cameras and markers
    rclcpp::Publisher<::mocap::msg::Markers>::SharedPtr pub_unlabeledmarkers_;
    rclcpp::Publisher<::mocap::msg::Cameras>::SharedPtr pub_cameras_;

    // Client and RPC
    RPCClientPtr rpc_client_;
    std::unique_ptr<Client> client_; ///< ptr to mocap client specialization

    // Configurations
    bool should_pub_unlabeledmarkers_;
    bool should_pub_cameras_;
    double mocap_dt_; ///< measured period at which we receive mocap

    /// Tracked rigid bodies (enabled in mocap GUI), keyed by name
    std::map<std::string, std::unique_ptr<Body>> bodyMap_;
    Body::Parameters bodyParams_; ///< user parameters for each rigid body

    /**
     * @brief      Initialize the mocap client with ROS2 parameters
     *
     * @return     True if successful
     */
    bool init();

    /**
     * @brief      Parse a string into a transform
     *
     * @param[in]  xyzYPR  String with "x y z Y P R"
     * @param      T       resulting transform object
     *
     * @return     Whether or not parsing was successful
     */
    bool parse_xyzYPR(const std::string& xyzYPR, tf2::Transform& T) const;

    /**
     * @brief      Print information to stdout
     */
    void screenPrint();

    /**
     * @brief      Publish camera data to ROS2 topic
     */
    void broadcastROS_cameras();

    /**
     * @brief      Publish unlabeled marker data to ROS2 topic
     */
    void broadcastROS_unlabeled_markers();
  };

} // namespace mocap
} // namespace acl
