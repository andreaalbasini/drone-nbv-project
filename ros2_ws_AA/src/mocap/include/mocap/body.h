/*!
 * \file body.h
 *
 * Class holding position and orientation filters for vicon objects.
 *
 * Created on: Oct 16, 2013
 *     Author: Mark Cutler
 *      Email: markjcutler@gmail.com
 * Updated further on: 12 Dec 2019
 *       Author: Parker Lusk
 *     Email: parkerclusk@gmail.com
 * Updated for ROS2: 2024
 */

#pragma once

#include <algorithm>
#include <deque>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include "mocap/msg/status.hpp"
#include "mocap/msg/marker.hpp"

#include "mocap/tf2_helpers.h"
#include "mocap/client.h"
#include "mocap/GHKFilter.h"
#include "mocap/attitude_ekf.h"
#include "mocap/rpc_client.h"

namespace acl {
namespace mocap {

  /**
   *  This class has position and orientation filters as members.
   */
  class Body
  {
  public:
    struct Parameters {
      std::string parent_frame; ///< name of (ROS) parent frame

      tf2::Transform T_PM; ///< mocap origin w.r.t parent frame (internal)
      tf2::Transform T_MbB; ///< ROS body w.r.t mocap body (internal)

      uint8_t num_markers_missing_tol = 4; ///< tolerance for missing markers
      double q_diff_threshold = 0.1; ///< orientation jump threshold

      uint8_t skipped_meas_pos_tol = 3; ///< skipped measurements for position
      uint8_t skipped_meas_att_tol = 20; ///< skipped measurements for attitude

      bool pub_twist = true;
      bool pub_accel = true;
      bool pub_status = true;

      bool tf_broadcast = true; ///< should broadcast pose on tf tree
    };

  public:
    Body(const std::shared_ptr<rclcpp::Node>& node, const RPCClientPtr rpc_client,
          const std::string& name, const Parameters& params);
    ~Body() = default;

    void update(double dt, const RigidBodyMeasurement& rb);
    bool broadcastROS();

  private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr pub_accel_;
    rclcpp::Publisher<::mocap::msg::Status>::SharedPtr pub_status_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;
    RPCClientPtr rpc_client_;

    /// \brief Internal state
    std::string name_;
    Parameters params_;
    ::mocap::msg::Status status_;
    uint64_t timestamp_ns_ = 0;
    bool pub_initialized_ = false;
    geometry_msgs::msg::Pose last_meas_;
    std::vector<Marker> model_markers_;

    /// \brief Filters
    GHKFilter ghk; ///< position, velocity, acceleration filter
    AttitudeEKF ekf; ///< attitude and attitude rate filter

    void initPublishers();

    void propagationUpdate(double dt);
    void measurementUpdate(double dt, const RigidBodyMeasurement& rb);

    geometry_msgs::msg::Pose unpackMeasurement(const RigidBodyMeasurement& rb) const;
    bool getFlipProcessedQuaternion(const geometry_msgs::msg::Quaternion& q_ref,
                                    geometry_msgs::msg::Quaternion& q) const;
    double getEuclideanDistanceSq(const geometry_msgs::msg::Quaternion& a,
                                  const geometry_msgs::msg::Quaternion& b) const;
    double getAngleDifference(const geometry_msgs::msg::Quaternion& a,
                              const geometry_msgs::msg::Quaternion& b) const;
  };

} // namespace mocap
} // namespace acl
