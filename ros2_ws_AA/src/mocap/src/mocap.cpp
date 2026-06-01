/**
 * @file mocap.cpp
 * @brief Broadcasts mocap data onto ROS2 network
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 11 Dec 2019 (Updated for ROS2)
 */

#include "mocap/mocap.h"
#include <chrono>
using namespace std::chrono_literals;

namespace acl {
namespace mocap {

Mocap::Mocap(const std::string& node_name)
: rclcpp::Node(node_name)
{
  // Initialize parameters
  this->declare_parameter<std::string>("rpc_server_uri", "http://192.168.0.9:1250");
  this->declare_parameter<bool>("enable_rpc", false);
  this->declare_parameter<std::string>("tf_parent_frame", "world");
  this->declare_parameter<std::string>("mocap_wrt_parent_frame", "0 0 0 0 0 0");
  this->declare_parameter<std::string>("body_wrt_mocap_body", "0 0 0 0 0 0");
  this->declare_parameter<bool>("should_pub_unlabeledmarkers", false);
  this->declare_parameter<bool>("should_pub_cameras", false);
  this->declare_parameter<std::string>("client", "vicon");

  // Initialize clients
  if (!init()) {
    RCLCPP_ERROR(this->get_logger(), "Initialization failed");
    rclcpp::shutdown();
  }

  // Connect to mocap client
  RCLCPP_INFO(this->get_logger(), "Attempting to connect to %s server...", client_->getName().c_str());
  if (client_->init()) {
    RCLCPP_INFO(this->get_logger(), "Connected! SDK version: %s", client_->getSDKVersion().c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Could not connect to %s server!", client_->getName().c_str());
    rclcpp::shutdown();
  }

}

// ----------------------------------------------------------------------------

void Mocap::spin()
{
  // Do some debouncing on the number of lost packets. It is not critical
  // unless many consecutive packets are being dropped.
  constexpr int MAX_LOST_PACKETS = 4;
  unsigned int lostPackets = 0;

  while (rclcpp::ok()) 
  {
    if (!client_->spinOnce()) {
      if (++lostPackets >= MAX_LOST_PACKETS) {
        RCLCPP_ERROR_STREAM_THROTTLE(
          this->get_logger(),      // your node’s logger
          *this->get_clock(),      // your node’s clock
          1000,
          "Did not receive data from "
            << client_->getName()
            << " server! (Are cameras on?)"
        );
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),      // your node’s logger
          *this->get_clock(),      // your node’s clock
          1000,
          "Missing/dropped packets."
        );
      }
    } else {
      lostPackets = 0;
    }

    //
    // Process rigid bodies from mocap client
    //

    // calculate dt between mocap data frames
    static uint64_t last_time_ns = client_->getCurrentTimestampNs();
    mocap_dt_ = (client_->getCurrentTimestampNs() - last_time_ns) * 1e-9;

    // skip iteration if dt is too small (occasionally happens on first iter)
    if (mocap_dt_ < 1e-6) {
      continue;
    }

    auto rbMeasurements = client_->getRigidBodyMeasurements();
    for (const auto & rb : rbMeasurements) {
      // if this measurement doesn't correspond to an existing rigid body, add it
      if (bodyMap_.find(rb.name) == bodyMap_.end()) {
        RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Found '" << rb.name << "'"
        );
        bodyMap_[rb.name].reset(new Body(
          shared_from_this(), // your Node*
          rpc_client_,
          rb.name,
          bodyParams_
        ));

        
      }

      // update body pose using latest mocap data
      bodyMap_[rb.name]->update(mocap_dt_, rb);

      // broadcast rigid body state/info onto ROS network
      bodyMap_[rb.name]->broadcastROS();
    }

    last_time_ns = client_->getCurrentTimestampNs();

    //
    // Remove any bodies that we didn't receive a measurement of
    //

    // NOTE: A "measurement" does not necessarily mean the object is visible.
    // Objects should only be removed from the body map if it is 'disabled' in
    // the motion capture software. In other words, invalid measurements do not
    // constitute removal from the body map.

    // For each body currently in the body map, was there a corresponding meas?
    for (auto it = bodyMap_.cbegin(); it != bodyMap_.cend(); /*manual inc*/) {
      if (std::find_if(rbMeasurements.begin(), rbMeasurements.end(),
                        [&](const RigidBodyMeasurement& rb) {
                          return rb.name == it->first;
                        }) == rbMeasurements.end())
      {
        // not found, remove from the body map
        RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Removing '" << it->first << "' from body map"
        );
        bodyMap_.erase(it++);
      } else {
        // found, advance to next body
        ++it;
      }
    }

    //
    // Process Cameras and Unlabeled Markers
    //

    if (should_pub_cameras_) {
      broadcastROS_cameras();
    }

    if (should_pub_unlabeledmarkers_) {
      broadcastROS_unlabeled_markers();
    }

    // print connection information
    screenPrint();
  }
}

// ----------------------------------------------------------------------------

bool Mocap::init()
{
  // ──────────────────────────────────────────────────────────
  // 1) RPC Client initialization
  // ──────────────────────────────────────────────────────────

  // ROS1: nhp_.param<bool>("enable_rpc", enable_rpc, false);
  // ROS2: declare, then get

  bool enable_rpc = this->get_parameter("enable_rpc").as_bool();
  std::string rpc_server_uri = this->get_parameter("rpc_server_uri").as_string();

  rpc_client_ = std::make_shared<RPCClient>(rpc_server_uri, enable_rpc);

  // ──────────────────────────────────────────────────────────
  // 2) General mocap parameters
  // ──────────────────────────────────────────────────────────

  // ROS1: nhp_.param<std::string>("tf_parent_frame", bodyParams_.parent_frame, "world");
  bodyParams_.parent_frame = this->get_parameter("tf_parent_frame").as_string();

  // ROS1: nhp_.param<std::string>("mocap_wrt_parent_frame", ...)
  {
    auto s = this->get_parameter("mocap_wrt_parent_frame").as_string();
    if (! parse_xyzYPR(s, bodyParams_.T_PM)) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "parameter 'mocap_wrt_parent_frame' expected 'x y z Y P R' but got '" << s << "'"
      );
      return false;
    }
  }

  // ROS1: nhp_.param<std::string>("body_wrt_mocap_body", ...)
  {
    auto s = this->get_parameter("body_wrt_mocap_body").as_string();
    if (! parse_xyzYPR(s, bodyParams_.T_MbB)) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "parameter 'body_wrt_mocap_body' expected 'x y z Y P R' but got '" << s << "'"
      );
      return false;
    }
  }

  // ROS1: nhp_.param<bool>("should_pub_unlabeledmarkers", should_pub_unlabeledmarkers_, false);
  should_pub_unlabeledmarkers_ = this->get_parameter("should_pub_unlabeledmarkers").as_bool();
  should_pub_cameras_ = this->get_parameter("should_pub_cameras").as_bool();

  if (should_pub_unlabeledmarkers_) {
    // ROS1: nhp_.advertise<::mocap::Markers>("unlabeled_markers",1);
    pub_unlabeledmarkers_ =
      this->create_publisher<::mocap::msg::Markers>("unlabeled_markers", 1);
  }
  if (should_pub_cameras_) {
    pub_cameras_ =
      this->create_publisher<::mocap::msg::Cameras>("cameras", 1);
  }

  // ──────────────────────────────────────────────────────────
  // 3) Client-specific parameters + initialization
  // ──────────────────────────────────────────────────────────

  std::string client = this->get_parameter("client").as_string();
  std::transform(client.begin(), client.end(), client.begin(),
                 [](unsigned char c){ return std::tolower(c); });

  if (client == "vicon") {
    VICON::Parameters params;
    // ROS1: nhp_.param<std::string>("host", params.host, ...)
    this->declare_parameter<std::string>("host", "192.168.0.9:801");
    params.host = this->get_parameter("host").as_string();

    client_.reset(new VICON(params));

  } else if (client == "optitrack") {
    OptiTrack::Parameters params;
    // ROS1: nhp_.param<std::string>("local", params.localIP, ...)
    this->declare_parameter<std::string>("local", "192.168.1.119");
    params.localIP = this->get_parameter("local").as_string();

    // ROS1: nhp_.param<std::string>("server", ...)
    this->declare_parameter<std::string>("server", "192.168.1.12");
    params.serverIP = this->get_parameter("server").as_string();

    // ROS1: nhp_.param<std::string>("multicast_group", ...)
    this->declare_parameter<std::string>("multicast_group", "239.255.42.99");
    params.multicastIP = this->get_parameter("multicast_group").as_string();

    // ROS1: nhp_.param<int>("command_port", ...), etc.
    this->declare_parameter<int>("command_port", 1510);
    params.commandPort = this->get_parameter("command_port").as_int();

    this->declare_parameter<int>("data_port", 1511);
    params.dataPort = this->get_parameter("data_port").as_int();

    client_.reset(new OptiTrack(params));

  } else {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "Invalid mocap client '" << client << "'."
    );
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------

bool Mocap::parse_xyzYPR(const std::string& xyzYPR, tf2::Transform& T) const
{
  constexpr int expectedValues = 6; // x y z Y P R
  std::vector<double> values;
  std::stringstream ss(xyzYPR);

  try {
    std::string tmp;
    while (std::getline(ss, tmp, ' ')) {
      values.push_back(std::stod(tmp));
    }
  } catch (...) {
    return false;
  }

  if (values.size() != expectedValues) return false;

  tf2::Vector3 p(values[0], values[1], values[2]);
  tf2::Quaternion q;
  q.setRPY(values[5], values[4], values[3]);

  // force w > 0
  if (q.w() < 0.0) 
  {
    q.setW(-q.w());
    q.setX(-q.x());
    q.setY(-q.y());
    q.setZ(-q.z());
  }

  T.setOrigin(p);
  T.setRotation(q);

  return true;
}

// ----------------------------------------------------------------------------

void Mocap::screenPrint()
{
  const double latency = client_->getTotalLatency();
  auto& clk = *this->get_clock();
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    clk,
    10000,
    "Mocap: %s, Latency: %.2f ms",
    client_->getName().c_str(),
    (latency > 0.0 ? latency * 1e3 : -1.0)
  );
}

// ----------------------------------------------------------------------------

void Mocap::broadcastROS_cameras()
{
  // once‐only initialization of the calibration & static message container
  static bool cameras_msg_initialized = false;
  auto cameras = client_->getCameras();

  static ::mocap::msg::Cameras cameras_msg;
  if (!cameras_msg_initialized) {
    // fetch calibrations via your RPC client
    const auto calibs = rpc_client_->getCameras();

    for (const auto & camera : cameras) {
      ::mocap::msg::Camera cam_msg;
      cam_msg.name     = camera.name;
      cam_msg.type     = camera.type;
      cam_msg.deviceid = camera.deviceid;

      // look up calibration for this device‐ID
      auto it = std::find_if(
        calibs.begin(), calibs.end(),
        [&id = camera.deviceid](auto const & c){ return c.deviceid == id; }
      );
      if (it != calibs.end()) {
        const auto & c = *it;
        cam_msg.enabled     = c.enabled;
        cam_msg.index       = c.index;
        cam_msg.image_error = c.image_error;

        // pose in mocap frame → pose in ROS “parent” frame
        tf2::convert(c.position,     cam_msg.pose.position);
        tf2::convert(c.orientation,  cam_msg.pose.orientation);

        tf2::Transform T_MC;
        tf2::convert(cam_msg.pose, T_MC);

        tf2::Transform T_PC = bodyParams_.T_PM * T_MC;
        tf2::convert(T_PC, cam_msg.pose);

        // fill in sensor_msgs::msg::CameraInfo
        sensor_msgs::msg::CameraInfo cinfo;
        cinfo.width     = c.image_width;
        cinfo.height    = c.image_height;
        cinfo.distortion_model = "vicon_radial";
        cinfo.d.push_back(c.radial_distortion[0]);
        cinfo.d.push_back(c.radial_distortion[1]);
        cinfo.k[0] = cinfo.k[4] = c.focal_length;
        cinfo.k[2] = c.principal_point.x();
        cinfo.k[5] = c.principal_point.y();
        cinfo.k[8] = 1.0; 
        cam_msg.camera_info = cinfo;
      }

      cameras_msg.cameras.push_back(std::move(cam_msg));
    }

    cameras_msg_initialized = true;
  }

  // stamp with mocap‐server time (instead of this->now())
  const uint64_t ns = client_->getEstimatedTimeNsOffset()
                    + client_->getCurrentTimestampNs();
  rclcpp::Time timestamp(ns /*nsec*/, RCL_ROS_TIME);
  cameras_msg.header.stamp    = timestamp;
  cameras_msg.header.frame_id = bodyParams_.parent_frame;

  // update per‐frame fields (centroids, etc.)
  for (size_t i = 0; i < cameras.size(); ++i) {
    const auto & camera = cameras[i];
    auto     & cam_msg = cameras_msg.cameras[i];
    assert(cam_msg.deviceid == camera.deviceid);

    cam_msg.camera_info.header = cameras_msg.header;
    cam_msg.centroids.clear();
    for (auto const & ctr : camera.centroids) {
      ::mocap::msg::Centroid ctr_msg;
      ctr_msg.u      = ctr.u;
      ctr_msg.v      = ctr.v;
      ctr_msg.radius = ctr.radius;
      cam_msg.centroids.push_back(std::move(ctr_msg));
    }
  }

  // finally, publish
  pub_cameras_->publish(cameras_msg);
}

// ----------------------------------------------------------------------------

void Mocap::broadcastROS_unlabeled_markers()
{
  // 1) Grab the latest markers from your client
  auto markers = client_->getUnlabeledMarkers();

  // 2) Build a fresh ROS2 message
  ::mocap::msg::Markers msg;
  // Stamp with your mocap‐server time
  const uint64_t ns = 
    client_->getEstimatedTimeNsOffset() + 
    client_->getCurrentTimestampNs();
  msg.header.stamp = rclcpp::Time(ns, RCL_ROS_TIME);
  msg.header.frame_id = bodyParams_.parent_frame;

  // 3) Fill in each marker
  for (const auto & marker : markers) {
    ::mocap::msg::Marker m;
    m.name = marker.name;
    m.position.x = marker.x;
    m.position.y = marker.y;
    m.position.z = marker.z;

    // Transform from mocap frame → ROS “parent” frame
    tf2::Transform T_Mm;
    tf2::convert(m.position, T_Mm);
    const tf2::Transform T_PBm = 
      bodyParams_.T_PM * T_Mm * bodyParams_.T_MbB;
    tf2::convert(T_PBm, m.position);

    msg.markers.push_back(std::move(m));
  }

  // 4) Publish
  pub_unlabeledmarkers_->publish(msg);
}

// ----------------------------------------------------------------------------

} // namespace mocap
} // namespace acl
