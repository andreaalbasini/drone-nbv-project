#include <memory>
#include <string>

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class GazeboImageBridge : public rclcpp::Node
{
public:
  GazeboImageBridge()
  : Node("gazebo_camera_bridge")
  {
    gazebo_topic_ = this->declare_parameter<std::string>(
      "gazebo_topic",
      "/gazebo/default/iris/camera_link/down_rgb_camera/image");

    ros_topic_ = this->declare_parameter<std::string>(
      "ros_topic",
      "/camera/image_raw");

    frame_id_ = this->declare_parameter<std::string>(
      "frame_id",
      "camera_link");

    pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      ros_topic_,
      rclcpp::SensorDataQoS());

    gazebo::client::setup();

    gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gz_node_->Init();

    sub_ = gz_node_->Subscribe(gazebo_topic_, &GazeboImageBridge::OnImageStamped, this);

    RCLCPP_INFO(this->get_logger(), "Subscribed Gazebo topic: %s", gazebo_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing ROS 2 topic: %s", ros_topic_.c_str());
  }

  ~GazeboImageBridge() override
  {
    gazebo::client::shutdown();
  }

private:
  void OnImageStamped(ConstImageStampedPtr &msg)
  {
    const auto &img = msg->image();

    sensor_msgs::msg::Image ros_img;
    ros_img.header.stamp = this->now();
    ros_img.header.frame_id = frame_id_;
    ros_img.height = img.height();
    ros_img.width = img.width();
    ros_img.step = img.step();
    ros_img.is_bigendian = false;

    // The camera SDF is configured as R8G8B8
    ros_img.encoding = "rgb8";

    const std::string &data = img.data();
    ros_img.data.assign(data.begin(), data.end());

    pub_->publish(ros_img);
  }

  std::string gazebo_topic_;
  std::string ros_topic_;
  std::string frame_id_;

  gazebo::transport::NodePtr gz_node_;
  gazebo::transport::SubscriberPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GazeboImageBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}