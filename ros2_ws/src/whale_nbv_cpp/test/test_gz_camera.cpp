#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <iostream>

static void cb(ConstImageStampedPtr &msg)
{
  const auto &img = msg->image();
  std::cout
    << "frame "
    << img.width() << "x" << img.height()
    << " step=" << img.step()
    << " bytes=" << img.data().size()
    << std::endl;
}

int main(int argc, char **argv)
{
  gazebo::client::setup(argc, argv);

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  std::string topic = "/gazebo/default/iris/camera_link/down_rgb_camera/image";
  auto sub = node->Subscribe(topic, cb);

  std::cout << "Subscribed to: " << topic << std::endl;

  while (true) {
    gazebo::common::Time::MSleep(100);
  }

  gazebo::client::shutdown();
  return 0;
}