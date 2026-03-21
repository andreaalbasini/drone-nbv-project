cmake_minimum_required(VERSION 3.8)
project(whale_nbv_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(TBB REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

add_executable(gazebo_camera_bridge src/gazebo_camera_bridge.cpp)

ament_target_dependencies(
  gazebo_camera_bridge
  rclcpp
  sensor_msgs
)

target_link_libraries(gazebo_camera_bridge
  ${GAZEBO_LIBRARIES}
  TBB::tbb
)

install(TARGETS
  gazebo_camera_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()