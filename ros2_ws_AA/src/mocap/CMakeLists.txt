cmake_minimum_required(VERSION 3.16)
project(mocap)

# ————————————————————————————————
# 1) C++ standard
# ROS 1: set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE "Release")
endif()

# ————————————————————————————————
# 2) ament + ROS 2 dependencies
# ROS 1: find_package(catkin REQUIRED COMPONENTS roscpp rospy …)
find_package(ament_cmake REQUIRED)          # replaces catkin macros
find_package(rclcpp REQUIRED)               # instead of roscpp
find_package(std_msgs REQUIRED)             # same
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)                  # instead of tf2
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

# For custom messages
find_package(rosidl_default_generators REQUIRED)

# System dependencies
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen Version: ${EIGEN3_VERSION_STRING} (${EIGEN3_INCLUDE_DIRS})")

# ————————————————————————————————
# 3) External SDKs
# ROS 1: add_subdirectory(sdk/vicon), add_subdirectory(sdk/optitrack)
include(cmake/fastrpc.cmake)
add_subdirectory(sdk/vicon)
add_subdirectory(sdk/optitrack)

# ————————————————————————————————
# 4) Message generation
# ROS 1: add_message_files + generate_messages()
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Status.msg"
  "msg/Centroid.msg"
  "msg/Camera.msg"
  "msg/Cameras.msg"
  "msg/Marker.msg"
  "msg/Markers.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ————————————————————————————————
# 5) Headers
# ROS 1: include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
# ROS 2: per-target instead

# ————————————————————————————————
# 6) Build executables
# ROS 1: add_executable(...) + target_link_libraries(... ${catkin_LIBRARIES})
add_executable(mocap_node
  src/mocap_node.cpp
  src/GHKFilter.cpp
  src/attitude_ekf.cpp
  src/body.cpp
  src/mocap.cpp
  src/rpc_client.cpp
  src/client/vicon.cpp
  src/client/optitrack.cpp
)

add_executable(test_vicon_rpc_node
  src/test_vicon_rpc_node.cpp
)

# ————————————————————————————————
# 7) Link and include
# ROS 1: ament_target_dependencies -> replaces catkin_target + ${catkin_LIBRARIES}
ament_target_dependencies(mocap_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  rosidl_default_runtime
)

# bring in the generated headers for your own msgs:
rosidl_target_interfaces(mocap_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

ament_target_dependencies(test_vicon_rpc_node
  rclcpp
)

# bring in the generated headers for your own msgs:
rosidl_target_interfaces(test_vicon_rpc_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# SDK libraries from add_subdirectory
target_link_libraries(mocap_node
    vicon_sdk
    optitrack_sdk
    fastrpc
)

target_link_libraries(test_vicon_rpc_node
    fastrpc
)

target_include_directories(mocap_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}   # <--- ADD THIS
)

target_include_directories(test_vicon_rpc_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}   # <--- ADD THIS
)

# ————————————————————————————————
# 8) Installation
# ROS 1: install(TARGETS... ARCHIVE/LIBRARY RUNTIME DESTINATION ${CATKIN_PACKAGE_*})
install(TARGETS
  mocap_node
  test_vicon_rpc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install entire launch/ directory into share/<project_name>/launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ──────────────────────────────────────────────────────────
# Install & export the Optitrack/fastRPC SDK libraries
# ──────────────────────────────────────────────────────────
install(
  TARGETS
    optitrack_sdk
    fastrpc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export package info
ament_export_include_directories(include)
ament_export_libraries(vicon_sdk optitrack_sdk fastrpc)
ament_package()

