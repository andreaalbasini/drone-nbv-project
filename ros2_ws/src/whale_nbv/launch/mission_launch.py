# launch/mission_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    hover_z_arg = DeclareLaunchArgument(
        'hover_z',
        default_value='-10.0',
        description='Quota iniziale di hover in NED (negativo = su)'
    )

    hover_z = LaunchConfiguration('hover_z')

    gazebo_bridge = Node(
        package='whale_nbv_cpp',
        executable='gazebo_camera_bridge',
        name='gazebo_camera_bridge',
        output='screen'
    )

    aruco_detector = Node(
        package='whale_nbv',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen'
    )

    offboard_controller = Node(
        package='whale_nbv',
        executable='offboard_controller',
        name='offboard_controller',
        output='screen'
    )

    nbv_planner = Node(
        package='whale_nbv',
        executable='nbv_planner',
        name='nbv_planner',
        output='screen',
        parameters=[{
            'hover_z': hover_z
        }]
    )

    return LaunchDescription([
        hover_z_arg,
        gazebo_bridge,
        aruco_detector,
        offboard_controller,
        nbv_planner,
    ])