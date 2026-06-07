from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    hover_z_arg = DeclareLaunchArgument(
        'hover_z',
        default_value='-2.0',
        description='Hover altitude in NED (negative = up, e.g. -2.0 = 2 metres)'
    )

    hover_z = LaunchConfiguration('hover_z')

    offboard_controller = Node(
        package='whale_nbv',
        executable='offboard_controller',
        name='offboard_controller',
        output='screen',
        parameters=[{'auto_hover_z': hover_z}]
    )

    siyi_node = Node(
        package='camera',
        executable='siyi_node',
        name='siyi_node',
        output='screen',
    )

    aruco_detector = Node(
        package='whale_nbv',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
    )

    return LaunchDescription([
        hover_z_arg,
        offboard_controller,
        siyi_node,
        aruco_detector,
    ])
