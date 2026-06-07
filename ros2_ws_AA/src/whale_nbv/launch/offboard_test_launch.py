from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    hover_z_arg = DeclareLaunchArgument(
        'hover_z',
        default_value='-2.0',
        description='Hover altitude in NED (negative = up)'
    )

    hover_z = LaunchConfiguration('hover_z')

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
        offboard_controller,
        nbv_planner,
    ])
