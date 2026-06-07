from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    hover_z_arg = DeclareLaunchArgument(
        'hover_z',
        default_value='-3.0',
        description='NBV hover altitude in NED (e.g. -3.0 = 3 metres)'
    )
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.70',
        description='Confidence threshold to declare marker acquired'
    )
    warmup_arg = DeclareLaunchArgument(
        'gps_warmup_s',
        default_value='20.0',
        description='Wait seconds for EKF convergence before takeoff'
    )
    aruco_dict_arg = DeclareLaunchArgument(
        'aruco_dict',
        default_value='4X4_50',
        description='ArUco dictionary: 4X4_50 (default, markers without plastic) or APRILTAG_36h11'
    )

    hover_z = LaunchConfiguration('hover_z')
    confidence = LaunchConfiguration('confidence_threshold')
    warmup = LaunchConfiguration('gps_warmup_s')
    aruco_dict = LaunchConfiguration('aruco_dict')

    # offboard_controller without auto_hover_z: first goal comes from nbv_planner
    offboard_controller = Node(
        package='whale_nbv',
        executable='offboard_controller',
        name='offboard_controller',
        output='screen',
    )

    nbv_planner = Node(
        package='whale_nbv',
        executable='nbv_planner',
        name='nbv_planner',
        output='screen',
        parameters=[{
            'hover_z': hover_z,
            'confidence_threshold': confidence,
            'gps_warmup_s': warmup,
        }]
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
        parameters=[{'aruco_dict': aruco_dict}]
    )

    return LaunchDescription([
        hover_z_arg,
        confidence_arg,
        warmup_arg,
        aruco_dict_arg,
        offboard_controller,
        nbv_planner,
        siyi_node,
        aruco_detector,
    ])
