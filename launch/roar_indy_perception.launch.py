import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="indy_bot_urdf").find(
        "indy_bot_urdf"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz/with_sensor_config.rviz")

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    robot_state_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("indy_bot_urdf"),
                "launch",
                "state_publisher_no_rviz.launch.py",
            )
        )
    )
    livox_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("livox_ros2_driver"),
                "launch/indy_livox_lidar.launch.py",
            )
        ),
        launch_arguments={
            "lidar_publish_freq": launch.substitutions.LaunchConfiguration(
                "lidar_publish_freq"
            ),
            "frame_id": launch.substitutions.LaunchConfiguration("frame_id"),
        }.items(),
    )
    zed_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed_wrapper"),
                "launch",
                "indy_zed2i_no_rviz.launch.py",
            )
        )
    )
    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="gui",
                default_value="True",
                description="Flag to enable joint_state_publisher_gui",
            ),
            launch.actions.DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            launch.actions.DeclareLaunchArgument(
                name="lidar_publish_freq",
                default_value="10.0",
            ),
            launch.actions.DeclareLaunchArgument(
                name="frame_id",
                default_value="lidar_link",
            ),
            livox_node,
            zed_node,
            robot_state_node,
            rviz_node,
        ]
    )
