"""
Bringup launchfile of demo n°10 : TIAGoPro bar bi-manipulation

Usage:
  ros2 launch agimus_demo_10_tiago_pro_bar_manip bringup.launch.py use_gazebo:=true use_sim_time:=True
"""

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PKG_NAME = "agimus_demo_10_tiago_pro_bar_manip"


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:

    mocap_tf_pub = Node(
        package="mocap_ros",
        executable="mocap_node",
        name="mocap_node",
        output="screen",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [FindPackageShare(PKG_NAME), "config", "mocap_config.yaml"]
                ),
            }
        ],
    )

    mocap_repositioning_node = Node(
        package=PKG_NAME,
        executable="mocap_repositioning_node",
        name="mocap_repositioning_node",
        output="screen",
    )

    return [
        mocap_tf_pub,
        mocap_repositioning_node,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
