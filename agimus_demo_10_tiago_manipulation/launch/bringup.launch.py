from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import (
    generate_default_tiago_pro_args,
    get_use_sim_time,
)
from launch.actions import (
    ExecuteProcess,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:

    # robot_state_publisher = include_scoped_launch_py_description(
    #     pkg_name='tiago_pro_description',
    #     paths=['launch', 'robot_state_publisher.launch.py'],
    #     launch_arguments={"arm_type_right": TiagoProArgs.arm_type_right,
    #                       "arm_type_left": TiagoProArgs.arm_type_left,
    #                       "end_effector_right": TiagoProArgs.end_effector_right,
    #                       "end_effector_left": TiagoProArgs.end_effector_left,
    #                       "ft_sensor_right":"no-ft-sensor",
    #                       "ft_sensor_left":"no-ft-sensor",
    #                       "ft_sensor_teleop_right": "no-ft-sensor",
    #                       "ft_sensor_teleop_left": "no-ft-sensor",
    #                       "wrist_model_right": TiagoProArgs.wrist_model_right,
    #                       "wrist_model_left": TiagoProArgs.wrist_model_left,
    #                       "tool_changer_right": TiagoProArgs.tool_changer_right,
    #                       "tool_changer_left": TiagoProArgs.tool_changer_left,
    #                       "laser_model": TiagoProArgs.laser_model,
    #                       "camera_model": TiagoProArgs.camera_model,
    #                       "base_type": TiagoProArgs.base_type,
    #                       "torque_estimation":"False",
    #                       "calibration_tool": "False",
    #                       "namespace":"",
    #                       "use_sim_time":"False",
    #                       "is_public_sim": "True",
    #                       "has_teleop_arms": "False",
    #                       "has_wrist_camera": "False",
    #                       })
    # joint_state_pub_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen')
    send_initial_pose = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/arm_right_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            '{joint_names: ["arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint"], '
            "points: [{positions: [-3.127, -1.75,  0.35, -2.15, 3.0, 0.6, 0.0], time_from_start: {sec: 4, nanosec: 0}}]}",
        ],
        output="screen",
    )

    lift_torso = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "torso_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            '{joint_names: ["torso_lift_joint"], '
            "points: [{positions: [1.], time_from_start: {sec: 4, nanosec: 0}}]}",
        ],
        output="screen",
    )
    # activate_controllers = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "control",
    #         "switch_controllers",
    #         "--deactivate",
    #         "arm_right_controller",
    #         "--activate",
    #         "arm_right_1_joint_inertia_shaping_controller",
    #         "arm_right_2_joint_inertia_shaping_controller",
    #         "arm_right_3_joint_inertia_shaping_controller",
    #         "arm_right_4_joint_inertia_shaping_controller",
    #         "arm_right_5_joint_inertia_shaping_controller",
    #         "arm_right_5_joint_inertia_shaping_controller",
    #         "arm_right_6_joint_inertia_shaping_controller",
    #         "arm_right_7_joint_inertia_shaping_controller",
    #         "linear_feedback_controller",
    #         "joint_state_estimator",
    #     ],
    #     output="screen",
    # )
    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_10_tiago_manipulation"),
            "rviz",
            "config.rviz",
        ]
    )
    tf_odom = Node(
        package="agimus_demo_10_tiago_manipulation",
        executable="tf_base_publisher",
        name="tf_base_publisher",
        output="screen",
    )
    mpc_params = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_10_tiago_manipulation"),
            "config",
        ]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": get_use_sim_time()}],
        arguments=["-d", rviz_config_path.perform(context)],
    )
    mpc_node = Node(
        package="agimus_demo_10_tiago_manipulation",
        executable="aligator_MPC",
        name="aligator_mpc",
        parameters=[{"config": mpc_params.perform(context)}],
    )
    return [
        # robot_state_publisher,
        rviz,
        tf_odom,
        # joint_state_pub_gui,
        mpc_node,
        send_initial_pose,
        lift_torso,
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_tiago_pro_args() + [OpaqueFunction(function=launch_setup)]
    )
