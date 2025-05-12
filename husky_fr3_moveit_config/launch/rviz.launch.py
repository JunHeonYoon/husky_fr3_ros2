from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def launch_setup(context, *args, **kwargs):
    load_gripper_val = LaunchConfiguration("load_gripper").perform(context)  # "true"/"false"
    load_mobile_val  = LaunchConfiguration("load_mobile").perform(context)

    srdf_file = (
        "config/husky_fr3.srdf" if load_mobile_val.lower() == "true"
        else "config/fr3.srdf"
    )

    moveit_config = (
        MoveItConfigsBuilder("husky_fr3")
        .robot_description(
            file_path="config/husky_fr3.urdf.xacro",
            mappings={
                "hand":  LaunchConfiguration("load_gripper"),
                "mobile_base": LaunchConfiguration("load_mobile"),
            })
        .robot_description_semantic(file_path=srdf_file)
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    rviz_cfg = os.path.join(
        get_package_share_directory("husky_fr3_moveit_config"),
        "config", "moveit_py.rviz",
    )

    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", rviz_cfg],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    robot_state_pub = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="log",
        parameters=[moveit_config.robot_description,
                    {"publish_frequency": 500.0},
                    {"ignore_timestamp": True},
                    ],
    )

    static_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="static_transform_publisher", output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    return [robot_state_pub, rviz_node, static_tf]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "load_gripper", default_value="true",
            description="If true, include FR3 gripper"
        ),
        DeclareLaunchArgument(
            "load_mobile", default_value="true",
            description="If true, mount FR3 on Husky base"
        ),
        OpaqueFunction(function=launch_setup),
    ])
