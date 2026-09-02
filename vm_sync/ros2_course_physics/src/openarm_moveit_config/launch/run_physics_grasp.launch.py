import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

os.environ["MUJOCO_GL"] = "osmesa"


def generate_launch_description():
    launch_dir = os.path.dirname(__file__)
    model_dir = os.path.join(launch_dir, "..", "model")

    mujoco_xml_file = os.path.join(model_dir, "table_scene_physics.xml")
    urdf_file = os.path.join(model_dir, "table_scene_physics.urdf")
    controller_config = os.path.join(launch_dir, "physics_controllers.yaml")
    detector_script = os.path.join(launch_dir, "apple_detector.py")
    table_script = os.path.join(launch_dir, "spawn_table_scene.py")

    with open(urdf_file, "r") as handle:
        robot_desc = handle.read()

    viewer_render_period = LaunchConfiguration("viewer_render_period", default="0.0166666667")

    node_mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": True},
            {"mujoco_model_path": mujoco_xml_file},
            controller_config,
            {"viewer_render_period": viewer_render_period},
            {"camera_period": 0.1666666667},
            {"grasp_telemetry.object_geom": "physics_object_geom"},
            {"grasp_telemetry.gripper_body": "openarm_left_hand"},
            {"grasp_telemetry.publish_every_n_steps": 10},
        ],
    )

    node_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
    )

    tf_world_to_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.5", "0", "1.2", "0", "0", "0", "1", "world", "camera_base_link"],
        output="screen",
    )

    tf_cam_to_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "1", "0", "0", "0", "camera_base_link", "camera_optical_frame"],
        output="screen",
    )

    load_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
        output="screen",
    )
    load_left_arm = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "left_arm_controller"],
        output="screen",
    )
    load_right_arm = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "right_arm_controller"],
        output="screen",
    )
    load_left_hand = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "left_hand_controller"],
        output="screen",
    )
    load_right_hand = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "right_hand_controller"],
        output="screen",
    )

    node_detector = ExecuteProcess(
        cmd=["python3", detector_script, "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )

    spawn_table = ExecuteProcess(
        cmd=["python3", table_script, "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )

    launch_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "move_group_physics.launch.py"))
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "viewer_render_period",
                default_value="0.0166666667",
                description="Simulated seconds between viewer redraws.",
            ),
            node_mujoco,
            node_rsp,
            tf_world_to_cam,
            tf_cam_to_optical,
            node_detector,
            launch_moveit,
            spawn_table,
            load_jsb,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_jsb,
                    on_exit=[load_left_arm, load_right_arm, load_left_hand, load_right_hand],
                )
            ),
        ]
    )
