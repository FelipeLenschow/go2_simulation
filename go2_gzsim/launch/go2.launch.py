from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import xacro
import os


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    simulation_arg = DeclareLaunchArgument(
        "simulation",
        default_value="true",
        description='Set to "true" to enable simulation mode, "false" for real robot',
    )

    fixed_base_arg = DeclareLaunchArgument(
        "fixed_base",
        default_value="false",
        description='Set to "true" if the base is fixed, "false" if the robot moves freely',
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("go2_description"), "urdf", "go2.xacro.urdf"]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "go2",
            "-allow_renaming",
            "true",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.5",
        ],
    )

    imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"],
    )

    go2_actuator_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["go2_actuator", "--controller-manager", "/controller_manager"],
    )

    go2_low_states_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["go2_lowstates", "--controller-manager", "/controller_manager"],
    )

    go2_joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "go2_jointcontroller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    bridge_params = os.path.join(
        get_package_share_directory("go2_gzsim"), "config", "gz_bridge.yaml"
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )

    go2_remap_node = Node(
        package="go2_remap",
        executable="go2_remap",
        output="screen",
    )

    # Define RViz Node
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("go2_description"), "rviz", "go2.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            {
                "use_sim_time": False,  # If using simulation time
                "tf_buffer_duration": 30.0,  # Increase TF buffer
            }
        ],
    )

    # lidar_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=["/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"],
    #     output="screen",
    # )

    return LaunchDescription(
        [
            simulation_arg,
            fixed_base_arg,
            # Launch gazebo environment
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ros_gz_sim"),
                                "launch",
                                "gz_sim.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments=[
                    (
                        "gz_args",
                        [
                            " -r -v 4 "
                            + os.path.join(
                                get_package_share_directory("go2_gzsim"),
                                "worlds/empty_world.sdf",
                            )
                        ],
                    )
                ],
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[go2_actuator_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=go2_actuator_spawner,
                    on_exit=[go2_low_states_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[go2_joint_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=go2_joint_controller,
                    on_exit=[go2_remap_node],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=go2_low_states_spawner,
                    on_start=[rviz_node],
                )
            ),
            node_robot_state_publisher,
            gz_spawn_entity,
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),
            imu_bridge,
            # lidar_bridge,
            ros_gz_bridge,
        ]
    )
