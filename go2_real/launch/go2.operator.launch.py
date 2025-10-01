from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("go2_description"), "urdf", "go2.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Path to controller configurations
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("go2_description"), "config", "go2_real.yaml"]
    )

    # Define the robot_state_publisher node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
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
        parameters=[{
            'use_sim_time': False,  # If using simulation time
            'tf_buffer_duration': 30.0  # Increase TF buffer
        }]
    )

    # Define go2_remap node
    go2_remap_node = Node(
        package="go2_remap",
        executable="go2_remap",
        output="screen",
    )

    go2_remap_cloud_node = Node(
        package="go2_remap",
        executable="go2_remap_cloud",
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_pub_node,
            go2_remap_node,
            go2_remap_cloud_node,
            rviz_node,
        ]
    )
