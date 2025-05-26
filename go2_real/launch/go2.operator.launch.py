from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

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

    return LaunchDescription(
        [
            rviz_node
        ]
    )
