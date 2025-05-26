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
                [FindPackageShare("go2_description"), "urdf", "go2.xacro.urdf"]
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

    # Start controller_manager (manages ros2_control)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml],
        output="both",
        remappings=[
             ("~/robot_description", "/robot_description"),
        ],
    )

    # Delay before launching joint controller
    delayed_joint_controller_spawner = TimerAction(
        period=3.0,  # Small delay to ensure controller_manager is ready
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["go2_jointcontroller"],
                output="screen",
            )
        ],
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

    # Event handlers to ensure correct order of node execution
    controller_manager_event_handler =  RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_pub_node,
            on_start=[controller_manager],
        )
    )


    remap_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_pub_node,
            on_start=[go2_remap_node],
        )
    )

    remap_cloud_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=go2_remap_node,
            on_start=[go2_remap_cloud_node],
        )
    )

    return LaunchDescription(
        [
            robot_state_pub_node,
            controller_manager_event_handler,   
            delayed_joint_controller_spawner,
            remap_event_handler,
            remap_cloud_event_handler,
        ]
    )
