from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart, OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ───── Launch arguments ─────
    simulation_arg = DeclareLaunchArgument(
        "simulation",
        default_value="true",
        description='Set to "true" for simulation mode'
    )

    fixed_base_arg = DeclareLaunchArgument(
        "fixed_base",
        default_value="false",
        description='Set to "true" to fix the robot base'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="If true, use simulated clock"
    )

    simulation = LaunchConfiguration("simulation")
    fixed_base = LaunchConfiguration("fixed_base")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ───── Generate robot_description using xacro ─────
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("go2_description"),
            "urdf",
            "go2.xacro.urdf"
        ]),
        " ",
        "simulation:=", simulation,
        " ",
        "fixed_base:=", fixed_base,
    ])

    robot_description = {"robot_description": robot_description_content}

    # ───── Nodes ─────
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
            "-topic", "robot_description",
            "-name", "go2",
            "-allow_renaming", "true",
            "-x", "0", "-y", "0", "-z", "0.5",
        ],
    )

    imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"],
        output="screen",
    )

    go2_actuator_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["go2_actuator", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    go2_low_states_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["go2_lowstates", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    go2_joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["go2_jointcontroller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    bridge_params = os.path.join(
        get_package_share_directory("go2_gzsim"),
        "config",
        "gz_bridge.yaml"
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args", "-p", f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    go2_remap_node = Node(
        package="go2_remap",
        executable="go2_remap",
        output="screen",
    )

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("go2_description"),
        "rviz",
        "go2.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "tf_buffer_duration": 30.0,
            }
        ],
    )

    # ───── Launch events ─────
    spawn_then_actuator = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gz_spawn_entity,
            on_start=[go2_actuator_spawner, go2_low_states_spawner, go2_remap_node],
        )
    )

    joint_controller_then_remap = RegisterEventHandler(
        event_handler=OnExecutionComplete(
            target_action=go2_actuator_spawner,
            on_completion=[go2_joint_controller],
        )
    )

    lowstates_then_rviz = RegisterEventHandler(
        event_handler=OnExecutionComplete(
            target_action=go2_joint_controller,
            on_completion=[rviz_node],
        )
    )

    # ───── Gazebo launch ─────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            "gz_args": "-r -v 4 " + os.path.join(
                get_package_share_directory("go2_gzsim"),
                "worlds/empty_world.sdf"
            )
        }.items()
    )

    # ───── Assemble description ─────
    return LaunchDescription([
        simulation_arg,
        fixed_base_arg,
        use_sim_time_arg,
        gazebo_launch,
        node_robot_state_publisher,
        gz_spawn_entity,
        spawn_then_actuator,
        joint_controller_then_remap,
        lowstates_then_rviz,
        imu_bridge,
        ros_gz_bridge,
    ])
