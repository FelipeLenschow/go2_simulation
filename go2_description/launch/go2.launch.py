from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

# if is a XACRO file, disocment this line
import xacro
import os

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('go2_description'),
                 'urdf', 'go2.xacro.urdf']
            ),
        ]
    )
    robot_description = {'robot_description':robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'go2', '-allow_renaming', 'true',
                   '-x', '0',
                   '-y', '0',
                   '-z', '0.45'],
    )

    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster'],
    # )

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
        arguments=["go2_jointcontroller", "--controller-manager", "/controller_manager"],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"],
    )    

    bridge_params = os.path.join(get_package_share_directory('go2_description'), 'config', 'gz_bridge.yaml')
    
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 ' + os.path.join(get_package_share_directory('go2_description'), 'worlds/empty_world.sdf')])]),
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
                target_action=go2_low_states_spawner,
                on_exit=[go2_joint_controller],
            )
        ),
        node_robot_state_publisher,
        gz_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
            bridge,
            ros_gz_bridge,
    ])