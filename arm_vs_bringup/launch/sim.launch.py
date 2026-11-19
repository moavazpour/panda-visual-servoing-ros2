from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_pkg = FindPackageShare('arm_vs_description')
    bringup_pkg = FindPackageShare('arm_vs_bringup')

    xacro_file = PathJoinSubstitution([description_pkg, 'urdf', 'panda.urdf.xacro'])
    controllers_file = PathJoinSubstitution([bringup_pkg, 'config', 'controllers.yaml'])

    # 1. Build the robot model and pass the controllers file into it
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' controllers_file:=', controllers_file]),
        value_type=str)

    # 2. Start Gazebo with an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. Publish the robot model and TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # 4. Put the robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'panda'],
    )

    # 5. Bring the simulation clock into ROS
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # 6. Start the controllers
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
    )

    # Controllers can only start after the robot is in Gazebo
    start_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot,
                      on_exit=[joint_state_broadcaster, arm_controller]))

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        clock_bridge,
        spawn_robot,
        start_controllers,
    ])