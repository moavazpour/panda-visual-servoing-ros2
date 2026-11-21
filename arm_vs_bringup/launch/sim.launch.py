from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, IncludeLaunchDescription,
                            RegisterEventHandler)
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
    world_file = PathJoinSubstitution([bringup_pkg, 'worlds', 'vs_world.sdf'])
    rviz_config = PathJoinSubstitution([bringup_pkg, 'rviz', 'sim.rviz'])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' controllers_file:=', controllers_file]),
        value_type=str)

    # 1. Tell Gazebo where our models are
    model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([bringup_pkg, 'models']))

    # 2. Gazebo with our world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'panda'],
    )

    # 3. Bridge between Gazebo and ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Simulation clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Wrist camera
            '/wrist_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Move objects (random layouts)
            '/world/vs_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            # Ground truth poses (benchmark)
            '/model/aruco_cube/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/place_shelf/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
    )

    # 4. Controllers, started once the robot exists in Gazebo
    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    arm_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller'],
    )
    velocity_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['velocity_controller', '--inactive'],
    )
    gripper_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_controller'],
    )
    start_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot,
                      on_exit=[joint_state_broadcaster, arm_controller,
                               velocity_controller, gripper_controller]))

    # 5. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        model_path,
        gazebo,
        robot_state_publisher,
        bridge,
        spawn_robot,
        start_controllers,
        rviz,
    ])