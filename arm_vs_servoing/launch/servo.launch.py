from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    servo_enabled = LaunchConfiguration('servo_enabled')

    declare_servo_enabled = DeclareLaunchArgument(
        'servo_enabled', default_value='true',
        description='Start the visual servo active (true) or wait for /servo_enable (false)')

    switch_controllers = ExecuteProcess(
        cmd=['ros2', 'control', 'switch_controllers',
             '--activate', 'velocity_controller',
             '--deactivate', 'arm_controller'],
        output='screen',
    )

    cube_detector = Node(
        package='arm_vs_perception',
        executable='marker_detector',
        name='cube_marker_detector',
        parameters=[{'use_sim_time': True, 'marker_id': 0}],
        output='screen',
    )

    place_detector = Node(
        package='arm_vs_perception',
        executable='marker_detector',
        name='place_marker_detector',
        parameters=[{'use_sim_time': True, 'marker_id': 1}],
        remappings=[
            ('/marker_pose', '/place_marker_pose'),
            ('/marker_debug_image', '/place_debug_image'),
        ],
        output='screen',
    )

    cartesian_velocity = Node(
        package='arm_vs_control',
        executable='cartesian_velocity_node',
        output='screen',
    )

    visual_servo = Node(
        package='arm_vs_servoing',
        executable='visual_servo',
        parameters=[{
            'use_sim_time': True,
            'start_enabled': ParameterValue(servo_enabled, value_type=bool),
        }],
        output='screen',
    )

    return LaunchDescription([
        declare_servo_enabled,
        switch_controllers,
        cube_detector,
        place_detector,
        cartesian_velocity,
        visual_servo,
    ])