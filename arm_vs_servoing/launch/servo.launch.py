from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Switch from the position controller to the velocity controller
    switch_controllers = ExecuteProcess(
        cmd=['ros2', 'control', 'switch_controllers',
             '--activate', 'velocity_controller',
             '--deactivate', 'arm_controller'],
        output='screen',
    )

    # 2. Marker detection (day 8)
    marker_detector = Node(
        package='arm_vs_perception',
        executable='marker_detector',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 3. Cartesian velocity control (day 6)
    cartesian_velocity = Node(
        package='arm_vs_control',
        executable='cartesian_velocity_node',
        output='screen',
    )

    # 4. Visual servoing (today)
    visual_servo = Node(
        package='arm_vs_servoing',
        executable='visual_servo',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        switch_controllers,
        marker_detector,
        cartesian_velocity,
        visual_servo,
    ])