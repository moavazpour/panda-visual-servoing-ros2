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

    # 2. Marker 0: the cube
    cube_detector = Node(
        package='arm_vs_perception',
        executable='marker_detector',
        name='cube_marker_detector',
        parameters=[{'use_sim_time': True, 'marker_id': 0}],
        output='screen',
    )

    # 3. Marker 1: the place location, same program with other topic names
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

    # 4. Cartesian velocity control
    cartesian_velocity = Node(
        package='arm_vs_control',
        executable='cartesian_velocity_node',
        output='screen',
    )

    # 5. Visual servoing
    visual_servo = Node(
        package='arm_vs_servoing',
        executable='visual_servo',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        switch_controllers,
        cube_detector,
        place_detector,
        cartesian_velocity,
        visual_servo,
    ])