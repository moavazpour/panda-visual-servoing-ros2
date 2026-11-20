from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. Simulation, robot and controllers
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('arm_vs_bringup'), 'launch', 'sim.launch.py'])),
    )

    # 2. Perception, Cartesian control and the visual servo, which waits to be enabled
    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('arm_vs_servoing'), 'launch', 'servo.launch.py'])),
        launch_arguments={'servo_enabled': 'false'}.items(),
    )

    # 3. The task: an endless loop of random pick-and-place cycles
    task = Node(
        package='arm_vs_servoing',
        executable='pick_and_place',
        parameters=[{
            'use_sim_time': True,
            'place_push': 0.005,
        }],
        output='screen',
    )

    # Give Gazebo and the controllers time to start before the rest
    return LaunchDescription([
        sim,
        TimerAction(period=10.0, actions=[servo, task]),
    ])