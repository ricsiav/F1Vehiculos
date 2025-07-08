from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='f1tenth_gym_ros',
            executable='follow_gap',
            name='follow_gap_node',
            output='screen'
        ),
        Node(
            package='f1tenth_gym_ros',
            executable='lap_counter',
            name='lap_counter_node',
            output='screen'
        ),
        Node(
            package='f1tenth_gym_ros',
            executable='lap_timer',
            name='lap_timer_node',
            output='screen'
        )
    ])
