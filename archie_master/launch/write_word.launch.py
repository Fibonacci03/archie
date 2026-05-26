from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='archie_master',
            executable='write_word_node',
            name='planing_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])