from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sigyn_testicle_twister',
            executable='sigyn_testicle_twister_node',
            output='screen',
            # prefix=['sudo']
        )
    ])
