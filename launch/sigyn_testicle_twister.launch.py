from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_software_pwm',
            default_value='true',
            description='Use software PWM (recommended for Pi 5)'
        ),
        DeclareLaunchArgument(
            'gpio_pin',
            default_value='18',
            description='GPIO pin for software PWM (default: 18 = Pin 12)'
        ),
        DeclareLaunchArgument(
            'pwm_chip',
            default_value='1',
            description='PWM chip for hardware PWM (default: 1)'
        ),
        DeclareLaunchArgument(
            'pwm_channel',
            default_value='0',
            description='PWM channel for hardware PWM (default: 0)'
        ),
        
        # Main node
        Node(
            package='sigyn_testicle_twister',
            executable='sigyn_testicle_twister_node',
            output='screen',
            parameters=[{
                'use_software_pwm': LaunchConfiguration('use_software_pwm'),
                'gpio_pin': LaunchConfiguration('gpio_pin'),
                'pwm_chip': LaunchConfiguration('pwm_chip'),
                'pwm_channel': LaunchConfiguration('pwm_channel'),
            }],
            # Note: May need to run with sudo for GPIO access
            # Uncomment the next line if needed:
            # prefix=['sudo']
        )
    ])
