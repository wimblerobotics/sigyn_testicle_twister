from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # PWM Driver Selection
        DeclareLaunchArgument(
            'pwm_driver_type',
            default_value='pca9685',
            description='PWM driver type: pca9685 (default), software, hardware'
        ),
        
        # PCA9685 specific arguments
        DeclareLaunchArgument(
            'i2c_address',
            default_value='64',  # 0x40 in decimal
            description='I2C address of PCA9685 (default: 64 = 0x40)'
        ),
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='1',
            description='I2C bus number (default: 1)'
        ),
        DeclareLaunchArgument(
            'pwm_frequency',
            default_value='50',
            description='PWM frequency in Hz (default: 50 for servos)'
        ),
        DeclareLaunchArgument(
            'pwm_channel',
            default_value='15',
            description='PWM channel (0-15 for PCA9685, 0 for others)'
        ),
        
        # Software PWM arguments (GPIO-based)
        DeclareLaunchArgument(
            'gpio_pin',
            default_value='18',
            description='GPIO pin for software PWM (default: 18 = Pin 12)'
        ),
        
        # Hardware PWM arguments (sysfs-based)
        DeclareLaunchArgument(
            'pwm_chip',
            default_value='1',
            description='PWM chip for hardware PWM (default: 1)'
        ),
        
        # Backward compatibility
        DeclareLaunchArgument(
            'use_software_pwm',
            default_value='false',
            description='Backward compatibility: use software PWM'
        ),
        
        # Main node
        Node(
            package='sigyn_testicle_twister',
            executable='sigyn_testicle_twister_node',
            output='screen',
            parameters=[{
                'pwm_driver_type': LaunchConfiguration('pwm_driver_type'),
                'i2c_address': LaunchConfiguration('i2c_address'),
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'pwm_frequency': LaunchConfiguration('pwm_frequency'),
                'pwm_channel': LaunchConfiguration('pwm_channel'),
                'gpio_pin': LaunchConfiguration('gpio_pin'),
                'pwm_chip': LaunchConfiguration('pwm_chip'),
                'use_software_pwm': LaunchConfiguration('use_software_pwm'),
            }],
        )
    ])
