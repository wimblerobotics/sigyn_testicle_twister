from setuptools import setup

package_name = 'sigyn_testicle_twister'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    include_package_data=True,  # Ensure this is True
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sigyn_testicle_twister.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Wimble',
    maintainer_email='mike@wimblerobotics.com',
    description='ROS2 package for controlling a gripper servo via PWM on Raspberry Pi.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sigyn_testicle_twister_node = sigyn_testicle_twister.sigyn_testicle_twister_node:main',
        ],
    },
)
