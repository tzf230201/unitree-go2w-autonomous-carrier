from setuptools import setup
from glob import glob

package_name = 'go2w_remote_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py') + glob('launch/*.rviz')),
        ('share/' + package_name + '/systemd', glob('systemd/*.service')),
        ('share/' + package_name + '/skills', glob('skills/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='biancanobelia@gmail.com',
    description='Go2W wireless remote to OpenManipulator-X direct joint teleop.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_node = go2w_remote_arm.teleop_node:main',
            'arm_launcher = go2w_remote_arm.launcher_node:main',
            'remote_servo_bridge = go2w_remote_arm.remote_servo_bridge:main',
            'grip_object = go2w_remote_arm.grip_object:main',
            'web_monitor = go2w_remote_arm.web_monitor:main',
        ],
    },
)
