from setuptools import setup
from glob import glob

package_name = 'open_manipulator_x_ctrl'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='biancanobelia@gmail.com',
    description='OpenManipulator-X position controller via U2D2 with FK/IK.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = open_manipulator_x_ctrl.controller_node:main',
            'gui = open_manipulator_x_ctrl.gui_node:main',
            'go_home = open_manipulator_x_ctrl.cli_tools:go_home',
            'disable_torque = open_manipulator_x_ctrl.cli_tools:disable_torque',
        ],
    },
)
