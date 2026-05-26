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
        ('share/' + package_name + '/launch', glob('launch/*.py')),
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
        ],
    },
)
