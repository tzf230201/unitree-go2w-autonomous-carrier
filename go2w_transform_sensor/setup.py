from setuptools import setup
from glob import glob

package_name = 'go2w_transform_sensor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tzf230201',
    maintainer_email='tzf230201@todo.todo',
    description='Rotate /utlidar/cloud to base-aligned orientation and restamp it with monotonic wall-clock time.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'transform_cloud = go2w_transform_sensor.transform_cloud:main',
        ],
    },
)
