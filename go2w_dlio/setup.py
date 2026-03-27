from setuptools import find_packages, setup

package_name = 'go2w_dlio'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/go2w_dlio.launch.py', 'launch/display_dlio.rviz']),
        ('share/' + package_name + '/config', ['config/go2w_dlio_params.yaml', 'config/hesai_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@example.com',
    description='Wrapper launch package for DLIO with Hesai LiDAR and Go2W IMU topics.',
    license='MIT',
    tests_require=['pytest'],
)
