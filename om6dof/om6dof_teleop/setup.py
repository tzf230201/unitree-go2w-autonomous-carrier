from setuptools import setup
from glob import glob

package_name = 'om6dof_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py') + glob('launch/*.rviz')),
        ('share/' + package_name + '/systemd', glob('systemd/*.service')),
        ('share/' + package_name + '/sudoers', glob('sudoers/*')),
        ('share/' + package_name + '/skills', glob('skills/*.md')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='biancanobelia@gmail.com',
    description='Go2W input adapter for the canonical OM6DOF controller topics.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_node = om6dof_teleop.joint_teleop:main',
            'grip_object = om6dof_teleop.grip_object:main',
            'web_monitor = om6dof_teleop.web_monitor:main',
        ],
    },
)
