from setuptools import find_packages, setup

package_name = "go2w_odom_projector"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="unitree",
    maintainer_email="unitree@example.com",
    description="Project 3D odometry into planar x/y/yaw odom for 2D stacks.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_projector = go2w_odom_projector.odom_projector:main",
        ],
    },
)
