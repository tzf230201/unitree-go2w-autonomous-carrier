from setuptools import find_packages, setup

package_name = "go2w_remote_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/remote_monitor.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="unitree",
    maintainer_email="unitree@example.com",
    description="ROS 2 Python package to monitor Unitree Go2W remote controller data from /lowstate.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "remote_monitor = go2w_remote_monitor.remote_monitor:main",
        ],
    },
)
