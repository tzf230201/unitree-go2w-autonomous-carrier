from glob import glob

from setuptools import setup


package_name = "om6dof_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="unitree",
    maintainer_email="biancanobelia@gmail.com",
    description="OM6DOF joint, Cartesian, and cylindrical command converter.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "controller_node = om6dof_controller.controller_node:main",
        ],
    },
)
