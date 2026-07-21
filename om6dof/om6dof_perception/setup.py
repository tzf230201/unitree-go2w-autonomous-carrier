from glob import glob

from setuptools import setup

package_name = "om6dof_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/systemd", glob("systemd/*.service")),
        ("share/" + package_name + "/sudoers", glob("sudoers/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="unitree",
    maintainer_email="biancanobelia@gmail.com",
    description="Camera-frame object and EE perception for OM6DOF",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "perception_node = om6dof_perception.perception_node:main",
            "perception_view = om6dof_perception.perception_view:main",
        ],
    },
)
