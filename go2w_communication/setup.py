from setuptools import find_packages, setup

package_name = "go2w_communication"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="unitree",
    maintainer_email="unitree@example.com",
    description="Speak text or chat with Ollama through the Go2W body speaker (audiohub megaphone).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "speak = go2w_communication.speak:main",
            "chat = go2w_communication.chat:main",
            "chat_stream = go2w_communication.chat_stream:main",
            "chat_webrtc = go2w_communication.chat_webrtc:main",
        ],
    },
)
