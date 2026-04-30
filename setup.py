from setuptools import setup

package_name = "mirobot_moveit_tracker"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/mirobot_moveit_tracker.launch.py",
                "launch/mirobot_wlkata_system.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="2022124194@kau.kr",
    description="ROS 2 Foxy MoveIt goal node for WLKATA Mirobot.",
    license="MIT",
    extras_require={
        "test": ["pytest"],
    },
   entry_points={
    "console_scripts": [
        "wheel_stop_to_goal_node = mirobot_moveit_tracker.wheel_stop_to_goal_node:main",
        "moveit_goal_node = mirobot_moveit_tracker.moveit_goal_node:main",
    ],
},

)
