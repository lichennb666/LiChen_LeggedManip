from glob import glob
from setuptools import find_packages, setup

package_name = "go2_piper_bringup"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/worlds", glob("worlds/*.world")),
        ("share/" + package_name + "/models", glob("models/*.sdf")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lili",
    maintainer_email="lili@robot.com",
    description="Go2-Piper Gazebo Classic bringup.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "odom_tf = go2_piper_bringup.odom_tf:main",
        "teleop_keyboard = go2_piper_bringup.teleop_keyboard:main",
        "teleop_gui = go2_piper_bringup.teleop_gui:main",
        "waypoint_follower = go2_piper_bringup.waypoint_follower:main",
        "odom_relay = go2_piper_bringup.odom_relay:main",
        "course1_goal_gateway = go2_piper_bringup.course1_goal_gateway:main",
        "pcd_to_occupancy = go2_piper_bringup.pcd_to_occupancy:main",
    ]},
)
