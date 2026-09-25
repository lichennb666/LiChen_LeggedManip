from setuptools import find_packages, setup

package_name = "go2_piper_mission"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/mission.yaml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="lili",
    maintainer_email="lili@robot.com",
    description="Autonomous fixed-waypoint pick and transport mission.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "mission_server = go2_piper_mission.mission_server:main",
        "start_mission = go2_piper_mission.start_mission:main",
        "wbc_gate = go2_piper_mission.wbc_gate:main",
    ]},
)

