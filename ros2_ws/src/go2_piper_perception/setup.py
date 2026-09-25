from setuptools import find_packages, setup

package_name = "go2_piper_perception"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/perception.yaml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="lili",
    maintainer_email="lili@robot.com",
    description="HSV RGB-D detector for a colored grasp handle.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "color_detector = go2_piper_perception.color_detector_node:main",
        "synthetic_camera = go2_piper_perception.synthetic_camera:main",
    ]},
)
