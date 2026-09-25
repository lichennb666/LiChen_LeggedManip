from setuptools import find_packages, setup


package_name = "pcd_to_nav2_map"


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
    maintainer="ubu22",
    maintainer_email="ubu22@example.com",
    description="Offline PCD/PLY to Nav2 PGM/YAML map converter.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "convert = pcd_to_nav2_map.converter:main",
        ],
    },
)
