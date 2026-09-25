from setuptools import find_packages, setup

package_name = "go2_piper_wbc"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/wbc.yaml", "config/policy_contract.yaml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="lili",
    maintainer_email="lili@robot.com",
    description="ROS 2 adapter for the Go2-Piper TorchScript WBC policy.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["wbc_node = go2_piper_wbc.wbc_node:main"]},
)
