from glob import glob
import os

from setuptools import find_packages, setup


package_name = "go1_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "LICENSE", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OMX-AI",
    maintainer_email="maintainer@example.com",
    description="Safety-filtered ROS 2 cmd_vel bridge for the legacy Unitree Go1 SDK.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["go1_driver = go1_driver.node:main"]},
)
