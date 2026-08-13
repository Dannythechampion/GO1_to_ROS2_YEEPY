from glob import glob
import os

from setuptools import find_packages, setup


package_name = "omx_navigation"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OMX-AI",
    maintainer_email="maintainer@example.com",
    description="RViz-driven Nav2 bringup for a differential-drive OMX robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "commission_start_pose = omx_navigation.commission_start_pose:main",
            "localization_supervisor = omx_navigation.localization_supervisor:main",
            "motion_gate = omx_navigation.motion_gate:main",
            "fixed_mission_manager = omx_navigation.fixed_mission_manager:main",
            "record_destination_pose = omx_navigation.record_destination_pose:main",
        ],
    },
)
