from glob import glob
import os

from setuptools import find_packages, setup


package_name = "omx_navigation"
map_files = glob("maps/hanyang_9f/20260728_204825/slam_toolbox/hanyang_9f*") or glob(
    "../../maps/hanyang_9f/20260728_204825/slam_toolbox/hanyang_9f*"
)


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
        (
            os.path.join(
                "share", package_name, "maps", "hanyang_9f", "20260728_204825", "slam_toolbox"
            ),
            map_files,
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OMX-AI",
    maintainer_email="maintainer@example.com",
    description="RViz-driven Nav2 bringup for a differential-drive OMX robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cmd_vel_safety_gate = omx_navigation.cmd_vel_safety_gate:main",
            "planar_base_frame = omx_navigation.planar_base_frame:main",
            "localization_supervisor = omx_navigation.localization_supervisor:main",
            "rviz_goal_bridge = omx_navigation.rviz_goal_bridge:main",
        ],
    },
)
