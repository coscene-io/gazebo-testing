from setuptools import setup, find_packages
import os
from glob import glob

package_name = "coscene_recorder"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "launch", "launch_ros"],
    zip_safe=True,
    maintainer="Yangming",
    maintainer_email="yangming@coscene.io",
    description="ROS2 mcap recorder package",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "coscene_recorder = recorder.recorder:main",
        ],
    },
)
