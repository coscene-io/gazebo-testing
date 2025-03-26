import os
from glob import glob
from setuptools import find_packages, setup

package_name = "marker_publisher"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "mesh"), glob("mesh/*.stl")),
        (os.path.join("share", package_name, "param"), glob("param/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Qingyu Zeng",
    maintainer_email="qingyu.zeng@coscene.io",
    description="A ROS2 package to publish markers",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "marker_publisher = marker_publisher.marker_publisher:main"
        ],
    },
)
