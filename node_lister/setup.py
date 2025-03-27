from setuptools import setup, find_packages
import os
from glob import glob

package_name = "node_lister"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yangming Huang",
    maintainer_email="yangming@coscene.io",
    description="ROS2 node monitor that publishes active/inactive node list.",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "node_lister = node_lister.node_lister:main",
        ],
    },
)
