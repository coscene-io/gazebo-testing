from setuptools import setup
import os
from glob import glob

package_name = "nav_test"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Qingyu",
    maintainer_email="qingyu@example.com",
    description="Navigation test system",
    license="Apache 2.0",
    entry_points={
        "console_scripts": [
            "nav_controller = nav_test.nav_controller:main",
        ],
    },
)
