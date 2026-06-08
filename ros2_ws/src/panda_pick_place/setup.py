from setuptools import find_packages, setup
import os
from glob import glob

package_name = "panda_pick_place"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.json")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="grtsinry43",
    maintainer_email="grtsinry43@outlook.com",
    description="Pick-and-place perception and executor nodes",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "perception_node = panda_pick_place.perception_node:main",
            "executor_node = panda_pick_place.executor_node:main",
        ],
    },
)
