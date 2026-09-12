from glob import glob
import os

from setuptools import find_packages, setup


package_name = "erc_bringup"


setup(
    name=package_name,
    version="1.0.0",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob("launch/*.launch.py"),
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="Ghala",
    maintainer_email="ghala@example.com",

    description=(
        "ERC 2026 full system bringup package"
    ),

    license="Apache-2.0",
)
