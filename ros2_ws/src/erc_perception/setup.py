from glob import glob
import os

from setuptools import find_packages, setup


package_name = "erc_perception"


setup(
    name=package_name,

    version="1.0.0",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [
                "resource/" + package_name
            ],
        ),

        (
            "share/" + package_name,
            [
                "package.xml"
            ],
        ),

        (
            os.path.join(
                "share",
                package_name,
                "images",
            ),
            glob("images/*.png"),
        ),

        (
            os.path.join(
                "share",
                package_name,
                "templates",
            ),
            glob("templates/*.png"),
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
        "ERC 2026 unified computer vision "
        "perception package"
    ),

    license="Apache-2.0",

    tests_require=[
        "pytest",
    ],

    entry_points={
        "console_scripts": [
            (
                "image_publisher = "
                "erc_perception.image_publisher:main"
            ),

            (
                "perception_node = "
                "erc_perception.perception_node:main"
            ),
        ],
    },
)
