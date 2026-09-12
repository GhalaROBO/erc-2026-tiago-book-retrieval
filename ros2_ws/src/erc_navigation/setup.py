from setuptools import find_packages, setup


package_name = "erc_navigation"


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
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="Ghala",
    maintainer_email="ghala@example.com",

    description=(
        "ERC 2026 navigation interface"
    ),

    license="Apache-2.0",

    entry_points={
        "console_scripts": [
            (
                "navigation_node = "
                "erc_navigation.navigation_node:main"
            ),
        ],
    },
)
