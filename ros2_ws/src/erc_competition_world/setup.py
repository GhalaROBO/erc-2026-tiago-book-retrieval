from setuptools import find_packages
from setuptools import setup


package_name = 'erc_competition_world'


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='ghala',
    maintainer_email='ghala@example.com',

    description=(
        'Fixed ERC competition arena for '
        'TIAGo Pro Gazebo simulation.'
    ),

    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            (
                'arena_node = '
                'erc_competition_world.arena_node:main'
            ),
        ],
    },
)
