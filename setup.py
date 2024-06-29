from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'imarker_pose_mockups'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include parameter files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include all YAML files from the config directory and subdirectories
        (os.path.join('share', package_name, 'config/example'), glob('config/example/*.yaml')),
        (os.path.join('share', package_name, 'config/rviz2'), glob('config/rviz2/*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rpros2024ss',
    maintainer_email='julian.eichenbaum@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'InteractiveMarkerPointXYZNode = imarker_pose_mockups.interactive_marker_point_xyz_node:main'
        ],
    },
)