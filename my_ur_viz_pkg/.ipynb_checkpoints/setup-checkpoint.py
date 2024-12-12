from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_ur_viz_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the launch directory and its files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'paho-mqtt'],  # Ensure paho-mqtt is included
    zip_safe=True,
    maintainer='jupyter-esarvey',
    maintainer_email='elijah.sarvey@tufts.edu',
    description='MQTT Simulator for UR3e',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'viz_nodes = my_ur_viz_pkg.viz_nodes:main',  # Register the node executable
        ],
    },
)
