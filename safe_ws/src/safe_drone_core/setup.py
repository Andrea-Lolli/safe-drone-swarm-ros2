import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'safe_drone_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # La riga per la cartella launch (nota la virgola alla fine!)
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea',
    maintainer_email='andrea@todo.todo',
    description='SAFE Project: Search and Rescue Drone Core',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Assicurati che questi nomi corrispondano ai tuoi file effettivi!
            'mission_manager = safe_drone_core.mission_manager:main',
            'mock_sensor = safe_simulation.mock_sensor_node:main',
        ],
    },
)