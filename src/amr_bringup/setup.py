from setuptools import setup
import os
from glob import glob

package_name = 'amr_bringup'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Al Azhar Faradis',
    maintainer_email='muhammadalazharf@gmail.com',
    description=(
        'Bringup launch files untuk Mobile Robot Ackermann Indoor Platform. '
        'Master launch file: amr_full.launch.py.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [],   # tidak ada Python node di package ini, hanya launch
    },
)
