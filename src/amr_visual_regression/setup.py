from setuptools import setup
import os
from glob import glob

package_name = 'amr_visual_regression'

setup(
    name=package_name,
    version='0.1.0',
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
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'scikit-learn',
        'joblib',
    ],
    zip_safe=True,
    maintainer='Muhammad Al Azhar Faradis',
    maintainer_email='muhammadalazharf@gmail.com',
    description='Visual Regression for AMR obstacle avoidance + exploration',
    license='MIT',
    entry_points={
        'console_scripts': [
            'data_collector_node       = amr_visual_regression.data_collector_node:main',
            'vr_inference_node         = amr_visual_regression.vr_inference_node:main',
            'lidar_line_segments_node  = amr_visual_regression.lidar_line_segments_node:main',
        ],
    },
)
