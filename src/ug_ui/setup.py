from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ug_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'images'),
            glob('ug_ui/*.png')
        ),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Joseph Polania',
    maintainer_email='joseph.polania@uni.lu',
    description='PyQt5 graphical user interface for the Universal Gripper',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ui = ug_ui.launcher:main',
        ],
    },
)
