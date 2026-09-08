from setuptools import find_packages, setup

package_name = 'ug_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/ug_hardware.launch.py',
        ]),
    ],
    install_requires=[
    'setuptools',
    'python-can',
    'numpy',
],
    zip_safe=True,
    maintainer='Joseph Polania',
    maintainer_email='joseph.polania@uni.lu',
    description='Hardware drivers and ROS 2 hardware interface for the Universal Gripper',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hardware_interface = ug_hardware.hardware_interface:main',
        ],
    },
)
