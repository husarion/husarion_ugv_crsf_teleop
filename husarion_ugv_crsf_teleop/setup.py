import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'husarion_ugv_crsf_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'config'), ['config/crsf_teleop.yaml']),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Milosz Lagan',
    maintainer_email='milosz.lagan@husarion.com',
    description='Teleoperate robots using a CRSF compatible remote control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crsf_teleop_node = husarion_ugv_crsf_teleop.crsf_teleop_node:main'
        ],
    },
)
