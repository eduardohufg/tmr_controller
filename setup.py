from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tmr_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardohufg',
    maintainer_email='eduardochavezmartin10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision = tmr_controller.vision:main',
            'path_generator = tmr_controller.path_generator:main',
            'odometry = tmr_controller.odometry:main',
            'controller = tmr_controller.controler_master:main',
            'mapping_points = tmr_controller.mapping_points:main',
            'move = tmr_controller.move:main',
        ],
    },
)
