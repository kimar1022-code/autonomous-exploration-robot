from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aeri',
    maintainer_email='aeri@todo.todo',
    description='Autonomous exploration robot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
'console_scripts': [
            'base_controller_mock=autonomous_robot.base_controller_mock:main',
            'base_controller=autonomous_robot.base_controller:main',
            'camera_node=autonomous_robot.camera_node:main',
            'teleop_node=autonomous_robot.teleop_node:main',
        ],
    },
)
