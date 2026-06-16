from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'params'), glob('params/*.json')),
        (os.path.join('share', package_name, 'combined_urdf'), glob('combined_urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='david@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["hardware_node = robot_control.hardware_node:main",
                            "servo_node = robot_control.servo_node:main",
                            "pick_node = robot_control.pick_node:main",
                            "cumotion_pick_node = robot_control.cumotion_pick_node:main",
        ],
    },
)
