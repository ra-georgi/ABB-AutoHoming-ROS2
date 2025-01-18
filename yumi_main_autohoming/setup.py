import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'yumi_main_autohoming'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name,'urdf'), glob('urdf/*')),  
        (os.path.join('share', package_name, 'config'), glob('config/*')),      
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reuben',
    maintainer_email='reuben@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_joint_states = yumi_main_autohoming.get_current_joints:main',
            'set_yumi_joints = yumi_main_autohoming.set_yumi_joints:main',
        ],
    },
)
