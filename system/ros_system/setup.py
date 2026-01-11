from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/curtis_f2a_fault_codes.json', 'resource/cavalier_error_codes.json']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cavallatestbench',
    maintainer_email='victor.w.boyd@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'forklift_health = ros_system.forklift_health:main',
            'system_health = ros_system.system_health:main',
            'teleop_start = ros_system.teleop_start:main',
            'forklift_state = ros_system.forklift_state:main'
        ],
    },
)
