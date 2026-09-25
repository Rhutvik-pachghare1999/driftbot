from setuptools import setup
import os
from glob import glob

package_name = 'driftbot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh') + glob('scripts/*.py')),
        (os.path.join('share', package_name, 'gz', 'robots'), glob('gz/robots/*.xacro')),
        (os.path.join('share', package_name, 'gz', 'worlds'), glob('gz/worlds/*.sdf')),
    ],
)