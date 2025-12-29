from setuptools import setup
import os
from glob import glob

package_name = 'dual_tb3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # index ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name,
            ['package.xml']),

        # 🔴 TOUS les launch (.py, pas *.launch.py)
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # 🔴 fichiers de config YAML
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samy',
    maintainer_email='samy@todo.todo',
    description='TB3 multi robot bringup',
    license='TODO',
)
