from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mcvilloid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        (os.path.join('share', package_name, 'protos'), glob('protos/*.proto')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'controllers', 'mcvilloid_controller'),
            glob('controllers/mcvilloid_controller/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maldonado',
    maintainer_email='maldonadoggmx@gmail.com',
    description='Mcvilloid prueba de paquete ROS2 con Webots',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [],
    },
)
