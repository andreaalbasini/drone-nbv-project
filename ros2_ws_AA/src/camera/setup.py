from setuptools import setup
import os
from glob import glob

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    # Trova i moduli Python nella sottocartella camera/camera
    packages=[package_name],
    data_files=[
        # Rende il pacchetto visibile a ROS 2 (Risolve il "Package not found")
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Installa il package.xml nella share directory
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lp10',
    maintainer_email='lp10@todo.com',
    description='SIYI camera node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Assicurati che siyi_node.py abbia una funzione def main():
            'siyi_node = camera.siyi_node:main',
        ],
    },
)
