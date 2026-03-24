from setuptools import find_packages, setup

package_name = 'whale_nbv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea',
    maintainer_email='andrea@example.com',
    description='Pacchetto ROS2 per progetto NBV drone',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'offboard_controller = whale_nbv.offboard_controller:main',
            'waypoint_manager = whale_nbv.waypoint_manager:main',
            'aruco_detector = whale_nbv.aruco_detector:main',
            'aruco_controller = whale_nbv.aruco_controller:main',
        ],
    },
)
