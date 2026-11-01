from glob import glob

from setuptools import find_packages, setup

package_name = 'aav_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AAV 2026-27 team',
    maintainer_email='aav-team@todo.invalid',
    description='The sensor watchdog, the stop hold and the geofence: twist_mux locks '
                'that stop the automatic drivers.',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'sensor_watchdog = aav_safety.sensor_watchdog:main',
            'stop_hold = aav_safety.stop_hold:main',
            'geofence = aav_safety.geofence_node:main',
        ],
    },
)
