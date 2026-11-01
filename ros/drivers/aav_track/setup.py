from glob import glob

from setuptools import find_packages, setup

package_name = 'aav_track'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AAV 2026-27 team',
    maintainer_email='aav-team@todo.invalid',
    description='Trial path followers in place of Nav2 controller_server: FollowPath.',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'track = aav_track.track_node:main',
        ],
    },
)
