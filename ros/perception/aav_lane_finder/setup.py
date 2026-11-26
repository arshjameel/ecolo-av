from glob import glob

from setuptools import find_packages, setup

package_name = 'aav_lane_finder'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AAV 2026-27 team',
    maintainer_email='aav-team@todo.invalid',
    description="The lane finder's launch file, config and Python maths (classic OpenCV); "
                'the node is aav_lane_finder_rs (the Python node removed 22 Nov 2026).',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
        ],
    },
)
