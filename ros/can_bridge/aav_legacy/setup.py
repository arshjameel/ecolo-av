from glob import glob

from setuptools import find_packages, setup

package_name = 'aav_legacy'

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
    description='Interim backend for the 2025-26 ESP32 firmware and UDP wheel encoder.',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'legacy_uart_bridge = aav_legacy.legacy_uart_bridge:main',
            'encoder_udp = aav_legacy.encoder_udp:main',
            'steering_limits = aav_legacy.steering_limits:main',
        ],
    },
)
