from setuptools import find_packages, setup

package_name = 'aav_actuator_tests'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AAV 2026-27 team',
    maintainer_email='aav-team@todo.invalid',
    description='The actuator tests: drive, log, and print the numbers for vehicle.yaml.',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'actuator_test = aav_actuator_tests.actuator_test:main',
            'car_test = aav_actuator_tests.car_test_node:main',
        ],
    },
)
