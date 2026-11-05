from setuptools import find_packages, setup

package_name = 'aav_diffusion_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'training']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AAV 2026-27 team',
    maintainer_email='aav-team@todo.invalid',
    description='A trial planner: a diffusion model draws the path, the lattice guards it.',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'diffusion_planner = aav_diffusion_planner.planner_node:main',
        ],
    },
)
