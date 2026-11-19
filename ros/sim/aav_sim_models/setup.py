from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'aav_sim_models'


def tree(folder):
    """Install every file under folder, keeping its layout (the models' meshes, pictures)."""
    return [(os.path.join('share', package_name, root), [os.path.join(root, f) for f in files])
            for root, _, files in os.walk(folder) if files]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf') + glob('worlds/*.lanes.yaml')),
        *tree('models'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AAV 2026-27 team',
    maintainer_email='aav-team@todo.invalid',
    description="Ravensim's worlds, models and car nodes, behind the real car's topics.",
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'sim_adapter = aav_sim_models.sim_adapter:main',
            'sim_imu = aav_sim_models.sim_imu:main',
            'sim_gps = aav_sim_models.sim_gps:main',
            'sim_radar = aav_sim_models.sim_radar:main',
            'sim_camera_relay = aav_sim_models.sim_camera_relay:main',
            'sim_lidar_relay = aav_sim_models.sim_lidar_relay:main',
            'sim_esp32 = aav_sim_models.sim_esp32:main',
            'truth_localizer = aav_sim_models.truth_localizer:main',
        ],
    },
)
