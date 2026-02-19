from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'imiev'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # <- Detecta imiev y imiev.utils
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join(f'share/{package_name}/launch'), glob('launch/*.launch.py')),
        # Config files (excluding directories)
        (os.path.join(f'share/{package_name}/config'), [f for f in glob('config/*') if os.path.isfile(f)]),
        # Behavior Trees
        (os.path.join(f'share/{package_name}/config/behavior_trees'), glob('config/behavior_trees/*.xml')),
        (os.path.join(f'share/{package_name}/worlds'), glob('worlds/*')),
        # Copiar XACRO y SDF
        (os.path.join(f'share/{package_name}/models/imiev'), glob('models/*.xacro')),
        (os.path.join(f'share/{package_name}/models/imiev'), glob('models/*.gazebo')),
        # Copiar STL
        # (os.path.join(f'share/{package_name}/models/imiev/meshes'), glob('models/meshes/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lagarto',
    maintainer_email='lauramartin8garcia@gmail.com',
    description='Robot Mutt para ROS2 + Gazebo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'interactive_waypoint_follower = imiev.interactive_waypoint_follower:main',
            'gps_waypoint_logger = imiev.gps_waypoint_logger:main',
            'logged_waypoint_follower = imiev.logged_waypoint_follower:main',
        ],
    },
)
