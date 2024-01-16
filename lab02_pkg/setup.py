from setuptools import find_packages, setup
import os
import glob

package_name = 'lab02_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giuseppe',
    maintainer_email='giuseppedeninarivera@gmail.com',
    description='lab02_pkg',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'service = lab02_pkg.ComputeTrajectoryService:main',
                'goal_generator = lab02_pkg.goal_generator:main',
                'ActionServer = lab02_pkg.ActionServer:main',
        ],
    },
)
