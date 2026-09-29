from glob import glob

from setuptools import setup

package_name = 'drone_s1_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Xabi',
    maintainer_email='64962381+XabMS@users.noreply.github.com',
    description='Hito S1: despegue, estacionario y aterrizaje del x500 desde ROS 2.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'takeoff_hover_land = drone_s1_demo.takeoff_hover_land:main',
        ],
    },
)
