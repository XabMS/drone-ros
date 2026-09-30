from setuptools import setup

package_name = 'drone_gcs_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pymavlink'],
    tests_require=['pytest'],   # sin esto colcon usa unittest y no encuentra los tests de pytest
    zip_safe=True,
    maintainer='Xabi',
    maintainer_email='64962381+XabMS@users.noreply.github.com',
    description='Puente con tierra: confirmación de suelta del piloto por MAVLink (ADR-006).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pilot_confirm_bridge = drone_gcs_bridge.pilot_confirm_bridge:main',
        ],
    },
)
