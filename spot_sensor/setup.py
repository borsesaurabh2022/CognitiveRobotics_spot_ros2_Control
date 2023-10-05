from setuptools import find_packages, setup

package_name = 'spot_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ekart',
    maintainer_email='saurabh.borse@alumni.fh-aachen.de',
    description='spot sensor control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_range_sensor = spot_sensor.spot_range_sensor:main',
            'spot_laser_sensor = spot_sensor.spot_laser_sensor:main',
        ],
    },
)
