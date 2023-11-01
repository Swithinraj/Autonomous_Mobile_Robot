from setuptools import setup
import os
from glob import glob

package_name = 'autonomous_map_navigate'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kishansawant',
    maintainer_email='kishansawant96@gmail.com',
    description='autonomous mapping by wall following and navigating with safety features using behavior tree',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collison_avoidance = autonomous_map_navigate.collison_avoidance:main',
            'battery_monitor = autonomous_map_navigate.battery_monitor:main',
        ],
    },
)
