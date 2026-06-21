import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'ball_detection'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)],
        ),
        (
            os.path.join('share', package_name),
            ['package.xml'] + glob(os.path.join(package_name, '*.rknn')),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
    ],
    install_requires=['setuptools'],
    include_package_data=True,
    zip_safe=True,
    maintainer='zou1043',
    maintainer_email='',
    description='Ball detection and tracked vehicle following package for Orange Pi',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_node = ball_detection.detect_node:main',
            'crtl_node = ball_detection.crtl_node:main',
            'stm_node = ball_detection.stm_node:main',
            'check_node = ball_detection.check_node:main',
            'ball_track_logic = ball_detection.ball_track_logic:main',
            'ball_mission_logic = ball_detection.ball_mission_logic:main',
            'esp32_serial_node = ball_detection.esp32_serial_node:main',
        ],
    },
)
