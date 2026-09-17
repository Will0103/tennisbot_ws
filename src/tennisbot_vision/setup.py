from setuptools import find_packages, setup

import os                                                ## Add launch folder
from glob import glob                                    ##

package_name = 'tennisbot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),   ##
            glob('launch/*.launch.py')),                  ##
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='will',
    maintainer_email='willysaixxx@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "ball_detector = tennisbot_vision.ball_detector:main",
        ],
    },
)
