from setuptools import setup
from glob import glob
import os

package_name = 'dobot_rviz_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('dobot_rviz_demo/launch/*.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('dobot_rviz_demo/rviz/*')),
        (os.path.join('share', package_name, 'scripts'),
            glob('dobot_rviz_demo/scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdulhamid',
    maintainer_email='your_email@example.com',
    description='Dobot RViz demonstration package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
