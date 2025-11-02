import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'debug_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='varsenii',
    maintainer_email='varsenyi@gmail.com',
    description='ROS 2 debugging utilities',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'topic_ts_checker = debug_tools.topic_ts_checker:main',
            'topic_latency_checker = debug_tools.topic_latency_checker:main'
        ],
    },
)
