from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'face_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools','rclpy', 'cv_bridge', 'deepface',],
    zip_safe=True,
    maintainer='varsenii',
    maintainer_email='varsenyi@gmail.com',
    description='TFace recognition package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_recognition = face_recognition.face_recognition:main'
        ],
    },
)
