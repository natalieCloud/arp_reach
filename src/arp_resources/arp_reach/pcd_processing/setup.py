import os
from glob import glob
from setuptools import setup

package_name = 'pcd_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Natalie Chmura',
    author_email='ntchmura@gmail.com',
    maintainer='Natalie Chmura',
    maintainer_email='ntchmura@gmail.com',
    keywords=['ROS 2', 'ARP'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: Apache 2.0',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='This package creates a ROS service that translates a PoseArray[] to a PCD file',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'service = pcd_processing.PoseArrayToPCDService:main',
            'client = pcd_processing.PoseArrayToPCDClient:main',
        ],
    },
)
