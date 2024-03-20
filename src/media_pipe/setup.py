from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'media_pipe'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'media_pipe_landmarkers'), glob(os.path.join(package_name, '*.task')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmedmuhammad',
    maintainer_email='thisisdeahmed@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'media_pipe = media_pipe.media_pipe:main'
        ],
    },
)
