from setuptools import setup
import os
from glob import glob

package_name = 'skiros2'

setup(
    name=package_name,
    packages=[],
    version='1.0.5',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name, "owl"), glob('owl/*.owl')),
        (os.path.join('share', package_name, "owl/IEEE-1872-2015"), glob('owl/IEEE-1872-2015/*.owl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
)
