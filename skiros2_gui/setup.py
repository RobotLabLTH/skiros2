from setuptools import setup
from glob import glob

package_name = 'skiros2_gui'

setup(
    name=package_name,
    version='1.0.5',
    packages=[package_name, package_name + "/core"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('share/' + package_name + "/core/imgs", glob(f'{package_name}/core/imgs/*.png')),
        ('share/' + package_name + "/core", glob(f'{package_name}/core/*.ui')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
)
