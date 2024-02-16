from setuptools import setup

package_name = 'skiros2_common'

setup(
    name=package_name,
    version='1.0.5',
    packages=[package_name, package_name + "/core", package_name + "/ros", package_name + "/tools"],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
)
