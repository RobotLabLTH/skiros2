from setuptools import setup

package_name = 'skiros2_skill'

setup(
    name=package_name,
    version='1.0.5',
    packages=[package_name, package_name + "/core", package_name + "/nodes", package_name + "/ros"],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'skill_manager_node = {package_name}.nodes.skill_manager_node:main',
            f'cmd = {package_name}.nodes.utils.cmd:main',
        ],
    },
)
