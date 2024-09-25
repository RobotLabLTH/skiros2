from setuptools import setup

package_name = 'skiros2_task'

setup(
    name=package_name,
    version='1.0.5',
    packages=[package_name, package_name + "/core", package_name + "/nodes", package_name + "/ros"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'test_domain = {package_name}.nodes.utils.test_domain:main',
            f'task_manager_node = {package_name}.nodes.task_manager_node:main'
        ],
    },
)
