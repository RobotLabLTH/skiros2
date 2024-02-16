from setuptools import setup

package_name = 'skiros2_world_model'

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
            f'ontology_server_node = {package_name}.nodes.ontology_server_node:main',
            f'world_model_server_node = {package_name}.nodes.world_model_server_node:main',
            f'view_scene = {package_name}.nodes.utils.view_scene:main',
            f'edit_scene = {package_name}.nodes.utils.edit_scene:main',
            f'simple_query = {package_name}.nodes.utils.simple_query:main',
        ],
    },
)
