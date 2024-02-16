import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # robot_description_config = load_file('ur_description', 'urdf/ur5.urdf')
    # robot_description = {'robot_description' : robot_description_config}

    # skiros_config_file = get_package_share_directory('skiros2') + "/cfg/skiros_config.yaml"
    wm_config = {
        "workspace_dir": get_package_share_directory('skiros2') + "/owl",
        "init_scene": "",
        "verbose": False,
        # "reasoners_pkgs": ["skiros2_std_reasoners"],
        # "load_contexts": [],
    }

    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'world'])

    wm = Node(package='skiros2_world_model',
              executable='world_model_server_node',
              name='wm',
              parameters=[wm_config],
              output='screen')

    gui = Node(package='rqt_gui',
               executable='rqt_gui',
               # arguments=["-s skiros2_gui.gui.Skiros"], # does not find the plugin. Not sure why - it appears in the list and can be selected with "ros2 run"
               output='screen')

    return LaunchDescription([static_tf, wm, gui])
