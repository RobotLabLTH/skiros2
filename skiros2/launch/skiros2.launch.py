import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


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
    libraries_list = LaunchConfiguration('libraries_list')
    skill_list = LaunchConfiguration('skill_list')
    robot_name = LaunchConfiguration('robot_name')
    init_scene = LaunchConfiguration('init_scene')
    verbose = LaunchConfiguration('verbose')
    workspace_dir = LaunchConfiguration('workspace_dir')
    robot_ontology_prefix = LaunchConfiguration('robot_ontology_prefix')

    libraries_list_arg = DeclareLaunchArgument(
        'libraries_list',
        default_value="['']"
    )
    skill_list_arg = DeclareLaunchArgument(
        'skill_list',
        default_value="['']"
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='test_robot'
    )
    init_scene_arg = DeclareLaunchArgument(
        'init_scene',
        default_value=''
    )
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false'
    )
    workspace_dir_arg = DeclareLaunchArgument(
        'workspace_dir',
        default_value=get_package_share_directory('skiros2') + "/owl"
    )
    robot_ontology_prefix_arg = DeclareLaunchArgument(
        'robot_ontology_prefix',
        default_value='skiros'
    )


    # skiros_config_file = get_package_share_directory('skiros2') + "/cfg/skiros_config.yaml"
    wm_config = {
        "workspace_dir": workspace_dir,
        "init_scene": init_scene,
        "verbose": verbose,
        "reasoners_pkgs": ["skiros2_std_reasoners"],
        # "load_contexts": [],
    }

    skill_mgr_config = {
        "prefix": robot_ontology_prefix,
        "verbose": verbose,
        "libraries_list": libraries_list,
        "skill_list": skill_list,
        "robot_name": robot_name,
        "deploy": False,
    }

    task_mgr_config = {
        "verbose": verbose,
    }

    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'world'])

    wm = Node(package='skiros2_world_model',
              executable='world_model_server_node',
              # Note: the name can not be set! The WM initializes two nodes "/wm" and "/skiros_element". If this is set, these names are overwritten and both are called "wm"
              # name='wm',
              parameters=[wm_config],
              output='screen')

    gui = Node(package='rqt_gui',
               executable='rqt_gui',
               arguments=["-s", "skiros2_gui"],
               output='screen')
    
    skill_mgr = Node(package='skiros2_skill',
                     executable='skill_manager_node',
                     output='screen',
                     respawn=True,
                     parameters=[skill_mgr_config])

    task_mgr = Node(package='skiros2_task',
                     executable='task_manager_node',
                     output='screen',
                     respawn=True,
                     parameters=[task_mgr_config])

    return LaunchDescription([libraries_list_arg,
                              skill_list_arg,
                              robot_name_arg,
                              init_scene_arg,
                              verbose_arg,
                              workspace_dir_arg,
                              robot_ontology_prefix_arg,
                              static_tf,
                              wm,
                              gui,
                              skill_mgr,
                              task_mgr])
