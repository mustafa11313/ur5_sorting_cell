"""Run the simulation with the randomized 'dataset' scenario and record a labelled YOLO dataset.

    ros2 launch sorting_cell_perception record_dataset.launch.py output_dir:=$HOME/sorting_dataset \
        max_images:=600 world_overrides:="sun=0.3 lamp=0.15 ambient=0.2" prefix:=dim
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('sorting_cell_bringup')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim.launch.py')),
        launch_arguments={
            'scenario': LaunchConfiguration('scenario'),
            'headless': LaunchConfiguration('headless'),
            'seed': LaunchConfiguration('seed'),
            'world_overrides': LaunchConfiguration('world_overrides'),
        }.items())
    recorder = Node(
        package='sorting_cell_perception', executable='dataset_recorder', output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_dir': LaunchConfiguration('output_dir'),
            'max_images': LaunchConfiguration('max_images'),
            'prefix': LaunchConfiguration('prefix'),
            'cell_config': os.path.join(bringup, 'config', 'cell.yaml'),
        }])
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='dataset'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('seed', default_value='7'),
        DeclareLaunchArgument('world_overrides', default_value=''),
        DeclareLaunchArgument('output_dir',
                              default_value=os.path.join(os.path.expanduser('~'), 'sorting_dataset')),
        DeclareLaunchArgument('max_images', default_value='600'),
        DeclareLaunchArgument('prefix', default_value='img'),
        sim,
        recorder,
        RegisterEventHandler(OnProcessExit(target_action=recorder,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ])
