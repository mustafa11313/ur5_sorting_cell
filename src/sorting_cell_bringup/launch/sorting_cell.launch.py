"""The complete sorting cell: simulation, MoveIt 2, perception, task logic and metrics.

    ros2 launch sorting_cell_bringup sorting_cell.launch.py
    ros2 launch sorting_cell_bringup sorting_cell.launch.py scenario:=touching backend:=classical
    # headless evaluation run that exits after 10 simulated minutes:
    ros2 launch sorting_cell_bringup sorting_cell.launch.py headless:=true rviz:=false \
        eval_duration:=600 results_dir:=results/nominal
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM_ARGS = ['scenario', 'headless', 'seed', 'max_parts', 'spawn_enabled', 'render_engine',
            'camera_rate', 'camera_extrinsics', 'world_overrides']


def _setup(context):
    bringup = get_package_share_directory('sorting_cell_bringup')
    perception = get_package_share_directory('sorting_cell_perception')
    cell_config = os.path.join(bringup, 'config', 'cell.yaml')
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    scenario = arg('scenario')
    scenario_path = scenario if scenario.endswith('.yaml') else os.path.join(
        bringup, 'config', 'scenarios', scenario + '.yaml')
    with open(scenario_path) as f:
        world = (yaml.safe_load(f) or {}).get('world', {})
    belt_speed = float(world.get('belt_speed', 0.08))
    for token in arg('world_overrides').split():
        if token.startswith('belt_speed='):
            belt_speed = float(token.split('=', 1)[1])

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim.launch.py')),
        launch_arguments={k: LaunchConfiguration(k) for k in SIM_ARGS}.items())
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'moveit.launch.py')),
        launch_arguments={'rviz': LaunchConfiguration('rviz'), 'use_sim_time': 'true'}.items())

    model_path = arg('model_path') or os.path.join(perception, 'models', 'parts_yolo.pt')
    detector = Node(
        package='sorting_cell_perception', executable='detector_node', output='screen',
        parameters=[{'use_sim_time': True, 'backend': arg('backend'), 'model_path': model_path,
                     'device': arg('device'), 'cell_config': cell_config}])
    scene = Node(
        package='sorting_cell_task', executable='planning_scene', output='screen',
        parameters=[{'use_sim_time': True, 'cell_config': cell_config}])
    task = Node(
        package='sorting_cell_task', executable='sorting_task', output='screen',
        parameters=[{'use_sim_time': True, 'cell_config': cell_config,
                     'belt_speed': belt_speed}])
    results_dir = arg('results_dir') or os.path.join(os.path.expanduser('~'), 'sorting_results',
                                                     os.path.basename(scenario).replace('.yaml', ''))
    duration = float(arg('eval_duration'))
    metrics = Node(
        package='sorting_cell_task', executable='metrics_node', output='screen',
        parameters=[{'use_sim_time': True, 'results_dir': results_dir,
                     'scenario': os.path.basename(scenario).replace('.yaml', ''),
                     'duration': duration}])
    actions = [sim, moveit, detector, scene, task, metrics]
    if duration > 0:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=metrics, on_exit=[EmitEvent(event=Shutdown(reason='evaluation done'))])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='nominal'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('max_parts', default_value='0'),
        DeclareLaunchArgument('spawn_enabled', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value='ogre2'),
        DeclareLaunchArgument('camera_rate', default_value='10'),
        DeclareLaunchArgument('camera_extrinsics', default_value=''),
        DeclareLaunchArgument('world_overrides', default_value=''),
        DeclareLaunchArgument('backend', default_value='yolo',
                              description='yolo (falls back to classical without weights) or classical'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('results_dir', default_value=''),
        DeclareLaunchArgument('eval_duration', default_value='0',
                              description='Simulated seconds to evaluate, then shut down (0 = forever)'),
        OpaqueFunction(function=_setup),
    ])
