"""Gazebo simulation of the sorting cell: world, robot, controllers, bridge, camera TF.

    ros2 launch sorting_cell_bringup sim.launch.py scenario:=touching headless:=true
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sorting_cell_bringup.world_builder import load_scenario, parse_overrides, render_world


def _launch_setup(context):
    bringup = get_package_share_directory('sorting_cell_bringup')
    description = get_package_share_directory('sorting_cell_description')
    gazebo_pkg = get_package_share_directory('sorting_cell_gazebo')

    scenario = LaunchConfiguration('scenario').perform(context)
    scenario_path = scenario if scenario.endswith('.yaml') else os.path.join(
        bringup, 'config', 'scenarios', scenario + '.yaml')
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'

    with open(os.path.join(bringup, 'config', 'cell.yaml')) as f:
        cell = yaml.safe_load(f)['cell']
    params = load_scenario(scenario_path)
    params['seed'] = LaunchConfiguration('seed').perform(context)
    params['max_parts'] = LaunchConfiguration('max_parts').perform(context)
    params['spawn_enabled'] = LaunchConfiguration('spawn_enabled').perform(context)
    params['render_engine'] = LaunchConfiguration('render_engine').perform(context)
    params['camera_rate'] = LaunchConfiguration('camera_rate').perform(context)
    params.update(parse_overrides(LaunchConfiguration('world_overrides').perform(context)))
    world = render_world(os.path.join(gazebo_pkg, 'worlds', 'sorting_cell.sdf.template'),
                         cell, params)

    gz_args = f'-r -v 2 {world}'
    if headless:
        gz_args = '-s --headless-rendering ' + gz_args
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    xacro_file = os.path.join(description, 'urdf', 'sorting_cell.urdf.xacro')
    controllers = os.path.join(bringup, 'config', 'controllers.yaml')
    import xacro  # noqa: E402  (only needed at launch time)
    robot_description = xacro.process_file(
        xacro_file, mappings={'sim_ignition': 'true', 'controllers_file': controllers}).toxml()

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'ur5', '-allow_renaming', 'false'])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(bringup, 'config', 'bridge.yaml'),
                     'use_sim_time': True}])

    # camera extrinsics (world -> camera_link) and the optical frame convention
    with open(os.path.join(bringup, 'config', 'camera_extrinsics.yaml')) as f:
        cam = yaml.safe_load(f)['camera_link']
    extrinsics_file = LaunchConfiguration('camera_extrinsics').perform(context)
    if extrinsics_file:
        with open(extrinsics_file) as f:
            cam = yaml.safe_load(f)['camera_link']
    x, y, z = cam['xyz']
    r, p, yaw = cam['rpy']
    camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='camera_link_tf',
        arguments=['--x', str(x), '--y', str(y), '--z', str(z), '--roll', str(r),
                   '--pitch', str(p), '--yaw', str(yaw),
                   '--frame-id', cam.get('parent', 'world'), '--child-frame-id', 'camera_link'],
        parameters=[{'use_sim_time': True}])
    optical_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='camera_optical_tf',
        arguments=['--roll', str(-math.pi / 2), '--yaw', str(-math.pi / 2),
                   '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame'],
        parameters=[{'use_sim_time': True}])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '120'])

    jsb = spawner('joint_state_broadcaster')
    arm = spawner('arm_controller')
    gripper = spawner('gripper_controller')

    return [
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', os.pathsep.join(
            filter(None, [os.path.join(os.path.dirname(os.path.dirname(gazebo_pkg)), 'lib',
                                       'sorting_cell_gazebo'),
                          os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')]))),
        gazebo, rsp, spawn, bridge, camera_tf, optical_tf,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, gripper])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='nominal',
                              description='Scenario name in config/scenarios or a YAML path'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo server only (no GUI)'),
        DeclareLaunchArgument('seed', default_value='42', description='Part spawner seed'),
        DeclareLaunchArgument('max_parts', default_value='0',
                              description='Stop spawning after N parts (0 = unlimited)'),
        DeclareLaunchArgument('spawn_enabled', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value='ogre2',
                              description='ogre2 (default) or ogre (for old/virtual GPUs)'),
        DeclareLaunchArgument('camera_rate', default_value='10'),
        DeclareLaunchArgument('camera_extrinsics', default_value='',
                              description='Optional calibrated extrinsics YAML'),
        DeclareLaunchArgument('world_overrides', default_value='',
                              description='Extra world params, e.g. "sun=0.3 belt_speed=0.12"'),
        OpaqueFunction(function=_launch_setup),
    ])
