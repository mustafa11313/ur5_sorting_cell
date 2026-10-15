"""MoveIt 2 move_group (+ optional RViz) for the sorting cell robot."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def moveit_config(sim_ignition='true'):
    description = get_package_share_directory('sorting_cell_description')
    controllers = os.path.join(get_package_share_directory('sorting_cell_bringup'), 'config',
                               'controllers.yaml')
    return (
        MoveItConfigsBuilder('ur5', package_name='sorting_cell_moveit_config')
        .robot_description(
            file_path=os.path.join(description, 'urdf', 'sorting_cell.urdf.xacro'),
            mappings={'sim_ignition': sim_ignition, 'controllers_file': controllers})
        .robot_description_semantic(file_path='config/ur5.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .planning_pipelines(default_planning_pipeline='ompl',
                            pipelines=['ompl', 'pilz_industrial_motion_planner'])
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .to_moveit_configs())


def _setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    config = moveit_config('true' if use_sim_time else 'false')
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[config.to_dict(), {'use_sim_time': use_sim_time,
                                       'publish_planning_scene_hz': 10.0}])
    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', os.path.join(get_package_share_directory('sorting_cell_bringup'),
                                      'rviz', 'sorting_cell.rviz')],
        parameters=[config.robot_description, config.robot_description_semantic,
                    config.robot_description_kinematics, config.planning_pipelines,
                    config.joint_limits, {'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')))
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
