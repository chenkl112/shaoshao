import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration 
from launch.conditions import IfCondition

def generate_launch_description():

  config_path = os.path.join(
      get_package_share_directory('sentry_bringup'), 'params') 

  
  # fast-lio localization   
  fast_lio_param = os.path.join(config_path, 'fast_lio_mapping_param.yaml')
  with open(fast_lio_param, encoding='utf-8') as stream:
      ceiling = yaml.safe_load(stream)['/**']['ros__parameters'].get('map_filter', {})
  fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
          fast_lio_param,
          {'map_filter.enabled': ParameterValue(LaunchConfiguration('map_ceiling_enabled'), value_type=bool),
           'map_filter.dynamic_enabled': ParameterValue(LaunchConfiguration('map_dynamic_enabled'), value_type=bool),
           'map_filter.max_height': ParameterValue(LaunchConfiguration('map_ceiling_height'), value_type=float)}
        ],
        output='screen',
        remappings=[('/Odometry','/state_estimation')]
    )

  start_octomap_server = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('sentry_bringup'), 'launch', 'octomap_server_intensity.launch.py')]),
    launch_arguments={'map_dynamic_enabled': LaunchConfiguration('map_dynamic_enabled')}.items()
  )
        
  rviz_config_file = os.path.join(
    get_package_share_directory('sentry_bringup'), 'rviz', 'loam_livox.rviz')
  start_rviz = Node(
    condition=IfCondition(LaunchConfiguration('use_rviz')),
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file,'--ros-args', '--log-level', 'warn'],
    output='screen'
  )

  delayed_start_mapping = TimerAction(
    period=8.0,
    actions=[
      fast_lio_node,
      start_octomap_server
    ]
  )

  ld = LaunchDescription()
  ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))
  ld.add_action(DeclareLaunchArgument('map_ceiling_enabled', default_value=str(ceiling.get('enabled', True)).lower()))
  ld.add_action(DeclareLaunchArgument('map_ceiling_height', default_value=str(ceiling.get('max_height', 1.5))))

  ld.add_action(DeclareLaunchArgument('map_dynamic_enabled', default_value=str(ceiling.get('dynamic_enabled', True)).lower()))

  ld.add_action(start_rviz)
  ld.add_action(delayed_start_mapping)

  return ld
