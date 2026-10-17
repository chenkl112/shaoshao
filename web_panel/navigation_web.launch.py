"""Headless localization and navigation; velocity goes through the Web mode gate."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    root = os.path.join(get_package_share_directory('sentry_bringup'), 'launch')
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(root, 'relocalization.launch.py')),
        launch_arguments={'use_rviz': 'false'}.items())
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(root, 'navigation.launch.py')),
        launch_arguments={'use_sim_time': 'false', 'use_rviz': 'false',
                          'use_respawn': 'false',
                          'output_cmd_vel': '/web/cmd_vel_auto'}.items())
    return LaunchDescription([localization, TimerAction(period=8.0, actions=[navigation])])
