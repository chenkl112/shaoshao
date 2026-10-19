"""Nav2 localization and translation using calibrated sensor/reference frames."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    root=Path.home()/'ccc/navigation_runtime'
    params=str(root/'nav2.yaml')
    nodes=[Node(package='fast_lio',executable='fastlio_mapping',name='navigation_lio',
                parameters=[str(root/'fast_lio.yaml')],
                remappings=[('/Odometry','/navcar/state_estimation')],output='screen'),
           Node(package='pointcloud_to_laserscan',executable='pointcloud_to_laserscan_node',
                name='navigation_scan',parameters=[params],
                remappings=[('cloud_in','/cloud_registered_body'),('scan','/navcar/scan')],output='screen')]
    frames_file=root/'frames.yaml'
    if frames_file.is_file():
        for transform in yaml.safe_load(frames_file.read_text())['static_transforms']:
            x,y,z=transform['xyz']
            nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',
                              name='navigation_'+transform['child'],arguments=[
                                  '--frame-id','navcar_imu','--child-frame-id',transform['child'],
                                  '--x',str(x),'--y',str(y),'--z',str(z),'--yaw','0','--pitch','0','--roll','0']))
    for package,executable,name in [('nav2_map_server','map_server','map_server'),
                                     ('nav2_amcl','amcl','amcl'),
                                     ('nav2_planner','planner_server','planner_server'),
                                     ('nav2_controller','controller_server','controller_server'),
                                     ('nav2_bt_navigator','bt_navigator','bt_navigator')]:
        nodes.append(Node(package=package,executable=executable,name=name,parameters=[params],
                          remappings=[('cmd_vel','/navcar/cmd_vel_auto')],output='screen'))
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
                      name='navigation_lifecycle',parameters=[{'autostart':True,'use_sim_time':False,
                      'node_names':['map_server','amcl','planner_server','controller_server','bt_navigator']}],output='screen'))
    handlers=[RegisterEventHandler(OnProcessExit(target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason='A required navigation node exited'))])) for node in nodes]
    return LaunchDescription(handlers+nodes)
