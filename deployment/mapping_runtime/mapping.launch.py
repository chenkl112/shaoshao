from pathlib import Path
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    root=Path.home()/'ccc'
    lio=yaml.safe_load((root/'mapping_runtime/fast_lio.yaml').read_text())
    resolution=lio['/**']['ros__parameters'].get('map_filter',{}).get('static_resolution',.01)
    nodes=[
      Node(package='fast_lio',executable='fastlio_mapping',output='screen',parameters=[str(root/'mapping_runtime/fast_lio.yaml')],remappings=[('/Odometry','/state_estimation')]),
      Node(package='terrain_analysis_ext',executable='terrainAnalysisExt',output='screen',parameters=[{
        'map_frame':'odom','scanVoxelSize':.1,'decayTime':.5,'noDecayDis':0.,'clearingDis':30.,
        'useSorting':True,'quantileZ':.25,'vehicleHeight':1.5,'voxelPointUpdateThre':100,'voxelTimeUpdateThre':2.,
        'lowerBoundZ':-3.,'upperBoundZ':1.,'disRatioZ':.1,'checkTerrainConn':False,'localTerrainMapRadius':0.,
        'vehicleLength':.7,'vehicleWidth':.7}],remappings=[('/registered_scan','/cloud_registered')]),
      Node(package='tf2_ros',executable='static_transform_publisher',name='mapping_map_odom',arguments=['--frame-id','map','--child-frame-id','odom','--qx','0','--qw','1']),
      Node(package='fast_lio',executable='static_map_projector',output='screen',parameters=[{'resolution':float(resolution)}]),
    ]
    return LaunchDescription(nodes)
