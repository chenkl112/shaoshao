"""Adapt sentry_planning's room.world collision geometry to Gazebo Harmonic.

The ROS 1 differential-drive robot is replaced by the measured Shaoshao hull.
Walls are preserved; test passages are additional, explicitly named fixtures.
"""
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import yaml

UPSTREAM = 'https://github.com/zzt-180/sentry_planning'
COMMIT = 'b9349d2ada3fe0299ccae9d1e483d5f3b79b1cc8'
SOURCE = Path(__file__).parent/'assets/room.world'


def transform(text):
    x, y, z, roll, pitch, yaw = map(float, (text or '0 0 0 0 0 0').split())
    if abs(roll)+abs(pitch) > 1e-8:
        raise ValueError('Only planar box poses are supported by the scan adapter')
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0., 0., 1.]])


def boxes(world):
    result = []
    for model in world.findall('model'):
        if model.get('name') == 'shaoshao':
            continue
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                box = collision.find('geometry/box/size')
                if box is None:
                    continue
                size = tuple(map(float, box.text.split()))
                matrix = transform(model.findtext('pose'))@transform(link.findtext('pose'))@transform(collision.findtext('pose'))
                result.append((matrix, size))
    return result


def add_box(world, name, center, size):
    model = ET.SubElement(world, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{center[0]} {center[1]} {size[2]/2} 0 0 0'
    link = ET.SubElement(model, 'link', name='body')
    for kind in ('collision', 'visual'):
        shape = ET.SubElement(link, kind, name=kind)
        geometry = ET.SubElement(shape, 'geometry')
        ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = ' '.join(map(str, size))


def make_scene(directory, width=.8, start_y=-2.):
    root = Path(directory)
    root.mkdir(parents=True, exist_ok=True)
    world = ET.Element('world', name='shaoshao_test')
    ET.SubElement(world, 'gravity').text = '0 0 0'
    physics = ET.SubElement(world, 'physics', name='physics', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = '.002'
    ET.SubElement(physics, 'real_time_factor').text = '1'
    for name in ('Physics', 'UserCommands', 'SceneBroadcaster'):
        filename = {'Physics': 'physics', 'UserCommands': 'user-commands', 'SceneBroadcaster': 'scene-broadcaster'}[name]
        ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
    original = ET.parse(SOURCE).getroot().find('world')
    for model in original.findall('model'):
        if model.get('name') not in ('ground_plane', 'room'):
            continue
        adapted = copy.deepcopy(model)
        # Empty Classic pose-frame attributes have no semantics; omit them to
        # avoid Harmonic schema warnings without changing any transform.
        for pose in adapted.iter('pose'):
            if pose.get('frame') == '':
                pose.attrib.pop('frame')
        world.append(adapted)
    for y, name in ((-2-width/2-1.5, 'passage_south'), (-2+width/2+1.5, 'passage_north')):
        add_box(world, name, (2., y), (2., 3., 1.))
    robot = ET.SubElement(world, 'model', name='shaoshao')
    ET.SubElement(robot, 'pose').text = f'0 {start_y} .4 0 0 0'
    link = ET.SubElement(robot, 'link', name='body')
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass').text = '20'
    inertia = ET.SubElement(inertial, 'inertia')
    for key in ('ixx', 'iyy', 'izz'):
        ET.SubElement(inertia, key).text = '1'
    for kind in ('collision', 'visual'):
        geometry = ET.SubElement(ET.SubElement(link, kind, name='hull'), 'geometry')
        cylinder = ET.SubElement(geometry, 'cylinder')
        ET.SubElement(cylinder, 'radius').text = '.297658'
        ET.SubElement(cylinder, 'length').text = '.3'
    velocity = ET.SubElement(robot, 'plugin', filename='gz-sim-velocity-control-system', name='gz::sim::systems::VelocityControl')
    ET.SubElement(velocity, 'topic').text = '/shaoshao/cmd_vel'
    pose = ET.SubElement(robot, 'plugin', filename='gz-sim-pose-publisher-system', name='gz::sim::systems::PosePublisher')
    for key, value in {'publish_link_pose': 'false', 'publish_model_pose': 'true',
                       'use_pose_vector_msg': 'true', 'update_frequency': '50'}.items():
        ET.SubElement(pose, key).text = value
    odom=ET.SubElement(robot,'plugin',filename='gz-sim-odometry-publisher-system',name='gz::sim::systems::OdometryPublisher')
    for key,value in {'odom_frame':'navcar_odom','robot_base_frame':'navcar_center','dimensions':'2',
                      'odom_publish_frequency':'50','odom_topic':'/shaoshao/odometry'}.items():
        ET.SubElement(odom,key).text=value
    sdf = ET.Element('sdf', version='1.9')
    sdf.append(world)
    ET.ElementTree(sdf).write(root/'scene.sdf', encoding='utf-8', xml_declaration=True)
    obstacles = boxes(world)
    resolution, origin, nx, ny = .025, (-7., -5., 0.), 520, 280
    # Rasterize the same oriented collision boxes used by the physics scene.
    xx, yy = np.meshgrid(origin[0]+(np.arange(nx)+.5)*resolution,
                         origin[1]+(np.arange(ny)+.5)*resolution)
    image = np.full((ny, nx), 254, dtype=np.uint8)
    for matrix, size in obstacles:
        dx, dy = xx-matrix[0, 2], yy-matrix[1, 2]
        rx = matrix[0, 0]*dx+matrix[1, 0]*dy
        ry = matrix[0, 1]*dx+matrix[1, 1]*dy
        # Include every cell intersecting the box, rather than erasing thin walls.
        inside = (np.abs(rx) <= size[0]/2+resolution/2) & (np.abs(ry) <= size[1]/2+resolution/2)
        image[inside] = 0
    Image.fromarray(np.flipud(image)).save(root/'site.png')
    (root/'site.yaml').write_text(yaml.safe_dump(dict(image='site.png', resolution=resolution,
        origin=list(origin), negate=0, occupied_thresh=.65, free_thresh=.196, mode='trinary')))
    return obstacles


def ranges_at(pose, obstacles, beams=720):
    x, y, yaw = pose
    angle = np.arange(beams)*2*math.pi/beams-math.pi+yaw
    direction = np.column_stack((np.cos(angle), np.sin(angle)))
    distances = np.full(beams, 15.)
    for matrix, size in obstacles:
        rotation = matrix[:2, :2]
        origin = (np.array([x, y])-matrix[:2, 2])@rotation
        rays = direction@rotation
        inv = np.divide(1., rays, out=np.full_like(rays, np.inf), where=np.abs(rays) > 1e-10)
        a, b = (-np.array(size[:2])/2-origin)*inv, (np.array(size[:2])/2-origin)*inv
        enter, leave = np.min([a, b], axis=0).max(axis=1), np.max([a, b], axis=0).min(axis=1)
        hit = np.where(enter > 0, enter, leave)
        distances = np.minimum(distances, np.where((leave >= enter) & (hit > 0), hit, 15.))
    return distances


def clearance(pose, obstacles):
    result = math.inf
    for matrix, size in obstacles:
        point = (np.asarray(pose[:2])-matrix[:2, 2])@matrix[:2, :2]
        delta = np.abs(point)-np.asarray(size[:2])/2
        distance = np.linalg.norm(np.maximum(delta, 0.))+min(float(np.max(delta)), 0.)
        result = min(result, float(distance)-.297658)
    return result
