"""Shared Navcar navigation settings, expressed in metres and seconds."""
import math
from pathlib import Path

import yaml


def navigation_options(config):
    settings=config.get('navigation',{})
    if not isinstance(settings,dict):raise ValueError('navigation 必须是配置对象')
    narrow=settings.get('narrow_passage',False)
    if type(narrow) is not bool:raise ValueError('navigation.narrow_passage 必须为 true/false')
    reference=settings.get('reference_point','imu')
    if reference not in ('imu','yaw_axis'):raise ValueError('navigation.reference_point 必须为 imu/yaw_axis')
    if narrow and reference!='yaw_axis':raise ValueError('窄通道需要已标定的 yaw_axis 中心参考点')
    return {'narrow':narrow,'reference':reference,'resolution':.01 if narrow else .05,
            'min_range':.2 if narrow else .5,'position_std_limit':.01 if narrow else .3}


def navigation_speed(config):
    settings=config.get('navigation',{})
    if not isinstance(settings,dict):raise ValueError('navigation 必须是配置对象')
    value=settings.get('max_speed_mps',.1)
    if type(value) not in (int,float) or not math.isfinite(value) or not 0 < value <= 3:
        raise ValueError('navigation.max_speed_mps 必须大于 0 且不超过 3 m/s')
    # Narrow-passage commissioning uses the same effective limit at all layers.
    return min(float(value),.05) if navigation_options(config)['narrow'] else float(value)


def load_navigation_speed(path):
    path=Path(path)
    # Keep standalone/manual controllers and older installations compatible.
    if not path.exists():return .1
    config=yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(config,dict):raise ValueError('robot.yaml 必须是配置对象')
    return navigation_speed(config)


def limit_translation(vx,vy,limit):
    scale=min(1.,limit/math.hypot(vx,vy)) if vx or vy else 1.
    return vx*scale,vy*scale
