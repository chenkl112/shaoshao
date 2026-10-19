"""Shared Navcar navigation settings, expressed in metres and seconds."""
import math
from pathlib import Path

import yaml


def navigation_options(config):
    settings=config.get('navigation',{})
    if not isinstance(settings,dict):raise ValueError('navigation 必须是配置对象')
    reference=settings.get('reference_point','imu')
    if reference not in ('imu','yaw_axis'):raise ValueError('navigation.reference_point 必须为 imu/yaw_axis')
    return {'reference':reference,'resolution':.01,'min_range':.2,'position_std_limit':.01}


def navigation_speed(config):
    settings=config.get('navigation',{})
    if not isinstance(settings,dict):raise ValueError('navigation 必须是配置对象')
    value=settings.get('max_speed_mps',.3)
    if type(value) not in (int,float) or not math.isfinite(value) or not 0 < value <= 3:
        raise ValueError('navigation.max_speed_mps 必须大于 0 且不超过 3 m/s')
    # Nav2, forwarding and UART share the same global limit.
    return min(float(value),.3)


def load_navigation_speed(path):
    path=Path(path)
    # Keep standalone/manual controllers and older installations compatible.
    if not path.exists():return .3
    config=yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(config,dict):raise ValueError('robot.yaml 必须是配置对象')
    return navigation_speed(config)


def limit_translation(vx,vy,limit):
    scale=min(1.,limit/math.hypot(vx,vy)) if vx or vy else 1.
    return vx*scale,vy*scale
