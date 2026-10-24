"""Shared Navcar navigation settings, expressed in metres and seconds."""
import math
from pathlib import Path

import yaml


def navigation_options(config):
    settings=config.get('navigation',{})
    if not isinstance(settings,dict):raise ValueError('navigation 必须是配置对象')
    reference=settings.get('reference_point','imu')
    if reference not in ('imu','yaw_axis'):raise ValueError('navigation.reference_point 必须为 imu/yaw_axis')
    enter=settings.get('localization_max_position_std_m',.05)
    stop=settings.get('localization_stop_position_std_m',.075)
    if (type(enter) not in (int,float) or not math.isfinite(enter) or not .001 <= enter <= .1):
        raise ValueError('navigation.localization_max_position_std_m 必须在 0.001 到 0.1 米之间')
    if (type(stop) not in (int,float) or not math.isfinite(stop) or not enter <= stop <= .15):
        raise ValueError('navigation.localization_stop_position_std_m 必须不小于进入门槛且不超过 0.15 米')
    return {'reference':reference,'resolution':.025,'min_range':.2,
            'position_std_limit':float(enter),'position_std_stop_limit':float(stop)}


def navigation_speed(config):
    settings=config.get('navigation',{})
    if not isinstance(settings,dict):raise ValueError('navigation 必须是配置对象')
    value=settings.get('max_speed_mps',.5)
    if type(value) not in (int,float) or not math.isfinite(value) or not 0 < value <= 3:
        raise ValueError('navigation.max_speed_mps 必须大于 0 且不超过 3 m/s')
    # Nav2, forwarding and UART share the same global limit.
    return min(float(value),.5)


def load_navigation_speed(path):
    path=Path(path)
    # Keep standalone/manual controllers and older installations compatible.
    if not path.exists():return .5
    config=yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(config,dict):raise ValueError('robot.yaml 必须是配置对象')
    return navigation_speed(config)


def limit_translation(vx,vy,limit):
    scale=min(1.,limit/math.hypot(vx,vy)) if vx or vy else 1.
    return vx*scale,vy*scale
