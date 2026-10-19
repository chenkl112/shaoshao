"""Shared Navcar navigation settings, expressed in metres and seconds."""
import math
from pathlib import Path

import yaml


def navigation_speed(config):
    settings=config.get('navigation',{})
    if not isinstance(settings,dict):raise ValueError('navigation 必须是配置对象')
    value=settings.get('max_speed_mps',.1)
    if type(value) not in (int,float) or not math.isfinite(value) or not 0 < value <= 3:
        raise ValueError('navigation.max_speed_mps 必须大于 0 且不超过 3 m/s')
    return float(value)


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
