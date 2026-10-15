"""Render the Gazebo world for a scenario.

The world SDF in sorting_cell_gazebo is a template; this module fills it in
from a scenario YAML (plus command line overrides) so that every experiment is
fully described by one small config file.
"""
import os
import string
import tempfile

import yaml

# Values every scenario may override.
DEFAULTS = {
    'belt_speed': 0.08,
    'spawn_interval': 7.0,
    'spawn_enabled': 'true',
    'defect_ratio': 0.25,
    'touching_probability': 0.0,
    'scale_jitter': 0.0,
    'colour_jitter': 0.0,
    'slip_probability': 0.0,
    'max_parts': 0,
    'seed': 42,
    'part_types': 'cube cylinder hex_nut',
    'sun': 0.8,
    'lamp': 0.5,
    'ambient': 0.45,
    'sun_direction': '-0.3 0.2 -1.0',
    'image_noise': 0.005,
    'camera_rate': 10,
    'camera_pose': '0 0.6 1.55 0 1.5708 1.5708',
    'render_engine': 'ogre2',
    'showcase_camera': 'false',
    # wrist_3_link -> tcp distance, must match gripper.xacro (gripper_tcp_z)
    'tcp_offset_z': 0.105,
}

BIN_COLOURS = {
    'bin_cube': (0.15, 0.35, 0.85),
    'bin_cylinder': (0.95, 0.7, 0.1),
    'bin_hex_nut': (0.55, 0.57, 0.6),
    'bin_reject': (0.85, 0.12, 0.05),
}


SHOWCASE_SDF = '''<model name="showcase_camera"><static>true</static>
      <pose>1.25 -1.05 1.75 0 0.55 2.36</pose>
      <link name="link"><sensor name="showcase" type="camera">
        <topic>showcase/image</topic><update_rate>10</update_rate><always_on>true</always_on>
        <camera><horizontal_fov>1.05</horizontal_fov>
          <image><width>960</width><height>540</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>12</far></clip></camera>
      </sensor></link></model>'''


def load_scenario(path):
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get('world', {})


def parse_overrides(text):
    """Parse 'key=value key2=value2' (values may not contain spaces; use ',' for vectors)."""
    out = {}
    for token in (text or '').split():
        if '=' not in token:
            raise ValueError(f'bad world override {token!r}, expected key=value')
        key, value = token.split('=', 1)
        out[key] = value.replace(',', ' ')
    return out


def bin_sdf(name, x, y, floor_z, inner=0.18, wall=0.01, height=0.12):
    """An open box (floor + 4 walls) as a static SDF model."""
    r, g, b = BIN_COLOURS.get(name, (0.5, 0.5, 0.5))
    mat = (f'<material><ambient>{r*0.6} {g*0.6} {b*0.6} 1</ambient>'
           f'<diffuse>{r} {g} {b} 1</diffuse></material>')
    outer = inner + 2 * wall
    parts = [
        ('floor', (0, 0, wall / 2), (outer, outer, wall)),
        ('wall_n', (0, inner / 2 + wall / 2, height / 2), (outer, wall, height)),
        ('wall_s', (0, -inner / 2 - wall / 2, height / 2), (outer, wall, height)),
        ('wall_e', (inner / 2 + wall / 2, 0, height / 2), (wall, inner, height)),
        ('wall_w', (-inner / 2 - wall / 2, 0, height / 2), (wall, inner, height)),
    ]
    elems = []
    for pname, (px, py, pz), (sx, sy, sz) in parts:
        geo = f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
        pose = f'<pose>{px} {py} {pz} 0 0 0</pose>'
        elems.append(f'<collision name="{pname}">{pose}{geo}</collision>')
        elems.append(f'<visual name="{pname}">{pose}{geo}{mat}</visual>')
    return (f'<model name="{name}"><static>true</static><pose>{x} {y} {floor_z} 0 0 0</pose>'
            f'<link name="link">{"".join(elems)}</link></model>')


def marker_sdf(marker_id, x, y, top_z, yaw, texture, plate=0.10):
    """A flat plate showing an ArUco marker texture (marker = 80 % of the plate)."""
    geo = f'<geometry><box><size>{plate} {plate} 0.004</size></box></geometry>'
    mat = (f'<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>'
           f'<pbr><metal><albedo_map>{texture}</albedo_map><roughness>1.0</roughness>'
           f'<metalness>0.0</metalness></metal></pbr></material>')
    post = ('<visual name="post"><pose>0 0 -0.03 0 0 0</pose><geometry><box><size>0.03 0.03 0.056'
            '</size></box></geometry><material><ambient>0.3 0.3 0.3 1</ambient>'
            '<diffuse>0.35 0.35 0.35 1</diffuse></material></visual>')
    return (f'<model name="aruco_{marker_id}"><static>true</static>'
            f'<pose>{x} {y} {top_z - 0.002} 0 0 {yaw}</pose><link name="link">'
            f'<visual name="plate">{geo}{mat}</visual>{post}</link></model>')


def render_world(template_path, cell, params, out_dir=None):
    """Return the path of a rendered world file."""
    values = dict(DEFAULTS)
    values.update(params)
    values['bins_sdf'] = '\n    '.join(
        bin_sdf(name, *xyz) for name, xyz in cell['bins'].items())
    values['showcase_sdf'] = SHOWCASE_SDF if str(values['showcase_camera']).lower() == 'true' else ''
    media = os.path.join(os.path.dirname(template_path), 'media')
    values['markers_sdf'] = '\n    '.join(
        marker_sdf(m['id'], m['x'], m['y'], m['top_z'], m['yaw'],
                   os.path.join(media, f'aruco_4x4_{m["id"]}.png'))
        for m in cell.get('markers', []))
    with open(template_path) as f:
        template = string.Template(f.read())
    sdf = template.substitute({k: str(v) for k, v in values.items()})
    out_dir = out_dir or tempfile.gettempdir()
    os.makedirs(out_dir, exist_ok=True)
    fd, path = tempfile.mkstemp(prefix='sorting_cell_', suffix='.sdf', dir=out_dir)
    with os.fdopen(fd, 'w') as f:
        f.write(sdf)
    return path
