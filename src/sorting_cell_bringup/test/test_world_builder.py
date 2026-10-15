import os
import xml.etree.ElementTree as ET

import yaml

from sorting_cell_bringup.world_builder import load_scenario, parse_overrides, render_world

HERE = os.path.dirname(__file__)
PKG = os.path.dirname(HERE)
TEMPLATE = os.path.join(PKG, '..', 'sorting_cell_gazebo', 'worlds', 'sorting_cell.sdf.template')


def load_cell():
    with open(os.path.join(PKG, 'config', 'cell.yaml')) as f:
        return yaml.safe_load(f)['cell']


def test_overrides():
    assert parse_overrides('sun=0.3 sun_direction=-1,0,-1') == {
        'sun': '0.3', 'sun_direction': '-1 0 -1'}


def test_every_scenario_renders_valid_sdf(tmp_path):
    cell = load_cell()
    scen_dir = os.path.join(PKG, 'config', 'scenarios')
    for name in sorted(os.listdir(scen_dir)):
        params = load_scenario(os.path.join(scen_dir, name))
        path = render_world(TEMPLATE, cell, params, out_dir=str(tmp_path))
        root = ET.parse(path).getroot()
        world = root.find('world')
        models = {m.get('name') for m in world.findall('model')}
        assert {'conveyor', 'pedestal', 'bin_cube', 'bin_reject', 'aruco_0', 'aruco_1',
                'overhead_camera'} <= models, name
        plugin = [p for p in world.findall('plugin') if p.get('name').startswith('sorting_cell')][0]
        assert float(plugin.find('belt_speed').text) == float(params['belt_speed'])
        assert len(plugin.findall('bin')) == 4
