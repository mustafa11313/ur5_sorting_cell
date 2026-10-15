"""The robot xacro must expand to a valid URDF in both simulation modes."""
import os
import subprocess
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory

XACRO = os.path.join(
    get_package_share_directory('sorting_cell_description'), 'urdf', 'sorting_cell.urdf.xacro')


@pytest.mark.parametrize('sim', ['true', 'false'])
def test_xacro_expands(sim, tmp_path):
    controllers = tmp_path / 'controllers.yaml'
    controllers.write_text('{}')
    out = subprocess.run(
        ['xacro', XACRO, f'sim_ignition:={sim}', f'controllers_file:={controllers}'],
        check=True, capture_output=True, text=True).stdout
    root = ET.fromstring(out)
    links = {link.get('name') for link in root.iter('link')}
    joints = {j.get('name'): j.get('type') for j in root.findall('joint')}
    assert {'world', 'base_link', 'tool0', 'gripper_base_link', 'tcp'} <= links
    assert joints['left_finger_joint'] == 'prismatic'
    assert joints['right_finger_joint'] == 'prismatic'
    hw = root.find('ros2_control/hardware/plugin').text
    assert hw == ('gz_ros2_control/GazeboSimSystem' if sim == 'true'
                  else 'mock_components/GenericSystem')
