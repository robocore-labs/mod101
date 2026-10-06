"""PGGripper actuator units, coupled jaw travel, mass and integration."""
import json
import xml.etree.ElementTree as ET
import pytest
import xacro
from configurator import server, tool_options

PKG = server.SRC / 'mod101_tool_pggripper'

@pytest.mark.parametrize('dof', [6, 7])
@pytest.mark.parametrize('camera', ['true', 'false'])
def test_pggripper_model_and_semantics(dof, camera):
    args = {'tool':'pggripper', 'arm_dof':str(dof), 'wrist_camera':camera}
    root = ET.fromstring(server.expand_urdf(args))
    drive = root.find("joint[@name='6']")
    assert drive.get('type') == 'revolute'
    maximum = float(drive.find('limit').get('upper'))
    for name, sign in [('left',1),('right',-1)]:
        joint = root.find(f"joint[@name='pggripper_{name}_slider']")
        mimic = joint.find('mimic')
        assert mimic.get('joint') == '6'
        assert float(mimic.get('multiplier')) * maximum == pytest.approx(sign * .027)
    assert root.find("joint[@name='pggripper_mount']/origin").get('xyz') == '0 0 0'
    assert float(root.find("link[@name='pggripper_servo']/inertial/mass").get('value')) == .056
    semantics = ET.fromstring(xacro.process_file(str(server.SRC/'mod101_moveit_config/srdf/mod101.srdf.xacro'),mappings=args).toxml())
    links = {l.get('name') for l in root.findall('link')}
    for pair in semantics.findall('disable_collisions'):
        assert pair.get('link1') in links and pair.get('link2') in links
    assert float(semantics.find("group_state[@name='open'][@group='gripper']/joint").get('value')) == pytest.approx(maximum)
    passive = {j.get('name') for j in semantics.findall('passive_joint')}
    assert {'pggripper_left_slider','pggripper_right_slider'} <= passive
    assert ('wrist_camera_v1_1' in links) == (camera == 'true')


def test_pggripper_discovery_mass_and_prefixed_control():
    schema = tool_options.discover(server.SRC)['pggripper']
    assert schema['label'] == 'PGGripper' and schema['servo_count'] == 1
    assert schema['actuated'] is True
    data = json.loads((PKG/'config/measurements.json').read_text())
    assert sum(p['mass_kg'] for p in data['parts'].values()) == pytest.approx(.1535805043569026)
    doc = xacro.parse('''<robot name="test" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:arg name="hardware" default="mock"/>
      <xacro:include filename="%s"/>
      <link name="right_wrist_flange"/>
      <xacro:mod101_tool_pggripper prefix="right_" use_sim="false"/>
    </robot>''' % (PKG/'urdf/tool.urdf.xacro'))
    xacro.process_doc(doc)
    root = ET.fromstring(doc.toxml())
    for j in root.findall('joint'):
        assert j.get('name').startswith('right_')
        if j.find('mimic') is not None: assert j.find('mimic').get('joint') == 'right_6'
    control = root.find('ros2_control')
    commands = [j.get('name') for j in control.findall('joint') if j.find('command_interface') is not None]
    assert commands == ['right_6']
