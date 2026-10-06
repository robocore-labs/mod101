"""Camera selection, inertials, optical axes, and extensible option contract."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import xacro
from configurator import server, tool_options
from configurator.payload import calculate, origin


@pytest.mark.parametrize('dof', [6, 7])
@pytest.mark.parametrize('sensor,weight', [('luxonis', .061), ('realsense', .072)])
@pytest.mark.parametrize('gopro', [False, True])
def test_mutually_exclusive_cameras_and_exact_weights(dof, sensor, weight, gopro):
    args = {'tool': 'camera', 'arm_dof': str(dof), 'camera_sensor': sensor,
            'camera_gopro': str(gopro).lower()}
    root = ET.fromstring(server.expand_urdf(args))
    assert root.find("joint[@name='6']") is None
    primary = root.find("link[@name='camera_tool_primary']")
    assert float(primary.find('inertial/mass').get('value')) == weight
    meshes = [v.find('geometry/mesh').get('filename') for v in primary.findall('visual')]
    assert len(meshes) == 1 and meshes[0].endswith(f'/{sensor}.stl')
    assert (root.find("link[@name='camera_tool_gopro']") is not None) == gopro
    assert (root.find("link[@name='camera_tool_gopro_optical_frame']") is not None) == gopro
    if gopro:
        assert float(root.find("link[@name='camera_tool_gopro']/inertial/mass").get('value')) == .153
    links = {l.get('name') for l in root.findall('link')}
    semantics = ET.fromstring(xacro.process_file(
        str(server.SRC / 'mod101_moveit_config/srdf/mod101.srdf.xacro'), mappings=args).toxml())
    for pair in semantics.findall('disable_collisions'):
        assert pair.get('link1') in links and pair.get('link2') in links


def test_gopro_optical_axes_match_rep103_and_cad_datum():
    root = ET.fromstring(server.expand_urdf({'tool':'camera','camera_gopro':'true'}))
    joint = root.find("joint[@name='camera_tool_gopro_optical_joint']")
    transform = origin(joint.find('origin'))
    np.testing.assert_allclose(transform[:3,3], [.035185104237,.021593041865,.066])
    np.testing.assert_allclose(transform[:3,:3] @ [0,0,1], [1,0,0], atol=1e-12)
    np.testing.assert_allclose(transform[:3,:3] @ [1,0,0], [0,-1,0], atol=1e-12)
    np.testing.assert_allclose(transform[:3,:3] @ [0,1,0], [0,0,-1], atol=1e-12)


def test_gopro_and_selected_weights_affect_payload_without_changing_jaws_baseline():
    ref = server.expand_urdf({'tool':'jaws','arm_dof':6,'wrist_camera':'true'})
    def result(sensor, gopro):
        return calculate(server.expand_urdf({'tool':'camera','camera_sensor':sensor,'camera_gopro':gopro}),server.SRC,{},ref)
    bare, top = result('luxonis',False), result('luxonis',True)
    assert top['mTotal'] - bare['mTotal'] == pytest.approx(.153)
    assert result('realsense',False)['mTotal'] - bare['mTotal'] == pytest.approx(.011)
    assert top['out']['duty-full']['payload'] < bare['out']['duty-full']['payload']


def test_option_save_reload_and_validation_are_generic(tmp_path, monkeypatch):
    cfg=tmp_path/'config.xacro';cfg.write_text(server.CONFIG.read_text())
    monkeypatch.setattr(server,'CONFIG',cfg)
    server.write_props(.14,.14,'small','small',6,True,
                       {'camera_sensor':'realsense','camera_gopro':True})
    assert server.read_props()['tool_options'] == {'camera_sensor':'realsense','camera_gopro':True}
    before=cfg.read_text()
    for invalid in ({'camera_sensor':'both'}, {'camera_gopro':1}, {'made_up_option':True}):
        with pytest.raises(ValueError):server.write_props(.14,.14,'small','small',6,True,invalid)
        assert cfg.read_text()==before
    assert tool_options.discover(server.SRC)['camera']['servo_count']==0


def test_new_package_schema_needs_no_camera_specific_backend(tmp_path):
    p=tmp_path/'mod101_tool_example';(p/'config').mkdir(parents=True)
    (p/'package.xml').write_text('<package/>')
    (p/'config/configurator.yaml').write_text('''schema_version: 1
label: Example
actuated: false
options:
  example_led:
    type: boolean
    label: LED
    default: false
''')
    definitions=tool_options.specs(tmp_path)
    text=tool_options.write_values('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"></robot>',{'example_led':True},definitions)
    assert tool_options.read_values(text,definitions)=={'example_led':True}
