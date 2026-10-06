"""Run with a sourced ROS Jazzy workspace: python3 -m pytest tests/."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from configurator import server
from configurator.payload import calculate


@pytest.mark.parametrize('dof', [6, 7])
@pytest.mark.parametrize('tool', ['none', 'jaws', 'parallel', 'pincopen'])
@pytest.mark.parametrize('mount', ['small', 'big'])
def test_description_controller_contract(dof, tool, mount):
    xml = server.expand_urdf({'arm_dof': dof, 'tool': tool,
                             'shoulder_mount': mount, 'elbow_mount': mount})
    root = ET.fromstring(xml)
    joints = {j.get('name'): j for j in root.findall('joint')}
    assert ('joint_wrist_yaw' in joints) == (dof == 7)
    assert ('6' in joints) == (tool != 'none')
    assert root.find("link[@name='wrist_flange']") is not None
    controlled = [j.get('name') for j in root.find('ros2_control').findall('joint')]
    filename = 'controllers.sim.7dof.yaml' if dof == 7 else 'controllers.sim.yaml'
    controllers = yaml.safe_load((server.SRC / 'mod101_control/config' / filename).read_text())
    arm = controllers['arm_trajectory_controller']['ros__parameters']['joints']
    assert controlled == arm
    if dof == 7:
        yaw = joints['joint_wrist_yaw']
        assert float(yaw.find('limit').get('lower')) == pytest.approx(-math.pi / 2)
        assert float(root.find("link[@name='servo_wrist_yaw_1']/inertial/mass").get('value')) == .056


def model(dof, shoulder=.1, elbow=.16, tool='jaws'):
    return calculate(server.expand_urdf({'arm_dof': dof, 'tool': tool,
                                        'shoulder_ext_length': shoulder,
                                        'elbow_ext_length': elbow}), server.SRC, {},
                     server.expand_urdf({'arm_dof': 6, 'tool': 'jaws',
                                         'shoulder_ext_length': shoulder,
                                         'elbow_ext_length': elbow}))


def test_added_actuator_and_prints_reduce_payload():
    six, seven = model(6), model(7)
    assert seven['mTotal'] > six['mTotal'] + .056
    assert seven['reach'] > six['reach']
    assert seven['out']['duty-full']['payload'] < six['out']['duty-full']['payload']


def test_lengths_change_real_mass_and_moments():
    short, long = model(7, elbow=.08), model(7, elbow=.16)
    assert long['mTotal'] - short['mTotal'] == pytest.approx(2 * .46 * .08)
    assert long['reach'] - short['reach'] == pytest.approx(.08, abs=1e-4)
    assert long['out']['duty-full']['payload'] < short['out']['duty-full']['payload']
    assert model(7, tool='none')['mTotal'] < model(7, tool='pincopen')['mTotal']


def test_save_variant_and_reject_invalid(tmp_path, monkeypatch):
    cfg = tmp_path / 'config.xacro'
    cfg.write_text(server.CONFIG.read_text())
    monkeypatch.setattr(server, 'CONFIG', cfg)
    server.write_props(.1, .16, 'small', 'small', 7)
    assert server.read_props()['arm_dof'] == 7
    before = cfg.read_text()
    with pytest.raises(ValueError):
        server.write_props(.1, .16, 'small', 'small', 8)
    assert cfg.read_text() == before
    server.write_props(.1, .16, 'small', 'small', 6)
    assert server.read_props()['arm_dof'] == 6


def test_print_profile_provenance():
    measures = json.loads((server.DESC / 'config/7dof_measurements.json').read_text())
    assert '15' in str(measures)
    assert '1270' in str(measures)


def test_bench_reference_140_140_jaws_stays_unchanged():
    # Golden values calculated with the original JS formula and its original
    # mass JSON. User confirmed this configuration's >700g load on the bench.
    r = model(6, shoulder=.14, elbow=.14)
    assert r['reach'] == pytest.approx(.506)
    assert r['mTotal'] == pytest.approx(.86142)
    assert r['out']['duty-full']['payload'] == pytest.approx(.9109356921065458)
    assert r['out']['peak-full']['payload'] > r['out']['duty-full']['payload']
    seven = model(7, shoulder=.14, elbow=.14)
    assert seven['out']['duty-full']['payload'] < r['out']['duty-full']['payload']


@pytest.mark.parametrize('dof', [6, 7])
@pytest.mark.parametrize('tool', ['none', 'jaws', 'parallel', 'pincopen'])
@pytest.mark.parametrize('camera', [True, False])
def test_camera_toggle_matches_semantics(dof, tool, camera):
    import xacro
    args = {'arm_dof': str(dof), 'tool': tool, 'wrist_camera': str(camera).lower()}
    root = ET.fromstring(server.expand_urdf(args))
    links = {link.get('name') for link in root.findall('link')}
    assert ('wrist_camera_v1_1' in links) == camera
    assert ('camera_adapter_1' in links) == camera
    srdf = ET.fromstring(xacro.process_file(
        str(server.SRC / 'mod101_moveit_config/srdf/mod101.srdf.xacro'), mappings=args).toxml())
    for pair in srdf.findall('disable_collisions'):
        assert pair.get('link1') in links and pair.get('link2') in links
    if not camera:
        assert not any(g.find("sensor[@type='camera']") is not None for g in root.findall('gazebo'))


def test_camera_setting_saves_and_reloads(tmp_path, monkeypatch):
    cfg = tmp_path / 'config.xacro'
    cfg.write_text(server.CONFIG.read_text())
    monkeypatch.setattr(server, 'CONFIG', cfg)
    server.write_props(.14, .14, 'small', 'small', 7, False)
    assert server.read_props()['wrist_camera'] is False
    server.write_props(.14, .14, 'small', 'small', 6)
    assert server.read_props()['wrist_camera'] is False
    server.write_props(.14, .14, 'small', 'small', 6, True)
    assert server.read_props()['wrist_camera'] is True


def test_parallel_servo_is_visible_without_double_counting_mass():
    root = ET.fromstring(server.expand_urdf({'tool': 'parallel'}))
    body = root.find("link[@name='Parallel_gripper_assembly_v1_1']")
    assert float(body.find('inertial/mass').get('value')) == .078
    visual = body.find("visual[@name='parallel_servo_visual']")
    assert visual is not None
    assert visual.find('geometry/mesh').get('filename').endswith('/parallel_servo.stl')
