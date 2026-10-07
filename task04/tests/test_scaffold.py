import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rover_sim_lab'))
from rover_sim_lab.configuration import load_settings,validate_settings,sdf,urdf,bridge_config,names


def test_independent_model_and_offline_world():
    s=load_settings(ROOT,'practice')
    validate_settings(s)
    world=ET.fromstring(sdf(s,'unit/','trial'))
    assert world.find('.//physics/max_step_size').text=='0.001'
    assert world.find('.//model[@name="teaching_cart"]') is not None
    assert len(world.findall('.//sensor'))==3
    assert len(world.findall('.//joint'))==2
    assert 'http' not in sdf(s,'unit/','trial')
    robot=ET.fromstring(urdf())
    assert robot.find(".//joint[@name='camera_optical_frame_fixed']") is not None


def test_one_clock_bridge_and_parameterized_frames():
    s=load_settings(ROOT,'practice')
    bridges=bridge_config(s,'alpha','beta/','arena')
    clock=[b for b in bridges if b['ros_topic_name']=='/clock']
    assert len(clock)==1 and clock[0]['gz_topic_name']=='/world/arena/clock'
    assert clock[0]['direction']=='GZ_TO_ROS'
    assert all(b['ros_topic_name'].startswith('/alpha/') or b['ros_topic_name'] in ['/clock','/tf'] for b in bridges)
    assert '<frame_id>beta/odom</frame_id>' in sdf(s,'beta/','arena')


def test_homework_stays_independent():
    s=load_settings(ROOT,'homework')
    # The fixed sensor endpoint must not move together with an edited bridge.
    s['bridge']['scan']['gz']='wrong'
    xml=sdf(s,'check/','check')
    assert '<topic>/lab/homework/front/ranges</topic>' in xml
    assert '<topic>/lab/homework/wrong</topic>' not in xml


@pytest.mark.parametrize('namespace,prefix,world',[('', 'r/', 'w'),('r','r/','../bad'),('r;echo','r/','w')])
def test_invalid_names(namespace,prefix,world):
    with pytest.raises(ValueError): names(namespace,prefix,world)


@pytest.mark.skipif(sys.platform != 'linux', reason='the runtime is a Linux container')
def test_cleanup_reaps_only_its_own_orphan_session():
    import subprocess
    from rover_sim_lab.processes import cleanup_session,session_members
    sentinel=subprocess.Popen([sys.executable,'-c','import time; time.sleep(20)'])
    parent=subprocess.Popen([sys.executable,'-c',
        'import subprocess,sys; subprocess.Popen([sys.executable,"-c","import time; time.sleep(20)"])'],
        start_new_session=True)
    try:
        parent.wait(timeout=3)
        assert session_members(parent.pid)
        cleanup_session(parent.pid)
        assert not session_members(parent.pid)
        assert sentinel.poll() is None
    finally:
        cleanup_session(parent.pid)
        sentinel.terminate()
        sentinel.wait(timeout=3)
