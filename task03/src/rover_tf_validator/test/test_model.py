"""Public model checks against the published physical specification (needs Xacro)."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
xacro = pytest.importorskip('xacro', reason='Xacro model checks run in the ROS container')
DESCRIPTION=Path(__file__).resolve().parents[2]/'rover_description'


@pytest.fixture(scope='module')
def model():
    if not DESCRIPTION.exists():
        pytest.skip('run model checks from the complete student source tree')
    xml=xacro.process_file(str(DESCRIPTION/'urdf/rover.urdf.xacro'), mappings={
        'frame_prefix':'audit/', 'mounts_file':str(DESCRIPTION/'config/homework_mounts.yaml')}).toxml()
    return ET.fromstring(xml)


def values(element, attribute, default='0 0 0'):
    return tuple(float(x) for x in element.attrib.get(attribute,default).split())


def test_connected_tree_and_prefixed_links(model):
    links={node.attrib['name'] for node in model.findall('link')}
    assert links and all(link.startswith('audit/') for link in links)
    parents={}
    for joint in model.findall('joint'):
        child=joint.find('child').attrib['link']
        parent=joint.find('parent').attrib['link']
        assert child in links and parent in links and child not in parents
        parents[child]=parent
    assert links-set(parents)=={'audit/base_link'}
    for child in parents:
        seen=set()
        while child in parents:
            assert child not in seen, 'cycle in model tree'
            seen.add(child); child=parents[child]
        assert child=='audit/base_link'


def test_lidar_mount_matches_the_public_mechanical_drawing(model):
    joint=model.find("joint[@name='lidar_joint']")
    assert joint.attrib['type']=='fixed'
    assert values(joint.find('origin'),'xyz')==pytest.approx((.10,0.,.16))
    assert values(joint.find('origin'),'rpy')==pytest.approx((0.,0.,0.))


def test_camera_optical_basis_matches_rep103(model):
    joint=model.find("joint[@name='camera_optical_joint']")
    # Fixed-axis RPY: optical z forward, x right, y down in camera body axes.
    roll,pitch,yaw=values(joint.find('origin'),'rpy')
    cr,sr,cp,sp,cy,sy=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
    x_axis=(cy*cp,sy*cp,-sp)
    y_axis=(cy*sp*sr-sy*cr,sy*sp*sr+cy*cr,cp*sr)
    z_axis=(cy*sp*cr+sy*sr,sy*sp*cr-cy*sr,cp*cr)
    assert x_axis==pytest.approx((0.,-1.,0.))
    assert y_axis==pytest.approx((0.,0.,-1.))
    assert z_axis==pytest.approx((1.,0.,0.))


def test_physical_links_have_collision_and_positive_inertia(model):
    for link in model.findall('link'):
        if link.find('visual') is None:  # A coordinate-only optical frame has no body.
            continue
        assert link.find('collision/geometry') is not None
        mass=float(link.find('inertial/mass').attrib['value'])
        inertia=link.find('inertial/inertia').attrib
        xx,yy,zz=[float(inertia[key]) for key in ('ixx','iyy','izz')]
        xy,xz,yz=[float(inertia[key]) for key in ('ixy','ixz','iyz')]
        assert mass>0 and all(math.isfinite(v) for v in (xx,yy,zz,xy,xz,yz))
        # Sylvester's criterion for this symmetric 3x3 tensor.
        assert xx>0 and xx*yy-xy*xy>0
        assert xx*yy*zz+2*xy*xz*yz-xx*yz*yz-yy*xz*xz-zz*xy*xy>0


def test_wheels_are_movable_with_nonzero_axis(model):
    joints=[j for j in model.findall('joint') if j.attrib['name'].endswith('_wheel_joint')]
    assert len(joints)==4
    for joint in joints:
        assert joint.attrib['type']=='continuous'
        assert sum(x*x for x in values(joint.find('axis'),'xyz'))==pytest.approx(1.)
