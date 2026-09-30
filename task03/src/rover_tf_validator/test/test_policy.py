"""Public behavioral checks. The untouched TODO starter is expected to fail."""
import math
import pytest
from rover_tf_validator.contracts import Observation, RigidTransform, LookupFailure
from rover_tf_validator.validator import quaternion_is_valid, validate_observation


def obs(**changes):
    values = dict(sensor='lidar', source_frame='unit/lidar', target_frame='unit/odom',
                  stamp_ns=12_500_000_000, point=(1.,0.,0.))
    values.update(changes)
    return Observation(**values)


def test_measurement_timestamp_and_transform_direction():
    calls=[]
    def lookup(target, source, stamp):
        calls.append((target, source, stamp))
        # Moving translation: latest (0) returns a distinct, incorrect value.
        x = 2.5 if stamp == 12_500_000_000 else 9.
        return RigidTransform((x,0.,0.), (0.,0.,0.,1.))
    result=validate_observation(obs(), lookup)
    assert result['status']=='OK', result
    assert calls==[('unit/odom','unit/lidar',12_500_000_000)]
    assert result['point']==pytest.approx([3.5,0.,0.])
    assert result['stamp_ns']==12_500_000_000


@pytest.mark.parametrize('changes', [
    {'stamp_ns':0}, {'source_frame':''}, {'source_frame':'/absolute'},
    {'point':(float('nan'),0.,0.)}, {'point':(0.,float('inf'),0.)},
])
def test_invalid_input_does_not_call_tf(changes):
    def forbidden(*args):
        pytest.fail('invalid input must be rejected before TF lookup')
    assert validate_observation(obs(**changes), forbidden)['status']=='INVALID_INPUT'


@pytest.mark.parametrize('status',['UNKNOWN_FRAME','DISCONNECTED','TIME_UNAVAILABLE'])
def test_lookup_failure_is_not_replaced_with_latest_or_identity(status):
    calls=[]
    def fail(*args):
        calls.append(args)
        raise LookupFailure(status, 'controlled unavailable TF')
    result=validate_observation(obs(),fail)
    assert result['status']==status
    assert result['point'] is None
    assert len(calls)==1


@pytest.mark.parametrize('rotation, expected', [
    ((0.,0.,0.,1.),True), ((0.,0.,0.,-1.),True),
    ((0.,0.,0.,0.),False), ((0.,0.,0.,2.),False),
    ((float('nan'),0.,0.,1.),False), ((0.,0.,float('inf'),1.),False),
])
def test_raw_quaternion_hook(rotation,expected):
    assert quaternion_is_valid(rotation) is expected
