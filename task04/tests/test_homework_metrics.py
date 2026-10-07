"""These tests intentionally fail until TODO C is completed."""
import sys
from pathlib import Path
from copy import deepcopy
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/rover_sim_lab'))
from rover_sim_lab.homework_metrics import analyze_samples


def test_no_samples():
    assert analyze_samples([])==dict(samples=0,sim_hz=None,wall_hz=None,gyro_mean=None,gyro_std=None,epochs=0)


def test_two_time_axes_and_population_std():
    rows=[{'stamp':s,'arrival':a,'value':v} for s,a,v in [(1,10,0.06),(1.1,10.2,0.10),(1.2,10.4,0.08)]]
    original=deepcopy(rows)
    result=analyze_samples(rows)
    assert rows==original
    assert result['samples']==3 and result['epochs']==1
    assert result['sim_hz']==pytest.approx(10)
    assert result['wall_hz']==pytest.approx(5)
    assert result['gyro_mean']==pytest.approx(0.08)
    assert result['gyro_std']==pytest.approx(0.01632993161855452)


def test_reset_and_duplicate():
    rows=[{'stamp':s,'arrival':a,'value':v} for s,a,v in [(8,100,9),(9,101,9),(0,102,0.1),(0,102.2,99),(0.1,102.5,0.3)]]
    result=analyze_samples(rows)
    assert result['epochs']==2 and result['samples']==2
    assert result['sim_hz']==pytest.approx(10)
    assert result['wall_hz']==pytest.approx(2)
    assert result['gyro_mean']==pytest.approx(0.2)
    assert result['gyro_std']==pytest.approx(0.1)


def test_single_sample_and_missing_value():
    assert analyze_samples([{'stamp':0,'arrival':4,'value':None}])==dict(
        samples=1,sim_hz=None,wall_hz=None,gyro_mean=None,gyro_std=None,epochs=1)


def test_nonpositive_wall_span_and_nonfinite_gyro():
    result=analyze_samples([{'stamp':0,'arrival':4,'value':float('nan')},
                            {'stamp':0.1,'arrival':4,'value':0.2}])
    assert result['sim_hz']==pytest.approx(10)
    assert result['wall_hz'] is None
    assert result['gyro_mean']==pytest.approx(0.2)
    assert result['gyro_std']==0
