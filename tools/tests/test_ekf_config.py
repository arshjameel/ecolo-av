"""
ekf_global refuses a GPS jump, but never the car's first fix (docs/simulation.md, 15 Oct).

robot_localization refuses a measurement whose squared Mahalanobis distance is at or over
threshold^2 (filter_base.cpp, checkMahalanobisThreshold): for a position, roughly its
distance over sqrt(the filter's variance + the fix's).
"""

import math
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'ros' / 'localization' / 'aav_localization' / 'config'
GPS_NOISE = 0.5         # m, 1 sigma: Gazebo's GPS (sim_adapter)
JUMP = 20.0             # m: pixi run sim-fault gps jump (aav_sim_models faults.py)


def params(name):
    return yaml.safe_load((CONFIG / f'{name}.yaml').read_text())[name]['ros__parameters']


def sigmas(distance, filter_variance, fix_variance=GPS_NOISE ** 2):
    return distance / math.sqrt(filter_variance + fix_variance)


def test_only_the_gps_input_is_gated():
    p = params('ekf_global')
    assert p['odom1'] == '/localization/gps/odom'
    gated = [k for k in p if k.endswith('rejection_threshold')]
    assert gated == ['odom1_pose_rejection_threshold']
    # ekf_local has no GPS, and gates nothing.
    assert not [k for k in params('ekf_local') if k.endswith('rejection_threshold')]


def test_the_first_fix_is_taken_even_far_from_the_start():
    p = params('ekf_global')
    threshold = p['odom1_pose_rejection_threshold']
    start = p['initial_estimate_covariance'][0]
    assert p['initial_estimate_covariance'][1] == start
    # A car parked 500 m from start_x, start_y (or the datum) still takes its first fix.
    assert sigmas(500.0, start) < threshold


def test_a_20_m_jump_is_refused_once_the_filter_is_sure():
    threshold = params('ekf_global')['odom1_pose_rejection_threshold']
    # After a few fixes the filter is about as sure as one fix; 10 s without GPS adds the
    # process noise's 0.05 m^2/s for x.
    settled = GPS_NOISE ** 2
    after_a_gap = settled + 0.05 * 10.0
    assert sigmas(JUMP, settled) > threshold and sigmas(JUMP, after_a_gap) > threshold
    # Normal fixes, 3 sigma of noise off, always pass.
    assert sigmas(3 * GPS_NOISE, settled) < threshold
