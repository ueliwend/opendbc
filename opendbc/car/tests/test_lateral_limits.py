#!/usr/bin/env python3
import copy
import unittest
import importlib
from opendbc.testing import parameterized_class

from opendbc.car import DT_CTRL
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import INSTER_HIGH_TORQUE_MAX_LATERAL_ACCEL, INSTER_STEER_MAX_LOOKUP
from opendbc.car.interfaces import get_torque_params
from opendbc.car.lateral import ISO_LATERAL_ACCEL
from opendbc.car.values import PLATFORMS

# ISO 11270 - allowed up jerk is strictly lower than recommended limits
MAX_LAT_JERK_UP = 2.5            # m/s^3
MAX_LAT_JERK_DOWN = 5.0          # m/s^3
MAX_LAT_JERK_UP_TOLERANCE = 0.5  # m/s^3

# jerk is measured over half a second
JERK_MEAS_T = 0.5

# Owner-approved experimental exception (Hyundai Inster, 20 / 22 Oct 2026; nothing else is exempt): its MAX_LAT_ACCEL_MEASURED is
# the highest lateral acceleration of its speed-dependent torque limit (sunnypilot switch InsterHighTorqueLimits), 3.3 m/s^2, above
# ISO 11270's 3.0. The jerk rules still apply in full: at its cap of 384 and at every cap of the speed-dependent table (430-450 raw)
# with its ramps, see TestInsterLateralLimitsException.
MAX_LAT_ACCEL_EXCEPTIONS = {'HYUNDAI_INSTER': INSTER_HIGH_TORQUE_MAX_LATERAL_ACCEL}


@parameterized_class('car_model', [(c,) for c in sorted(PLATFORMS)])
class TestLateralLimits(unittest.TestCase):
  car_model: str

  @classmethod
  def setUpClass(cls):
    if 'car_model' not in cls.__dict__:
      raise unittest.SkipTest('Base class')

    CarInterface = interfaces[cls.car_model]
    CP = CarInterface.get_non_essential_params(cls.car_model)
    _ = CarInterface.get_non_essential_params_sp(CP, cls.car_model)

    if cls.car_model == 'MOCK':
      raise unittest.SkipTest('Mock car')

    # TODO: test all platforms
    if CP.steerControlType != 'torque':
      raise unittest.SkipTest

    if CP.notCar:
      raise unittest.SkipTest

    CarControllerParams = importlib.import_module(f'opendbc.car.{CP.brand}.values').CarControllerParams
    cls.control_params = CarControllerParams(CP)
    cls.torque_params = get_torque_params()[cls.car_model]

  @staticmethod
  def calculate_0_5s_jerk(control_params, torque_params):
    steer_step = control_params.STEER_STEP
    max_lat_accel = torque_params['MAX_LAT_ACCEL_MEASURED']

    # Steer up/down delta per 10ms frame, in percentage of max torque
    steer_up_per_frame = control_params.STEER_DELTA_UP / control_params.STEER_MAX / steer_step
    steer_down_per_frame = control_params.STEER_DELTA_DOWN / control_params.STEER_MAX / steer_step

    # Lateral acceleration reached in 0.5 seconds, clipping to max torque
    accel_up_0_5_sec = min(steer_up_per_frame * JERK_MEAS_T / DT_CTRL, 1.0) * max_lat_accel
    accel_down_0_5_sec = min(steer_down_per_frame * JERK_MEAS_T / DT_CTRL, 1.0) * max_lat_accel

    # Convert to m/s^3
    return accel_up_0_5_sec / JERK_MEAS_T, accel_down_0_5_sec / JERK_MEAS_T

  def test_jerk_limits(self):
    up_jerk, down_jerk = self.calculate_0_5s_jerk(self.control_params, self.torque_params)
    assert up_jerk <= MAX_LAT_JERK_UP + MAX_LAT_JERK_UP_TOLERANCE
    assert down_jerk <= MAX_LAT_JERK_DOWN

  def test_max_lateral_accel(self):
    assert self.torque_params["MAX_LAT_ACCEL_MEASURED"] <= MAX_LAT_ACCEL_EXCEPTIONS.get(self.car_model, ISO_LATERAL_ACCEL)


class TestInsterLateralLimitsException(unittest.TestCase):
  """The Hyundai Inster's owner-approved exception (MAX_LAT_ACCEL_EXCEPTIONS): limited to 3.3 m/s^2 and to this car, and the jerk rules
  hold at every cap the car side can use: 384 (switch off) and each value of the speed-dependent table, 430-450 raw, with the same ramps.
  The jerk scales with ramp / cap x MAX_LAT_ACCEL_MEASURED, so it is highest at the lowest cap."""

  def test_exception_scope(self):
    assert set(MAX_LAT_ACCEL_EXCEPTIONS) == {'HYUNDAI_INSTER'}
    assert MAX_LAT_ACCEL_EXCEPTIONS['HYUNDAI_INSTER'] == 3.3
    assert get_torque_params()['HYUNDAI_INSTER']['MAX_LAT_ACCEL_MEASURED'] == INSTER_HIGH_TORQUE_MAX_LATERAL_ACCEL

  def test_jerk_limits_speed_dependent_caps(self):
    CarInterface = interfaces['HYUNDAI_INSTER']
    CP = CarInterface.get_non_essential_params('HYUNDAI_INSTER')
    CarControllerParams = importlib.import_module('opendbc.car.hyundai.values').CarControllerParams
    control_params = CarControllerParams(CP)
    torque_params = get_torque_params()['HYUNDAI_INSTER']
    low, high = min(INSTER_STEER_MAX_LOOKUP[1]), max(INSTER_STEER_MAX_LOOKUP[1])
    assert (control_params.STEER_DELTA_UP, control_params.STEER_DELTA_DOWN) == (3, 5)
    for cap in [control_params.STEER_MAX, *range(low, high + 1)]:
      params = copy.copy(control_params)
      params.STEER_MAX = cap
      up_jerk, down_jerk = TestLateralLimits.calculate_0_5s_jerk(params, torque_params)
      assert up_jerk <= MAX_LAT_JERK_UP + MAX_LAT_JERK_UP_TOLERANCE, (cap, up_jerk)
      assert down_jerk <= MAX_LAT_JERK_DOWN, (cap, down_jerk)
