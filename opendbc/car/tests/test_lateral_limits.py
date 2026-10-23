#!/usr/bin/env python3
import copy
import unittest
import importlib
from opendbc.testing import parameterized_class

from opendbc.car import DT_CTRL
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import INSTER_MAX_LAT_ACCEL_MEASURED, INSTER_STEER_MAX, INSTER_STEER_FULL_SCALE
from opendbc.car.interfaces import get_torque_params
from opendbc.car.lateral import ISO_LATERAL_ACCEL
from opendbc.car.values import PLATFORMS

# ISO 11270 - allowed up jerk is strictly lower than recommended limits
MAX_LAT_JERK_UP = 2.5            # m/s^3
MAX_LAT_JERK_DOWN = 5.0          # m/s^3
MAX_LAT_JERK_UP_TOLERANCE = 0.5  # m/s^3

# jerk is measured over half a second
JERK_MEAS_T = 0.5

# Owner-approved experimental exception (Hyundai Inster, 20-23 Oct 2026; nothing else is exempt): its MAX_LAT_ACCEL_MEASURED is a
# placeholder for the lateral acceleration at its cap of 409 raw, above ISO 11270's 3.0. The jerk rules still apply in full, see
# TestInsterLateralLimitsException.
MAX_LAT_ACCEL_EXCEPTIONS = {'HYUNDAI_INSTER': INSTER_MAX_LAT_ACCEL_MEASURED}


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
  """The Hyundai Inster's owner-approved exception (MAX_LAT_ACCEL_EXCEPTIONS): this car only, the value in the torque data equals the
  constant in values.py, and the jerk rules hold with its ramps (3 up / 5 down) both at the cap 409 (MAX_LAT_ACCEL_MEASURED reached at
  409, the reading TestLateralLimits uses) and, more conservatively, as if it were already reached at the full scale 384."""

  def test_exception_scope(self):
    assert set(MAX_LAT_ACCEL_EXCEPTIONS) == {'HYUNDAI_INSTER'}
    # the owner-approved bound: raising the Inster's value above 3.3 m/s^2 needs a deliberate change here
    assert INSTER_MAX_LAT_ACCEL_MEASURED <= 3.3
    assert get_torque_params()['HYUNDAI_INSTER']['MAX_LAT_ACCEL_MEASURED'] == INSTER_MAX_LAT_ACCEL_MEASURED

  def test_jerk_limits_at_cap_and_full_scale(self):
    CarInterface = interfaces['HYUNDAI_INSTER']
    CP = CarInterface.get_non_essential_params('HYUNDAI_INSTER')
    control_params = importlib.import_module('opendbc.car.hyundai.values').CarControllerParams(CP)
    torque_params = get_torque_params()['HYUNDAI_INSTER']
    assert (control_params.STEER_MAX, control_params.STEER_FULL_SCALE) == (INSTER_STEER_MAX, INSTER_STEER_FULL_SCALE) == (409, 384)
    assert (control_params.STEER_DELTA_UP, control_params.STEER_DELTA_DOWN) == (3, 5)
    for cap in (INSTER_STEER_MAX, INSTER_STEER_FULL_SCALE):
      params = copy.copy(control_params)
      params.STEER_MAX = cap
      up_jerk, down_jerk = TestLateralLimits.calculate_0_5s_jerk(params, torque_params)
      assert up_jerk <= MAX_LAT_JERK_UP + MAX_LAT_JERK_UP_TOLERANCE, (cap, up_jerk)
      assert down_jerk <= MAX_LAT_JERK_DOWN, (cap, down_jerk)
