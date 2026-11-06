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

# Owner-approved experimental exceptions, Hyundai Inster only (nothing else is exempt), see TestInsterLateralLimitsException:
# * MAX_LAT_ACCEL_EXCEPTIONS (20-23 Oct 2026): its MAX_LAT_ACCEL_MEASURED is a placeholder for the lateral acceleration at its cap of
#   409 raw, above ISO 11270's 3.0.
# * MAX_LAT_JERK_DOWN_FULL_SCALE_EXCEPTIONS (5 Nov 2026, ramp 3 up / 6 down): the Inster test also reads the jerk, more conservatively,
#   as if MAX_LAT_ACCEL_MEASURED were already reached at the full scale 384 instead of the cap 409; in that reading only, the release
#   at 6 raw per frame gives 5.16 m/s^3 and may go up to 5.2 instead of ISO's 5.0. The up-jerk bound (2.5 + 0.5) applies in full,
#   and so does the down bound at the cap (the reading TestLateralLimits uses for every car; the Inster's is 4.84).
MAX_LAT_ACCEL_EXCEPTIONS = {'HYUNDAI_INSTER': INSTER_MAX_LAT_ACCEL_MEASURED}
MAX_LAT_JERK_DOWN_FULL_SCALE_EXCEPTIONS = {'HYUNDAI_INSTER': 5.2}


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
  """The Hyundai Inster's owner-approved exceptions (MAX_LAT_ACCEL_EXCEPTIONS, MAX_LAT_JERK_DOWN_FULL_SCALE_EXCEPTIONS): this car only,
  the value in the torque data equals the constant in values.py, and with its ramps (3 up / 6 down) the jerk rules hold at the cap 409
  (MAX_LAT_ACCEL_MEASURED reached at 409, the reading TestLateralLimits uses) in full and, more conservatively, as if it were already
  reached at the full scale 384 with the down bound 5.2 instead of 5.0 (the up bound in full)."""

  @staticmethod
  def jerk_ok(control_params, torque_params, steer_delta_up, steer_delta_down) -> bool:
    # the Inster's jerk rule for the given ramp: at the cap ISO's bounds; at the full scale ISO's up bound, the excepted down bound
    for cap, down_max in ((INSTER_STEER_MAX, MAX_LAT_JERK_DOWN),
                          (INSTER_STEER_FULL_SCALE, MAX_LAT_JERK_DOWN_FULL_SCALE_EXCEPTIONS['HYUNDAI_INSTER'])):
      params = copy.copy(control_params)
      params.STEER_MAX, params.STEER_DELTA_UP, params.STEER_DELTA_DOWN = cap, steer_delta_up, steer_delta_down
      up_jerk, down_jerk = TestLateralLimits.calculate_0_5s_jerk(params, torque_params)
      if up_jerk > MAX_LAT_JERK_UP + MAX_LAT_JERK_UP_TOLERANCE or down_jerk > down_max:
        return False
    return True

  @staticmethod
  def inster_params():
    CarInterface = interfaces['HYUNDAI_INSTER']
    CP = CarInterface.get_non_essential_params('HYUNDAI_INSTER')
    control_params = importlib.import_module('opendbc.car.hyundai.values').CarControllerParams(CP)
    return control_params, get_torque_params()['HYUNDAI_INSTER']

  def test_exception_scope(self):
    assert set(MAX_LAT_ACCEL_EXCEPTIONS) == {'HYUNDAI_INSTER'}
    assert set(MAX_LAT_JERK_DOWN_FULL_SCALE_EXCEPTIONS) == {'HYUNDAI_INSTER'}
    # the owner-approved bound of the down-jerk exception (full-scale reading only): widening it needs a deliberate change here
    assert MAX_LAT_JERK_DOWN_FULL_SCALE_EXCEPTIONS['HYUNDAI_INSTER'] == 5.2
    # the owner-approved bound: raising the Inster's value above 3.3 m/s^2 needs a deliberate change here
    assert INSTER_MAX_LAT_ACCEL_MEASURED <= 3.3
    assert get_torque_params()['HYUNDAI_INSTER']['MAX_LAT_ACCEL_MEASURED'] == INSTER_MAX_LAT_ACCEL_MEASURED

  def test_jerk_limits_at_cap_and_full_scale(self):
    control_params, torque_params = self.inster_params()
    assert (control_params.STEER_MAX, control_params.STEER_FULL_SCALE) == (INSTER_STEER_MAX, INSTER_STEER_FULL_SCALE) == (409, 384)
    assert (control_params.STEER_DELTA_UP, control_params.STEER_DELTA_DOWN) == (3, 6)
    assert self.jerk_ok(control_params, torque_params, control_params.STEER_DELTA_UP, control_params.STEER_DELTA_DOWN)

  def test_exception_admits_no_faster_ramp(self):
    # the exception is just wide enough for 3 up / 6 down: a rise of 4 (up 3.23 m/s^3 at the cap) or a release of 7 (down 6.02 at the
    # full scale, 5.65 at the cap) still fails
    control_params, torque_params = self.inster_params()
    assert self.jerk_ok(control_params, torque_params, 3, 6)
    for up, down in ((4, 6), (3, 7), (4, 5), (4, 7)):
      assert not self.jerk_ok(control_params, torque_params, up, down), (up, down)
