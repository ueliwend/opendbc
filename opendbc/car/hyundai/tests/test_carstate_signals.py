import json
import lzma
import os
import unittest
from unittest import mock

from opendbc.can import CANParser
from opendbc.can.parser import UndecodedSignalError
from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai import carstate
from opendbc.car.hyundai.carstate import CARSTATE_SIGNALS
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC
from opendbc.car import Bus
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.sunnypilot.car.interfaces import setup_interfaces

# Recorded on a Hyundai Inster: 17 s of the car-state messages on buses 0 and 2 (braking, standstill, pulling away,
# LDA button) and the car's fingerprint.
FIXTURE = os.path.join(os.path.dirname(__file__), "inster_carstate_can.json.xz")
COPIED_MESSAGES = ("LKAS11", "CLU11", "LFAHDA_MFC")  # copied and re-sent by the car controller: decoded whole

VisualAlert = structs.CarControl.HUDControl.VisualAlert
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState


def load_fixture():
  with lzma.open(FIXTURE, "rt") as f:
    data = json.load(f)
  fingerprint = {bus: {} for bus in range(8)}
  for bus, msgs in data["fingerprint"].items():
    fingerprint[int(bus)] = {int(addr): size for addr, size in msgs.items()}
  cycles = [(t, [CanData(addr, bytes.fromhex(dat), bus) for addr, dat, bus in frames]) for t, frames in data["cycles"]]
  return fingerprint, cycles


def controls(frame: int, long_control: bool) -> tuple[structs.CarControl, structs.CarControlSP]:
  """Scripted controls through the car controller's paths: steering, cancel, resume, ICBM buttons, HUD alerts."""
  CC = structs.CarControl()
  CC_SP = structs.CarControlSP()
  phase = frame % 1700
  CC.enabled = phase >= 100
  CC.latActive = CC.enabled
  CC.longActive = CC.enabled and long_control
  CC.actuators.torque = 0.5 if (frame // 50) % 2 else -0.5
  CC.actuators.accel = 0.5 if CC.longActive else 0.0
  CC.cruiseControl.cancel = 200 <= phase < 300
  CC.cruiseControl.resume = 400 <= phase < 500
  if 600 <= phase < 700:
    CC_SP.intelligentCruiseButtonManagement.sendButton = SendButtonState.increase
  elif 700 <= phase < 800:
    CC_SP.intelligentCruiseButtonManagement.sendButton = SendButtonState.decrease
  hud = CC.hudControl
  hud.leftLaneVisible = hud.rightLaneVisible = True
  if 900 <= phase < 1000:
    hud.visualAlert = VisualAlert.steerRequired
    hud.leftLaneDepart = hud.rightLaneDepart = True
  elif 1000 <= phase < 1100:
    hud.visualAlert = VisualAlert.ldw
    hud.rightLaneVisible = False
  hud.leadVisible = 1100 <= phase < 1400
  hud.setSpeed = 20.0
  return CC.as_reader(), CC_SP


def run_inster(fingerprint, cycles, alpha_long=False):
  CP = CarInterface.get_params(CAR.HYUNDAI_INSTER, fingerprint, [], alpha_long, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.HYUNDAI_INSTER, fingerprint, [], alpha_long, False, False)
  setup_interfaces(CarInterface, CP, CP_SP, [{"HyundaiLongitudinalTuning": 2}])
  CP_SP.flags |= HyundaiFlagsSP.LONGITUDINAL_MAIN_CRUISE_TOGGLEABLE.value  # set by openpilot for this car
  CI = CarInterface(CP, CP_SP)
  outputs = []
  for frame, (t, frames) in enumerate(cycles):
    CS, CS_SP = CI.update([(t, frames)])
    CC, CC_SP = controls(frame, CP.openpilotLongitudinalControl)
    actuators, can_sends = CI.apply(CC, CC_SP, t)
    outputs.append(repr((CS, CS_SP, actuators.to_dict(), can_sends)))
  return CI, outputs


class TestHyundaiCarStateSignals(unittest.TestCase):
  """The car-state parsers of a platform in CARSTATE_SIGNALS decode only the listed signals. These tests run its car state
  and car controller over recorded CAN in strict mode, so code that reads a signal missing from the list fails here."""

  @classmethod
  def setUpClass(cls):
    cls.fingerprint, cls.cycles = load_fixture()

  def setUp(self):
    CANParser.strict_decode = True

  def tearDown(self):
    CANParser.strict_decode = False

  def fingerprint_without(self, *addrs):
    return {bus: {addr: size for addr, size in msgs.items() if addr not in addrs} for bus, msgs in self.fingerprint.items()}

  def test_list_names(self):
    for platform, signals in CARSTATE_SIGNALS.items():
      with self.subTest(platform=platform):
        CANParser(DBC[platform][Bus.pt], [], 0, decode_only=signals)  # raises for a name missing from the DBC
        for name in COPIED_MESSAGES:
          assert name not in signals, f"{name} is copied whole by the car state"

  def test_recorded_drive_strict(self):
    variants = {
      "as on the car": (self.fingerprint, False),
      "openpilot longitudinal": (self.fingerprint, True),
      # flags from the fingerprint: without FCA11 the AEB signals come from SCC12; without LFAHDA_MFC, LCA11, Navi_HU or
      # LKAS12 those messages are not read
      "fingerprint without FCA11": (self.fingerprint_without(0x38d), False),
      "fingerprint without optional messages": (self.fingerprint_without(0x38d, 0x485, 0x58b, 0x544, 0x53E), False),
    }
    for name, (fingerprint, alpha_long) in variants.items():
      with self.subTest(variant=name):
        CI, outputs = run_inster(fingerprint, self.cycles, alpha_long)
        assert CI.CP.openpilotLongitudinalControl == alpha_long
        partial = {st.name for cp in CI.can_parsers.values() for st in cp.message_states.values() if st.undecoded}
        assert partial and partial <= set(CARSTATE_SIGNALS[CAR.HYUNDAI_INSTER])
        assert all(cp.can_valid for cp in CI.can_parsers.values())

        # same car state, actuators and CAN messages as with every signal decoded
        with mock.patch.dict(carstate.CARSTATE_SIGNALS, clear=True):
          _, outputs_full = run_inster(fingerprint, self.cycles, alpha_long)
        assert outputs == outputs_full

  def test_missing_signal_decoded_on_demand(self):
    # a signal missing from the list does not change any output: it is decoded on demand with one warning
    CANParser.strict_decode = False
    _, expected = run_inster(self.fingerprint, self.cycles)
    signals = dict(CARSTATE_SIGNALS[CAR.HYUNDAI_INSTER])
    signals["WHL_SPD11"] = tuple(s for s in signals["WHL_SPD11"] if s != "WHL_SPD_RR")
    signals["AX1_CRUISE_BUTTONS"] = tuple(s for s in signals["AX1_CRUISE_BUTTONS"] if s != "CRUISE_BUTTONS")
    with mock.patch.dict(carstate.CARSTATE_SIGNALS, {CAR.HYUNDAI_INSTER: signals}), \
         self.assertLogs("carlog", level="WARNING") as logs:
      _, outputs = run_inster(self.fingerprint, self.cycles)
    assert outputs == expected
    warnings = [line for line in logs.output if "decode_only" in line]
    assert len(warnings) == 2
    assert any("WHL_SPD_RR" in line for line in warnings) and any("CRUISE_BUTTONS" in line for line in warnings)

    # the strict mode of the tests raises instead
    CANParser.strict_decode = True
    with mock.patch.dict(carstate.CARSTATE_SIGNALS, {CAR.HYUNDAI_INSTER: signals}), self.assertRaises(UndecodedSignalError):
      run_inster(self.fingerprint, self.cycles)


if __name__ == "__main__":
  unittest.main()
