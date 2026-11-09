import random
import unittest
from unittest.mock import patch

from opendbc.can import CANPacker, CANParser
from opendbc.can.parser import UndecodedSignalError
from opendbc.car import Bus, DT_CTRL, gen_empty_fingerprint, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.structs import CarParams, CarParamsSP
from opendbc.car.vin import VIN_UNKNOWN
from opendbc.car.fw_versions import build_fw_dict, fw_request_order, get_fw_versions_ordered, match_fw_to_car
from opendbc.car.hyundai import hyundaican
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.carstate import AX1_CRUISE_BUTTONS, AX1_PAUSE_RESUME_BUTTON
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.radar_interface import HYUNDAI_RADAR_TRACK_SPECS, RADAR_210_21F, RADAR_235_248, RADAR_3A5_3C4, \
                                                RADAR_500_51F, RADAR_602_617, RADAR_TRACK_BUSES, HyundaiRadarTrackConfig, \
                                                HyundaiRadarTrackSpec, RadarInterface, get_detected_radar_tracks, \
                                                get_radar_can_parser
from opendbc.car.hyundai.values import CAR, DATE_FW_ECUS, \
                                         FW_QUERY_CONFIG, CANFD_FUZZY_WHITELIST, \
                                         PLATFORM_CODE_ECUS, HYUNDAI_VERSION_REQUEST_LONG, \
                                         HyundaiFlags, get_platform_codes, HyundaiSafetyFlags, Buttons, DBC, \
                                         NON_SCC_CAR, CarControllerParams, INSTER_STEER_MAX, INSTER_STEER_DELTA_UP, \
                                         INSTER_STEER_DELTA_DOWN, INSTER_STEER_ACTUATOR_DELAY, INSTER_STEER_FULL_SCALE
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.testing import fuzzy_test

Ecu = CarParams.Ecu

# Some platforms have date codes in a different format we don't yet parse (or are missing).
# For now, assert list of expected missing date cars
NO_DATES_PLATFORMS = {
  # CAN FD
  CAR.KIA_SPORTAGE_5TH_GEN,
  CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN,
  CAR.HYUNDAI_TUCSON_4TH_GEN,
  # CAN
  CAR.HYUNDAI_ELANTRA,
  CAR.HYUNDAI_ELANTRA_GT_I30,
  CAR.KIA_CEED,
  CAR.KIA_CEED_PHEV,
  CAR.KIA_FORTE,
  CAR.KIA_OPTIMA_G4,
  CAR.KIA_OPTIMA_G4_FL,
  CAR.KIA_SORENTO,
  CAR.HYUNDAI_KONA,
  CAR.HYUNDAI_KONA_EV,
  CAR.HYUNDAI_KONA_EV_2022,
  CAR.HYUNDAI_KONA_HEV,
  CAR.HYUNDAI_SONATA_LF,
  CAR.HYUNDAI_VELOSTER,
  CAR.HYUNDAI_KONA_2022,
}

CANFD_EXPECTED_ECUS = {Ecu.fwdCamera, Ecu.fwdRadar}

# CAN-only feature flags that should not appear on CAN FD platforms
CAN_FEATURE_FLAGS = (HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.TCU_GEARS | HyundaiFlags.CHECKSUM_CRC8 |
                     HyundaiFlags.CHECKSUM_6B | HyundaiFlags.LEGACY | HyundaiFlags.UNSUPPORTED_LONGITUDINAL |
                     HyundaiFlags.CAMERA_SCC)


def cars_with(flags):
  return {c for c in CAR if c.config.flags & flags}


def add_radar_range(fingerprint, radar_spec, bus, missing_addr=None):
  for addr in range(radar_spec.start_addr, radar_spec.end_addr + 1):
    if addr != missing_addr:
      fingerprint[bus][addr] = 8


def mark_track_updated(parser, radar_spec, addr, prefix=""):
  msg_name = f"RADAR_TRACK_{addr:x}"
  signal = "DISTANCE" if radar_spec == RADAR_602_617 else "LONG_DIST"
  parser.ts_nanos[msg_name][f"{prefix}{signal}"] = 1


def get_radar_parser(RI, radar_spec, bus):
  return next(rp for rp in RI.radar_parsers if rp.spec == radar_spec and rp.bus == bus)


class TestHyundaiFingerprint(unittest.TestCase):
  def test_feature_detection(self):
    # LKA steering
    for lka_steering in (True, False):
      fingerprint = gen_empty_fingerprint()
      if lka_steering:
        cam_can = CanBus(None, fingerprint).CAM
        fingerprint[cam_can] = [0x50, 0x110]  # LKA steering messages
      CP = CarInterface.get_params(CAR.KIA_EV6, fingerprint, [], False, False, False)
      assert bool(CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG) == lka_steering

    # radar available
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        add_radar_range(fingerprint, RADAR_500_51F, 1)
      CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_radar_track_detection(self):
    for radar_spec in HYUNDAI_RADAR_TRACK_SPECS:
      for bus in (0, 1, 2):
        with self.subTest(radar_spec=radar_spec.name, bus=bus):
          fingerprint = gen_empty_fingerprint()
          add_radar_range(fingerprint, radar_spec, bus)

          CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
          radar_tracks = get_detected_radar_tracks(CP)

          assert not CP.radarUnavailable
          assert len(radar_tracks) == 1
          assert radar_tracks[0].spec == radar_spec
          assert radar_tracks[0].bus == bus

  def test_radar_track_detection_requires_full_range(self):
    for radar_spec in HYUNDAI_RADAR_TRACK_SPECS:
      with self.subTest(radar_spec=radar_spec.name):
        fingerprint = gen_empty_fingerprint()
        add_radar_range(fingerprint, radar_spec, 1, missing_addr=radar_spec.end_addr)

        CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)

        assert CP.radarUnavailable
        assert get_detected_radar_tracks(CP) == ()

  def test_radar_track_detection_multiple_ranges(self):
    fingerprint = gen_empty_fingerprint()
    add_radar_range(fingerprint, RADAR_500_51F, 1)
    add_radar_range(fingerprint, RADAR_3A5_3C4, 2)

    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    radar_tracks = get_detected_radar_tracks(CP)

    assert not CP.radarUnavailable
    assert [(rt.spec, rt.bus) for rt in radar_tracks] == [(RADAR_500_51F, 1), (RADAR_3A5_3C4, 2)]

  def test_radar_track_detection_keeps_duplicate_range_candidates(self):
    fingerprint = gen_empty_fingerprint()
    add_radar_range(fingerprint, RADAR_500_51F, 0)
    add_radar_range(fingerprint, RADAR_500_51F, 1)
    add_radar_range(fingerprint, RADAR_500_51F, 2)

    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    radar_tracks = get_detected_radar_tracks(CP)

    assert not CP.radarUnavailable
    assert [(rt.spec, rt.bus) for rt in radar_tracks] == [(RADAR_500_51F, 0), (RADAR_500_51F, 1), (RADAR_500_51F, 2)]

  def test_radar_interface_multiple_parsers(self):
    fingerprint = gen_empty_fingerprint()
    add_radar_range(fingerprint, RADAR_500_51F, 1)
    add_radar_range(fingerprint, RADAR_3A5_3C4, 2)
    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    CP_SP = CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value)

    RI = RadarInterface(CP, CP_SP)

    radar_500_51f_parser = get_radar_parser(RI, RADAR_500_51F, 1)
    radar_3a5_3c4_parser = get_radar_parser(RI, RADAR_3A5_3C4, 2)
    radar_500_51f_msg = radar_500_51f_parser.parser.vl[f"RADAR_TRACK_{RADAR_500_51F.start_addr:x}"]
    radar_500_51f_msg["STATE"] = 3
    radar_500_51f_msg["LONG_DIST"] = 10
    radar_500_51f_msg["AZIMUTH"] = 0
    radar_500_51f_msg["REL_SPEED"] = 1
    radar_500_51f_msg["REL_ACCEL"] = 0
    mark_track_updated(radar_500_51f_parser.parser, RADAR_500_51F, RADAR_500_51F.start_addr)

    radar_3a5_3c4_msg = radar_3a5_3c4_parser.parser.vl[f"RADAR_TRACK_{RADAR_3A5_3C4.start_addr:x}"]
    radar_3a5_3c4_msg["STATE"] = 3
    radar_3a5_3c4_msg["LONG_DIST"] = 20
    radar_3a5_3c4_msg["LAT_DIST"] = 1
    radar_3a5_3c4_msg["REL_SPEED"] = 2
    radar_3a5_3c4_msg["REL_ACCEL"] = 0
    mark_track_updated(radar_3a5_3c4_parser.parser, RADAR_3A5_3C4, RADAR_3A5_3C4.start_addr)

    points = RI._update([]).points

    assert len(points) == 2
    assert ("RADAR_500_51F", RADAR_500_51F.start_addr) in RI.pts
    assert ("RADAR_3A5_3C4", RADAR_3A5_3C4.start_addr) in RI.pts

  def test_radar_interface_duplicate_range_selects_populated_bus(self):
    fingerprint = gen_empty_fingerprint()
    add_radar_range(fingerprint, RADAR_210_21F, 0)
    add_radar_range(fingerprint, RADAR_210_21F, 2)
    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    CP_SP = CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value)

    RI = RadarInterface(CP, CP_SP)

    bus0_parser = get_radar_parser(RI, RADAR_210_21F, 0).parser
    bus2_parser = get_radar_parser(RI, RADAR_210_21F, 2).parser
    msg = bus2_parser.vl[f"RADAR_TRACK_{RADAR_210_21F.start_addr:x}"]
    msg["1_STATE"] = 3
    msg["1_LONG_DIST"] = 10
    msg["1_LAT_DIST"] = 1
    msg["1_REL_SPEED"] = 2
    mark_track_updated(bus2_parser, RADAR_210_21F, RADAR_210_21F.start_addr, "1_")

    mark_track_updated(bus0_parser, RADAR_210_21F, RADAR_210_21F.start_addr, "1_")

    points = RI._update({(0, RADAR_210_21F.start_addr), (2, RADAR_210_21F.start_addr)}).points

    assert len(points) == 1
    assert RI.active_radar_buses[RADAR_210_21F.name] == 2

  def test_radar_interface_detects_late_live_range(self):
    fingerprint = gen_empty_fingerprint()
    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    CP_SP = CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value)

    RI = RadarInterface(CP, CP_SP)
    radar_parser = get_radar_parser(RI, RADAR_235_248, 1)
    assert len(RI._update([]).points) == 0

    for addr in range(RADAR_235_248.start_addr, RADAR_235_248.end_addr + 1):
      RI.seen_radar_addrs.setdefault((RADAR_235_248.name, 1), set()).add(addr)

    msg = radar_parser.parser.vl[f"RADAR_TRACK_{RADAR_235_248.start_addr:x}"]
    msg["STATE"] = 3
    msg["LONG_DIST"] = 10
    msg["LAT_DIST"] = 1
    msg["REL_SPEED"] = 2
    mark_track_updated(radar_parser.parser, RADAR_235_248, RADAR_235_248.start_addr)

    points = RI._update({(1, RADAR_235_248.start_addr)}).points

    assert len(points) == 1
    assert RI.active_radar_buses[RADAR_235_248.name] == 1

  def test_radar_interface_two_track_ranges(self):
    for radar_spec in (RADAR_210_21F, RADAR_602_617):
      with self.subTest(radar_spec=radar_spec.name):
        fingerprint = gen_empty_fingerprint()
        add_radar_range(fingerprint, radar_spec, 1)
        CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
        CP_SP = CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value)
        RI = RadarInterface(CP, CP_SP)

        parser = get_radar_parser(RI, radar_spec, 1).parser
        msg = parser.vl[f"RADAR_TRACK_{radar_spec.start_addr:x}"]
        if radar_spec == RADAR_210_21F:
          for i in ("1_", "2_"):
            msg[f"{i}STATE"] = 3
            msg[f"{i}LONG_DIST"] = 10
            msg[f"{i}LAT_DIST"] = 1
            msg[f"{i}REL_SPEED"] = 2
            mark_track_updated(parser, radar_spec, radar_spec.start_addr, i)
        else:
          for addr in range(radar_spec.start_addr, radar_spec.end_addr + 1):
            for i in ("1_", "2_"):
              parser.vl[f"RADAR_TRACK_{addr:x}"][f"{i}DISTANCE"] = 255.75
          for i in ("1_", "2_"):
            msg[f"{i}DISTANCE"] = 10
            msg[f"{i}LATERAL"] = 1
            msg[f"{i}SPEED"] = 2
            mark_track_updated(parser, radar_spec, radar_spec.start_addr, i)

        points = RI._update([]).points

        assert len(points) == 2

  def test_radar_interface_fingerprinted_range_only(self):
    # a range found at fingerprinting is used as is: no candidate parsers for other layouts or buses
    fingerprint = gen_empty_fingerprint()
    add_radar_range(fingerprint, RADAR_3A5_3C4, 1)
    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    RI = RadarInterface(CP, CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value))

    assert [(rp.spec, rp.bus) for rp in RI.radar_parsers] == [(RADAR_3A5_3C4, 1)]
    assert RI.rcp is RI.radar_parsers[0].parser

  def test_radar_interface_scans_candidates_without_fingerprinted_range(self):
    fingerprint = gen_empty_fingerprint()
    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    RI = RadarInterface(CP, CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value))

    expected = [(spec, bus) for spec in HYUNDAI_RADAR_TRACK_SPECS for bus in RADAR_TRACK_BUSES]
    assert [(rp.spec, rp.bus) for rp in RI.radar_parsers] == expected

  def test_radar_parsers_decode_only_read_signals(self):
    # the track parsers decode only the signals the radar interface reads, with the same values, timestamps and
    # validity as a parser of the whole DBC message
    rng = random.Random(0)
    for radar_spec in HYUNDAI_RADAR_TRACK_SPECS:
      with self.subTest(radar_spec=radar_spec.name):
        bus = 1
        slim = get_radar_can_parser(HyundaiRadarTrackConfig(radar_spec, bus))
        full = CANParser(radar_spec.dbc_name, [(f"RADAR_TRACK_{addr:x}", 50) for addr in radar_spec.address_range], bus)

        for state in slim.message_states.values():
          assert {sig.name for sig in state.signals} == radar_spec.signal_names
          assert set(dict.keys(slim.vl[state.name])) == radar_spec.signal_names
          assert set(dict.keys(slim.ts_nanos[state.name])) == radar_spec.signal_names

        for cycle in range(1, 51):
          frames = [(addr, bytes(rng.getrandbits(8) for _ in range(full.message_states[addr].size)), bus)
                    for addr in radar_spec.address_range if rng.random() < 0.7]
          frames.append((radar_spec.start_addr, b"\x00" * full.message_states[radar_spec.start_addr].size, bus + 1))
          strings = [(cycle * 10_000_000, frames)]
          assert slim.update(strings) == full.update(strings)
          assert slim.can_valid == full.can_valid
          for addr in radar_spec.address_range:
            for name in radar_spec.signal_names:
              assert slim.vl[addr][name] == full.vl[addr][name]
              assert slim.ts_nanos[addr][name] == full.ts_nanos[addr][name]

        # a signal outside the list is decoded on demand (the strict mode of the tests raises instead)
        state = slim.message_states[radar_spec.start_addr]
        dropped = next(name for name in full.vl[state.name] if name not in radar_spec.signal_names)
        try:
          CANParser.strict_decode = True
          with self.assertRaises(UndecodedSignalError):
            _ = slim.vl[state.name][dropped]
        finally:
          CANParser.strict_decode = False
        with self.assertLogs("carlog", level="WARNING"):
          assert slim.vl[state.name][dropped] == full.vl[state.name][dropped]

  def test_radar_parser_missing_signal_raises(self):
    spec = HyundaiRadarTrackSpec("RADAR_3A5_3C4", 0x3A5, 32, ("",), ("STATE", "NOT_A_SIGNAL"))
    with self.assertRaises(RuntimeError):
      get_radar_can_parser(HyundaiRadarTrackConfig(spec, 1))

  def test_alternate_limits(self):
    # Alternate lateral control limits, for high torque cars, verify Panda safety mode flag is set
    fingerprint = gen_empty_fingerprint()
    for car_model in CAR:
      CP = CarInterface.get_params(car_model, fingerprint, [], False, False, False)
      assert bool(CP.flags & HyundaiFlags.ALT_LIMITS) == bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.ALT_LIMITS)

  def test_inster_limits(self):
    # The Inster has its own lateral limit entry; only it sets the flag, and the Panda safety flag must match
    fingerprint = gen_empty_fingerprint()
    for car_model in CAR:
      CP = CarInterface.get_params(car_model, fingerprint, [], False, False, False)
      is_inster = car_model == CAR.HYUNDAI_INSTER
      assert bool(CP.flags & HyundaiFlags.INSTER_LIMITS) == is_inster
      assert bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.INSTER_LIMITS) == is_inster

    CP = CarInterface.get_params(CAR.HYUNDAI_INSTER, fingerprint, [], False, False, False)
    params = CarControllerParams(CP)
    assert (params.STEER_MAX, params.STEER_DELTA_UP, params.STEER_DELTA_DOWN) == \
           (INSTER_STEER_MAX, INSTER_STEER_DELTA_UP, INSTER_STEER_DELTA_DOWN)

  def test_inster_full_scale(self):
    # the Inster: cap 409 with actuators.torque 1.0 = 384 raw; every other car: full scale = cap (unchanged)
    fingerprint = gen_empty_fingerprint()
    assert (INSTER_STEER_MAX, INSTER_STEER_FULL_SCALE) == (409, 384)
    for car_model in CAR:
      CP = CarInterface.get_params(car_model, fingerprint, [], False, False, False)
      params = CarControllerParams(CP)
      expected = INSTER_STEER_FULL_SCALE if car_model == CAR.HYUNDAI_INSTER else params.STEER_MAX
      assert params.STEER_FULL_SCALE == expected, car_model

  def test_inster_car_controller_cap(self):
    # requests beyond 1.0 reach 409 raw on the Inster (1.0 = 384), never more; other cars still cap at 1.0 = their STEER_MAX
    fingerprint = gen_empty_fingerprint()
    for car_model, request, expected in ((CAR.HYUNDAI_INSTER, -1.5, -409), (CAR.HYUNDAI_INSTER, 1.0, 384),
                                         (CAR.HYUNDAI_INSTER, 409 / 384, 409), (CAR.HYUNDAI_INSTER, 1.07, 409),
                                         (CAR.HYUNDAI_KONA_EV, 1.5, 1.0), (CAR.KIA_CEED, -1.0, -1.0), (CAR.HYUNDAI_ELANTRA, 1.3, 1.0)):
      CP = CarInterface.get_params(car_model, fingerprint, [], False, False, False)
      if car_model != CAR.HYUNDAI_INSTER:
        expected = int(expected * CarControllerParams(CP).STEER_MAX)
      CP_SP = CarInterface.get_params_sp(CP, car_model, fingerprint, [], False, False, False)
      CI = CarInterface(CP, CP_SP)
      CC = structs.CarControl()
      CC.enabled = True
      CC.latActive = True
      CC.actuators.torque = request
      CC = CC.as_reader()
      outputs = []
      now_nanos = 0
      for _ in range(200):
        CI.update([])
        CI.CS.out.steeringTorque = 0
        actuators, _ = CI.apply(CC, structs.CarControlSP(), now_nanos)
        outputs.append(actuators.torqueOutputCan)
        now_nanos += int(DT_CTRL * 1e9)
      full_scale = INSTER_STEER_FULL_SCALE if car_model == CAR.HYUNDAI_INSTER else CarControllerParams(CP).STEER_MAX
      assert outputs[-1] == expected, (car_model, request, outputs[-1])
      assert max(abs(o) for o in outputs) == abs(expected)
      self.assertAlmostEqual(actuators.torque, expected / full_scale, places=6)

  def test_inster_steer_actuator_delay(self):
    # The Inster has its own default steering delay (INSTER_STEER_ACTUATOR_DELAY); the other cars keep theirs
    fingerprint = gen_empty_fingerprint()
    CP = CarInterface.get_params(CAR.HYUNDAI_INSTER, fingerprint, [], False, False, False)
    self.assertAlmostEqual(CP.steerActuatorDelay, INSTER_STEER_ACTUATOR_DELAY, places=6)
    for car_model in (CAR.HYUNDAI_KONA_EV, CAR.KIA_CEED):
      CP = CarInterface.get_params(car_model, fingerprint, [], False, False, False)
      self.assertAlmostEqual(CP.steerActuatorDelay, 0.1, places=6)

  def test_alt_ax1ev_buttons(self):
    fingerprint = gen_empty_fingerprint()
    for car_model in CAR:
      CP = CarInterface.get_params(car_model, fingerprint, [], False, False, False)
      is_ax1ev = car_model == CAR.HYUNDAI_INSTER
      assert bool(CP.flags & HyundaiFlags.ALT_AX1EV_BUTTONS) == is_ax1ev
      assert bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.ALT_AX1EV_BUTTONS) == is_ax1ev

    assert AX1_CRUISE_BUTTONS == {
      0: Buttons.NONE,
      1: Buttons.RES_ACCEL,
      2: Buttons.SET_DECEL,
      3: Buttons.GAP_DIST,
    }
    assert AX1_PAUSE_RESUME_BUTTON == Buttons.CANCEL

    CP = CarInterface.get_params(CAR.HYUNDAI_INSTER, fingerprint, [], False, False, False)
    packer = CANPacker(DBC[CAR.HYUNDAI_INSTER][Bus.pt])
    expected_payloads = {
      Buttons.RES_ACCEL: b"\x00\x00\x00\x00\x00\x00\x00\x10",
      Buttons.SET_DECEL: b"\x00\x00\x00\x00\x00\x00\x00\x20",
      Buttons.GAP_DIST: b"\x00\x00\x00\x00\x00\x00\x00\x30",
      Buttons.CANCEL: b"\x00\x00\x00\x00\x00\x00\x00\x40",
    }
    for button, expected_payload in expected_payloads.items():
      addr, payload, bus = hyundaican.create_cruise_button(packer, 0, {}, button, CP)
      assert addr == 0x3EF
      assert payload == expected_payload
      assert bus == 2

  def test_lfahda_mfc_hda_passthrough(self):
    # stock AX1EV camera frame: HDA_USM=2, HDA_Icon_State=2 (green), LFA_SysWarning=5, LFA_Icon_State=2
    dbc = DBC[CAR.HYUNDAI_INSTER][Bus.pt]
    stock_dat = bytes.fromhex("1200050200000000")
    parser = CANParser(dbc, [("LFAHDA_MFC", 0)], 2)
    parser.update([(0, [CanData(0x485, stock_dat, 2)])])
    packer = CANPacker(dbc)

    # HDA state is forwarded, our LFA icon replaces the stock one and stock LFA warnings are dropped
    addr, payload, bus = hyundaican.create_lfahda_mfc(packer, True, 1, parser.vl["LFAHDA_MFC"])
    assert (addr, bus) == (0x485, 0)
    assert payload == bytes.fromhex("1200000100000000")

    addr, payload, bus = hyundaican.create_lfahda_mfc(packer, True, 1)
    assert payload == bytes.fromhex("0000000100000000")

  def test_can_features(self):
    for car_model in CAR:
      flags = car_model.config.flags
      # Test no EV/HEV with cluster/TCU gears (should all use ELECT_GEAR)
      if flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
        assert not (flags & (HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.TCU_GEARS))

      # Test CAN FD car not in CAN feature lists
      if flags & HyundaiFlags.CANFD:
        assert not (flags & CAN_FEATURE_FLAGS), "CAN FD car unexpectedly has a CAN feature flag"

  def test_hybrid_ev_flags(self):
    for car_model in CAR:
      flags = car_model.config.flags
      assert not (flags & HyundaiFlags.HYBRID and flags & HyundaiFlags.EV), "Shared cars between hybrid and EV"
      assert not (flags & HyundaiFlags.CANFD and flags & HyundaiFlags.HYBRID), \
        "Hard coding CAN FD cars as hybrid is no longer supported"

  def test_canfd_ecu_whitelist(self):
    # Asserts only expected Ecus can exist in database for CAN-FD cars
    for car_model in CAR:
      if not (car_model.config.flags & HyundaiFlags.CANFD):
        continue
      ecus = {fw[0] for fw in FW_VERSIONS[car_model].keys()}
      ecus_not_in_whitelist = ecus - CANFD_EXPECTED_ECUS
      ecu_strings = ", ".join([f"Ecu.{ecu}" for ecu in ecus_not_in_whitelist])
      assert len(ecus_not_in_whitelist) == 0, \
                       f"{car_model}: Car model has unexpected ECUs: {ecu_strings}"

  def test_blacklisted_parts(self):
    # Asserts no ECUs known to be shared across platforms exist in the database.
    # Tucson having Santa Cruz camera and EPS for example
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        if car_model == CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN:
          raise unittest.SkipTest("Skip checking Santa Cruz for its parts")

        for code, _ in get_platform_codes(ecus[(Ecu.fwdCamera, 0x7c4, None)]):
          if b"-" not in code:
            continue
          part = code.split(b"-")[1]
          assert not part.startswith(b'CW'), "Car has bad part number"

  def test_correct_ecu_response_database(self):
    """
    Assert standard responses for certain ECUs, since they can
    respond to multiple queries with different data
    """
    expected_fw_prefix = HYUNDAI_VERSION_REQUEST_LONG[1:]
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          assert all(fw.startswith(expected_fw_prefix) for fw in fws), \
                          f"FW from unexpected request in database: {(ecu, fws)}"

  @fuzzy_test(max_examples=100)
  def test_platform_codes_fuzzy_fw(self, fuzzy):
    """Ensure function doesn't raise an exception"""
    get_platform_codes(fuzzy.list(fuzzy.binary))

  def test_expected_platform_codes(self):
    # Ensures we don't accidentally add multiple platform codes for a car unless it is intentional
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          # Third and fourth character are usually EV/hybrid identifiers
          codes = {code.split(b"-")[0][:2] for code, _ in get_platform_codes(fws)}
          if car_model == CAR.HYUNDAI_PALISADE:
            assert codes == {b"LX", b"ON"}, f"Car has unexpected platform codes: {car_model} {codes}"
          elif car_model == CAR.HYUNDAI_KONA_EV and ecu[0] == Ecu.fwdCamera:
            assert codes == {b"OE", b"OS"}, f"Car has unexpected platform codes: {car_model} {codes}"
          else:
            assert len(codes) == 1, f"Car has multiple platform codes: {car_model} {codes}"

  # Tests for platform codes, part numbers, and FW dates which Hyundai will use to fuzzy
  # fingerprint in the absence of full FW matches:
  def test_platform_code_ecus_available(self):
    # TODO: add queries for these non-CAN FD cars to get EPS
    no_eps_platforms = cars_with(HyundaiFlags.CANFD) | {CAR.KIA_SORENTO, CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL, CAR.KIA_OPTIMA_H, CAR.KIA_K7_2017,
                                    CAR.KIA_OPTIMA_H_G4_FL, CAR.HYUNDAI_SONATA_LF, CAR.HYUNDAI_TUCSON, CAR.GENESIS_G90, CAR.GENESIS_G80, CAR.HYUNDAI_ELANTRA}

    # Asserts ECU keys essential for fuzzy fingerprinting are available on all platforms
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for platform_code_ecu in PLATFORM_CODE_ECUS:
          if platform_code_ecu in (Ecu.fwdRadar, Ecu.eps) and car_model == CAR.HYUNDAI_GENESIS:
            continue
          if platform_code_ecu == Ecu.eps and car_model in no_eps_platforms:
            continue
          if car_model in NON_SCC_CAR:
            continue
          assert platform_code_ecu in [e[0] for e in ecus]

  def test_fw_format(self):
    # Asserts:
    # - every supported ECU FW version returns one platform code
    # - every supported ECU FW version has a part number
    # - expected parsing of ECU FW dates

    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          if car_model in NON_SCC_CAR:
            continue

          codes = set()
          for fw in fws:
            result = get_platform_codes([fw])
            assert 1 == len(result), f"Unable to parse FW: {fw}"
            codes |= result

          if ecu[0] not in DATE_FW_ECUS or car_model in NO_DATES_PLATFORMS:
            assert all(date is None for _, date in codes)
          else:
            assert all(date is not None for _, date in codes)

          if car_model == CAR.HYUNDAI_GENESIS:
            raise unittest.SkipTest("No part numbers for car model")

          # Hyundai places the ECU part number in their FW versions, assert all parsable
          # Some examples of valid formats: b"56310-L0010", b"56310L0010", b"56310/M6300"
          assert all(b"-" in code for code, _ in codes), \
                          f"FW does not have part number: {fw}"

  def test_platform_codes_spot_check(self):
    # Asserts basic platform code parsing behavior for a few cases
    results = get_platform_codes([b"\xf1\x00DH LKAS 1.1 -150210"])
    assert results == {(b"DH", b"150210")}

    # Some cameras and all radars do not have dates
    results = get_platform_codes([b"\xf1\x00AEhe SCC H-CUP      1.01 1.01 96400-G2000         "])
    assert results == {(b"AEhe-G2000", None)}

    results = get_platform_codes([b"\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         "])
    assert results == {(b"CV1-CV000", None)}

    results = get_platform_codes([
      b"\xf1\x00DH LKAS 1.1 -150210",
      b"\xf1\x00AEhe SCC H-CUP      1.01 1.01 96400-G2000         ",
      b"\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         ",
    ])
    assert results == {(b"DH", b"150210"), (b"AEhe-G2000", None), (b"CV1-CV000", None)}

    results = get_platform_codes([
      b"\xf1\x00LX2 MFC  AT USA LHD 1.00 1.07 99211-S8100 220222",
      b"\xf1\x00LX2 MFC  AT USA LHD 1.00 1.08 99211-S8100 211103",
      b"\xf1\x00ON  MFC  AT USA LHD 1.00 1.01 99211-S9100 190405",
      b"\xf1\x00ON  MFC  AT USA LHD 1.00 1.03 99211-S9100 190720",
    ])
    assert results == {(b"LX2-S8100", b"220222"), (b"LX2-S8100", b"211103"),
                               (b"ON-S9100", b"190405"), (b"ON-S9100", b"190720")}

  def test_fuzzy_excluded_platforms(self):
    # Asserts a list of platforms that will not fuzzy fingerprint with platform codes due to them being shared.
    # This list can be shrunk as we combine platforms and detect features
    excluded_platforms = {
      CAR.GENESIS_G70,            # shared platform code, part number, and date
      CAR.GENESIS_G70_2020,
    }
    excluded_platforms |= cars_with(HyundaiFlags.CANFD) - cars_with(HyundaiFlags.EV) - CANFD_FUZZY_WHITELIST  # shared platform codes
    excluded_platforms |= NO_DATES_PLATFORMS  # date codes are required to match

    platforms_with_shared_codes = set()
    for platform, fw_by_addr in FW_VERSIONS.items():
      car_fw = []
      for ecu, fw_versions in fw_by_addr.items():
        ecu_name, addr, sub_addr = ecu
        for fw in fw_versions:
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

      if platform in NON_SCC_CAR:
        continue

      CP = CarParams(carFw=car_fw)
      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
      if len(matches) == 1:
        assert list(matches)[0] == platform
      else:
        platforms_with_shared_codes.add(platform)

    assert platforms_with_shared_codes == excluded_platforms

  # *** Hyundai Inster: a missed EPS answer is re-queried once (route 000000ff, 8 Nov 2026) ***

  @staticmethod
  def _inster_fw(eps=True, camera=True, radar=True, eps_version=None):
    # Same layout as the Inster's logged answers: camera + EPS on bus 0, the radar on bus 0 and bus 1, plus
    # logging-only answers (never matched). The EU car's ABS never answers. A refused request (NRC) or a timeout
    # gives no entry at all (IsoTpParallelQuery).
    db = FW_VERSIONS[CAR.HYUNDAI_INSTER]
    fw = []

    def add(ecu, addr, version, bus=0, logging=False, request=HYUNDAI_VERSION_REQUEST_LONG):
      fw.append(CarParams.CarFw(ecu=ecu, fwVersion=version, address=addr, brand='hyundai', bus=bus, logging=logging,
                                request=[request], obdMultiplexing=bus == 0))  # as logged: bus 0 multiplexed, bus 1 not

    if camera:
      add(Ecu.fwdCamera, 0x7c4, db[(Ecu.fwdCamera, 0x7c4, None)][0])
    if eps:
      add(Ecu.eps, 0x7d4, eps_version if eps_version is not None else db[(Ecu.eps, 0x7d4, None)][-1])
    if radar:
      add(Ecu.fwdRadar, 0x7d0, db[(Ecu.fwdRadar, 0x7d0, None)][0], bus=0)
      add(Ecu.fwdRadar, 0x7d0, db[(Ecu.fwdRadar, 0x7d0, None)][0], bus=1)
    add(Ecu.eps, 0x7d4, b'\x25\x01\x01', logging=True, request=b'\x22\xf1\x8b')  # manufacturing date: logging only
    return fw

  def _run_inster_query(self, passes):
    # Runs get_fw_versions_ordered with the Hyundai passes given (other brands answer nothing), returns (queried brands, matches)
    queried = []
    hyundai_passes = iter(passes)

    def fake_get_fw_versions(can_recv, can_send, set_obd_multiplexing, query_brand=None, **kwargs):
      queried.append(query_brand)
      return next(hyundai_passes) if query_brand == 'hyundai' else []

    present = {(0x7c4 + 8, None, 0), (0x7d4 + 8, None, 0), (0x7d0 + 8, None, 0), (0x7c6 + 8, None, 0)}
    with patch('opendbc.car.fw_versions.get_fw_versions', side_effect=fake_get_fw_versions):
      car_fw = get_fw_versions_ordered(None, None, None, VIN_UNKNOWN, present)
    return queried, match_fw_to_car(car_fw, VIN_UNKNOWN, log=False)

  def test_inster_fw_rff_answers_do_not_match(self):
    # Route 000000ff: camera + radar answered with the database versions, the EPS version answer was missing
    assert match_fw_to_car(self._inster_fw(eps=False), VIN_UNKNOWN, log=False) == (True, set())
    assert match_fw_to_car(self._inster_fw(), VIN_UNKNOWN, log=False) == (True, {CAR.HYUNDAI_INSTER})

  def test_inster_fw_retry_recovers_missed_eps(self):
    queried, matches = self._run_inster_query([self._inster_fw(eps=False), self._inster_fw()])
    assert queried.count('hyundai') == 2 and queried[-1] == 'hyundai'  # one retry, then no other brand
    assert matches == (True, {CAR.HYUNDAI_INSTER})

  def test_inster_fw_retry_keeps_request_order(self):
    # The recovered EPS version answer goes before the EPS manufacturing-date answer, as in one complete pass
    # (sunnypilot's NNLC reads the first EPS entry of carFw)
    db = FW_VERSIONS[CAR.HYUNDAI_INSTER]
    passes = iter([self._inster_fw(eps=False), self._inster_fw()])
    present = {(0x7c4 + 8, None, 0), (0x7d4 + 8, None, 0), (0x7d0 + 8, None, 0)}
    with patch('opendbc.car.fw_versions.get_fw_versions', side_effect=lambda *_, query_brand=None, **__:
               next(passes) if query_brand == 'hyundai' else []):
      car_fw = get_fw_versions_ordered(None, None, None, VIN_UNKNOWN, present)
    eps = [fw for fw in car_fw if fw.ecu == Ecu.eps]
    assert [fw.logging for fw in eps] == [False, True]
    assert eps[0].fwVersion in db[(Ecu.eps, 0x7d4, None)]
    key = fw_request_order('hyundai')
    assert [key(fw) for fw in car_fw] == sorted(key(fw) for fw in car_fw)

  def test_inster_fw_normal_start_no_retry(self):
    queried, matches = self._run_inster_query([self._inster_fw()])
    assert queried.count('hyundai') == 1 and queried[-1] == 'hyundai'
    assert matches == (True, {CAR.HYUNDAI_INSTER})

  def test_inster_fw_eps_missing_twice_one_retry(self):
    queried, matches = self._run_inster_query([self._inster_fw(eps=False), self._inster_fw(eps=False)])
    assert queried.count('hyundai') == 2
    assert all(queried.count(brand) == 1 for brand in set(queried) - {'hyundai'})  # other brands: no retry
    assert matches == (True, set())

  def test_inster_fw_unknown_eps_version_fails(self):
    other_eps = FW_VERSIONS[CAR.HYUNDAI_KONA_EV_2022][(Ecu.eps, 0x7d4, None)][0]
    unknown = self._inster_fw(eps_version=other_eps)
    for desc, passes in (("unknown in both passes", [unknown, unknown]),
                         ("missing, then unknown", [self._inster_fw(eps=False), unknown]),
                         # the retry never adds a second answer to a request that was answered in the first pass
                         ("unknown, then known", [unknown, self._inster_fw()])):
      with self.subTest(desc):
        queried, matches = self._run_inster_query(passes)
        assert queried.count('hyundai') == 2
        assert matches == (True, set())
