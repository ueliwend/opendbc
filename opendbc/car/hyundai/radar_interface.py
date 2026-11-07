import math
from dataclasses import dataclass

from opendbc.can import CANParser
from opendbc.can.dbc import SignalType
from opendbc.car import structs
from opendbc.car.interfaces import RadarInterfaceBase

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

RADAR_TRACK_BUSES = (0, 1, 2)


# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


@dataclass(frozen=True)
class HyundaiRadarTrackSpec:
  name: str
  start_addr: int
  msg_count: int
  track_prefixes: tuple[str, ...]
  # per-track signals read by decode_radar_track, is_radar_track_valid and get_track_ts_nanos (without prefix)
  track_signals: tuple[str, ...]

  @property
  def end_addr(self) -> int:
    return self.start_addr + self.msg_count - 1

  @property
  def address_range(self) -> range:
    return range(self.start_addr, self.end_addr + 1)

  @property
  def signal_names(self) -> frozenset[str]:
    return frozenset(f"{prefix}{sig}" for prefix in self.track_prefixes for sig in self.track_signals)

  @property
  def dbc_name(self) -> str:
    return f"hyundai_{self.name.lower()}_generated"


@dataclass(frozen=True)
class HyundaiRadarTrackConfig:
  spec: HyundaiRadarTrackSpec
  bus: int


HyundaiRadarTrackKey = tuple[str, int]


@dataclass
class HyundaiRadarParserConfig:
  spec: HyundaiRadarTrackSpec
  bus: int
  parser: CANParser


RADAR_500_51F = HyundaiRadarTrackSpec("RADAR_500_51F", 0x500, 32, ("",), ("STATE", "AZIMUTH", "LONG_DIST", "REL_SPEED"))
RADAR_210_21F = HyundaiRadarTrackSpec("RADAR_210_21F", 0x210, 16, ("1_", "2_"), ("STATE", "LONG_DIST", "LAT_DIST", "REL_SPEED"))
RADAR_235_248 = HyundaiRadarTrackSpec("RADAR_235_248", 0x235, 20, ("",), ("STATE", "LONG_DIST", "LAT_DIST", "REL_SPEED"))
RADAR_3A5_3C4 = HyundaiRadarTrackSpec("RADAR_3A5_3C4", 0x3A5, 32, ("",), ("STATE", "LONG_DIST", "LAT_DIST", "REL_SPEED"))
RADAR_602_617 = HyundaiRadarTrackSpec("RADAR_602_617", 0x602, 16, ("1_", "2_"), ("DISTANCE", "LATERAL", "SPEED"))

HYUNDAI_RADAR_TRACK_SPECS = (RADAR_500_51F, RADAR_210_21F, RADAR_235_248, RADAR_3A5_3C4, RADAR_602_617)
DEFAULT_RADAR_TRIGGER_MSG = max(rt.end_addr for rt in HYUNDAI_RADAR_TRACK_SPECS)

DETECTED_RADAR_TRACKS: dict[str, tuple[HyundaiRadarTrackConfig, ...]] = {}


def detect_radar_tracks(fingerprint: dict[int, dict[int, int]]) -> tuple[HyundaiRadarTrackConfig, ...]:
  detected: list[HyundaiRadarTrackConfig] = []
  for radar_spec in HYUNDAI_RADAR_TRACK_SPECS:
    for bus in RADAR_TRACK_BUSES:
      if all(addr in fingerprint[bus] for addr in radar_spec.address_range):
        detected.append(HyundaiRadarTrackConfig(radar_spec, bus))
  return tuple(detected)


def set_detected_radar_tracks(car_fingerprint: str, radar_tracks: tuple[HyundaiRadarTrackConfig, ...]) -> None:
  DETECTED_RADAR_TRACKS[car_fingerprint] = radar_tracks


def get_detected_radar_tracks(CP) -> tuple[HyundaiRadarTrackConfig, ...]:
  return DETECTED_RADAR_TRACKS.get(CP.carFingerprint, ())


def decode_only_signals(parser: CANParser, signal_names: frozenset[str]) -> None:
  """Restrict every message of a radar track parser to the given signals.

  The track messages carry ~28 signals, of which the radar interface reads 3-4 per track; decoding the rest costs
  most of the radar parsing time. Counter and checksum signals are kept, since they decide whether a frame is
  accepted, so validity and timestamps are unchanged. Dropped signals are also removed from vl and ts_nanos: reading
  one raises KeyError instead of returning a value that is never updated.
  """
  for state in parser.message_states.values():
    missing = signal_names - {sig.name for sig in state.signals}
    if missing:
      raise RuntimeError(f"{state.name} in DBC {parser.dbc_name} lacks radar track signals {sorted(missing)}")

    state.signals = [sig for sig in state.signals
                     if sig.name in signal_names or sig.type != SignalType.DEFAULT or sig.calc_checksum is not None]
    kept = {sig.name for sig in state.signals}
    for values in (parser.vl[state.address], parser.ts_nanos[state.address]):
      for name in [name for name in values if name not in kept]:
        del values[name]


def get_radar_can_parser(radar_track: HyundaiRadarTrackConfig) -> CANParser:
  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in radar_track.spec.address_range]
  parser = CANParser(radar_track.spec.dbc_name, messages, radar_track.bus)
  decode_only_signals(parser, radar_track.spec.signal_names)
  return parser


def get_all_radar_track_candidates() -> tuple[HyundaiRadarTrackConfig, ...]:
  return tuple(HyundaiRadarTrackConfig(radar_spec, bus) for radar_spec in HYUNDAI_RADAR_TRACK_SPECS for bus in RADAR_TRACK_BUSES)


def get_track_storage_key(radar_spec: HyundaiRadarTrackSpec, addr: int, track_prefix: str) -> HyundaiRadarTrackKey:
  if radar_spec.name in ("RADAR_500_51F", "RADAR_235_248", "RADAR_3A5_3C4"):
    return (radar_spec.name, addr)

  track_index = int(track_prefix[0]) - 1
  return (radar_spec.name, addr * 2 + track_index)


def get_track_ts_nanos(parser: CANParser, msg_name: str, radar_spec: HyundaiRadarTrackSpec, track_prefix: str) -> int:
  if radar_spec.name == "RADAR_602_617":
    return parser.ts_nanos[msg_name][f"{track_prefix}DISTANCE"]
  if radar_spec.name in ("RADAR_210_21F", "RADAR_235_248"):
    return parser.ts_nanos[msg_name][f"{track_prefix}LONG_DIST"]
  return parser.ts_nanos[msg_name]["LONG_DIST"]


def decode_radar_track(radar_spec: HyundaiRadarTrackSpec, track_msg, track_prefix: str) -> tuple[float, float, float]:
  if radar_spec.name == "RADAR_602_617":
    return (
      track_msg[f"{track_prefix}DISTANCE"],
      track_msg[f"{track_prefix}LATERAL"],
      track_msg[f"{track_prefix}SPEED"],
    )

  if radar_spec.name == "RADAR_210_21F":
    return (
      track_msg[f"{track_prefix}LONG_DIST"],
      track_msg[f"{track_prefix}LAT_DIST"],
      track_msg[f"{track_prefix}REL_SPEED"],
    )

  if radar_spec.name == "RADAR_500_51F":
    azimuth = math.radians(track_msg["AZIMUTH"])
    long_dist = track_msg["LONG_DIST"]
    return (
      math.cos(azimuth) * long_dist,
      0.5 * -math.sin(azimuth) * long_dist,
      track_msg["REL_SPEED"],
    )

  # RADAR_235_248, RADAR_3A5_3C4
  return (
    track_msg["LONG_DIST"],
    track_msg["LAT_DIST"],
    track_msg["REL_SPEED"],
  )


def is_radar_track_valid(radar_spec: HyundaiRadarTrackSpec, track_msg, track_prefix: str) -> bool:
  if radar_spec.name == "RADAR_602_617":
    return track_msg[f"{track_prefix}DISTANCE"] != 255.75

  if radar_spec.name == "RADAR_210_21F":
    return track_msg[f"{track_prefix}STATE"] in (3, 4)

  if radar_spec.name == "RADAR_235_248":
    return track_msg["STATE"] in (3, 4)

  return track_msg["STATE"] in (3, 4)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.pts: dict[int | HyundaiRadarTrackKey, structs.RadarData.RadarPoint] = {}
    detected_tracks = get_detected_radar_tracks(CP)
    # Track ranges found at fingerprinting are used as they are. Only when none was found (radar silent or not yet
    # sending at startup) are all known layouts on all buses watched, so that a late radar is still picked up.
    scan_candidates = bool(CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR) and not detected_tracks
    candidate_tracks = get_all_radar_track_candidates() if scan_candidates else ()
    self.radar_tracks = tuple(dict.fromkeys((*detected_tracks, *candidate_tracks)))
    self.radar_parsers = tuple(HyundaiRadarParserConfig(rt.spec, rt.bus, get_radar_can_parser(rt)) for rt in self.radar_tracks)
    self.seen_radar_addrs = {(rt.spec.name, rt.bus): set(rt.spec.address_range) for rt in detected_tracks}
    self.updated_messages: set[tuple[int, int]] = set()
    self.active_radar_buses: dict[str, int] = {}
    self.trigger_msg = max((rt.spec.end_addr for rt in self.radar_tracks), default=DEFAULT_RADAR_TRIGGER_MSG)
    self.trigger_msg = self.get_trigger_msg(self.trigger_msg)
    self.track_id = 0

    self.radar_off_can = CP.radarUnavailable and not (CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR)
    self.rcp: CANParser | None = self.radar_parsers[0].parser if self.radar_parsers else None

    if self.CP_SP.flags & HyundaiFlagsSP.RADAR_LEAD_ONLY:
      self.initialize_radar_ext(self.trigger_msg)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    if self.CP_SP.flags & HyundaiFlagsSP.RADAR_LEAD_ONLY and self.use_radar_interface_ext:
      vls = self.rcp.update(can_strings)
      self.updated_messages.update((0, addr) for addr in vls)

      if (0, self.trigger_msg) not in self.updated_messages:
        return None

      rr = self._update(self.updated_messages)
      self.updated_messages.clear()
      return rr

    radar_updated = False
    for radar_parser in self.radar_parsers:
      vls = radar_parser.parser.update(can_strings)
      relevant_addrs = {
        addr for addr in vls
        if radar_parser.spec.start_addr <= addr <= radar_parser.spec.end_addr
      }
      if relevant_addrs:
        self.seen_radar_addrs.setdefault((radar_parser.spec.name, radar_parser.bus), set()).update(relevant_addrs)
        self.updated_messages.update((radar_parser.bus, addr) for addr in relevant_addrs)
        # tracks are sent in address order, publish once per cycle when the last track of a range arrives
        if radar_parser.spec.end_addr in relevant_addrs:
          radar_updated = True

    if not radar_updated:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()
    return rr

  def _radar_range_complete(self, radar_parser: HyundaiRadarParserConfig) -> bool:
    seen_addrs = self.seen_radar_addrs.get((radar_parser.spec.name, radar_parser.bus), set())
    return all(addr in seen_addrs for addr in radar_parser.spec.address_range)

  @staticmethod
  def _radar_parser_valid_count(radar_parser: HyundaiRadarParserConfig, updated_messages: set[tuple[int, int]]) -> int:
    valid_count = 0
    for addr in radar_parser.spec.address_range:
      if updated_messages and (radar_parser.bus, addr) not in updated_messages:
        continue

      msg_name = f"RADAR_TRACK_{addr:x}"
      msg = radar_parser.parser.vl[msg_name]
      for track_prefix in radar_parser.spec.track_prefixes:
        if get_track_ts_nanos(radar_parser.parser, msg_name, radar_parser.spec, track_prefix) == 0:
          continue
        if is_radar_track_valid(radar_parser.spec, msg, track_prefix):
          valid_count += 1
    return valid_count

  def _select_active_radar_parser(self, radar_parsers: list[HyundaiRadarParserConfig],
                                  updated_messages: set[tuple[int, int]]) -> HyundaiRadarParserConfig:
    valid_counts = {rp.bus: self._radar_parser_valid_count(rp, updated_messages) for rp in radar_parsers}
    current_bus = self.active_radar_buses.get(radar_parsers[0].spec.name)
    current_parser = next((rp for rp in radar_parsers if rp.bus == current_bus), None)

    best_parser = max(
      radar_parsers,
      key=lambda rp: (
        valid_counts[rp.bus],
        rp.bus == current_bus,
      ),
    )
    if valid_counts[best_parser.bus] == 0 and current_parser is not None:
      best_parser = current_parser

    self.active_radar_buses[best_parser.spec.name] = best_parser.bus
    return best_parser

  def _update_radar_points_from_parser(self, radar_parser: HyundaiRadarParserConfig) -> None:
    for addr in radar_parser.spec.address_range:
      msg_name = f"RADAR_TRACK_{addr:x}"
      msg = radar_parser.parser.vl[msg_name]
      for track_prefix in radar_parser.spec.track_prefixes:
        track_key = get_track_storage_key(radar_parser.spec, addr, track_prefix)
        if get_track_ts_nanos(radar_parser.parser, msg_name, radar_parser.spec, track_prefix) != 0 and \
           is_radar_track_valid(radar_parser.spec, msg, track_prefix):
          if track_key not in self.pts:
            self.pts[track_key] = structs.RadarData.RadarPoint()
            self.pts[track_key].trackId = self.track_id
            self.track_id += 1

          d_rel, y_rel, v_rel = decode_radar_track(radar_parser.spec, msg, track_prefix)
          pt = self.pts[track_key]
          pt.dRel = d_rel
          pt.yRel = y_rel
          pt.vRel = v_rel
        elif track_key in self.pts:
          del self.pts[track_key]

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if self.CP_SP.flags & HyundaiFlagsSP.RADAR_LEAD_ONLY:
      if self.use_radar_interface_ext:
        return self.update_ext(ret)

    active_radar_parsers: list[HyundaiRadarParserConfig] = []
    if self.CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR:
      radar_parsers_by_spec: dict[str, list[HyundaiRadarParserConfig]] = {}
      for radar_parser in self.radar_parsers:
        if not self._radar_range_complete(radar_parser):
          continue
        radar_parsers_by_spec.setdefault(radar_parser.spec.name, []).append(radar_parser)

      for radar_parsers in radar_parsers_by_spec.values():
        active_radar_parser = self._select_active_radar_parser(radar_parsers, updated_messages)
        active_radar_parsers.append(active_radar_parser)
        self._update_radar_points_from_parser(active_radar_parser)

    if any(not radar_parser.parser.can_valid for radar_parser in active_radar_parsers):
      ret.errors.canError = True

    ret.points = list(self.pts.values())
    return ret
