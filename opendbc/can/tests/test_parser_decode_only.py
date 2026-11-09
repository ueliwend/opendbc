import copy
import pickle
import random
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.can.parser import UndecodedSignalError

DBC_NAME = "hyundai_canfd_generated"  # messages with checksum and counter signals
MESSAGES = ("WHEEL_SPEEDS", "STEERING_SENSORS")
KEPT = {"WHEEL_SPEEDS": ("WHL_SpdFLVal", "WHL_SpdRRVal"), "STEERING_SENSORS": ("STEERING_ANGLE",)}


UNDECODED = "WHL_SpdFRVal"  # a signal of WHEEL_SPEEDS outside KEPT


def _copied(d):
  return type(d).__name__, list(dict.items(d))


ACCESS_PATHS = {
  "d[sig]": lambda d, sig: d[sig],
  "d.get(sig)": lambda d, sig: d.get(sig),
  "sig in d": lambda d, sig: sig in d,
  "iter(d)": lambda d, sig: list(d),
  "reversed(d)": lambda d, sig: list(reversed(d)),
  "len(d)": lambda d, sig: len(d),
  "bool(d)": lambda d, sig: bool(d),
  "d.keys()": lambda d, sig: list(d.keys()),
  "d.values()": lambda d, sig: list(d.values()),
  "d.items()": lambda d, sig: list(d.items()),
  "dict(d)": lambda d, sig: list(dict(d).items()),
  "{**d}": lambda d, sig: list({**d}.items()),
  "d | {}": lambda d, sig: _copied(d | {}),
  "{} | d": lambda d, sig: _copied({} | d),
  "d == d.copy()": lambda d, sig: d == dict(zip(list(d), [d[k] for k in list(d)], strict=True)),
  "repr(d)": lambda d, sig: repr(d),
  "d.copy()": lambda d, sig: _copied(d.copy()),
  "copy.copy(d)": lambda d, sig: _copied(copy.copy(d)),
  "copy.deepcopy(d)": lambda d, sig: _copied(copy.deepcopy(d)),
  "pickle": lambda d, sig: _copied(pickle.loads(pickle.dumps(d))),
}


class TestCanParserDecodeOnly(unittest.TestCase):
  def setUp(self):
    self.rng = random.Random(0)
    self.packer = CANPacker(DBC_NAME)
    self.full = CANParser(DBC_NAME, [], 0)
    self.slim = CANParser(DBC_NAME, [], 0, decode_only=KEPT)
    self.addresses = [self.full.dbc.name_to_msg[name].address for name in MESSAGES]
    for parser in (self.full, self.slim):
      for name in MESSAGES:
        parser.vl[name]  # registered lazily, as car states do

  def tearDown(self):
    CANParser.strict_decode = False

  def frames(self):
    frames = []
    for name in MESSAGES:
      for _ in range(self.rng.randrange(3)):  # none, one or two frames per update
        msg = self.full.dbc.name_to_msg[name]
        values = {sig: self.rng.randrange(1 << min(msg.sigs[sig].size, 16)) * msg.sigs[sig].factor
                  for sig in msg.sigs if sig not in ("CHECKSUM", "COUNTER")}
        addr, dat, bus = self.packer.make_can_msg(name, 0, values)
        if self.rng.random() < 0.1:  # bad checksum or counter: the frame must be rejected by both parsers
          dat = bytes([dat[0] ^ 0xFF]) + dat[1:]
        frames.append((addr, dat, bus))
    self.rng.shuffle(frames)
    return frames

  def step(self, cycle):
    strings = [(cycle * 10_000_000, self.frames())]
    assert self.slim.update(strings) == self.full.update(strings)
    assert self.slim.can_valid == self.full.can_valid
    assert self.slim.bus_timeout == self.full.bus_timeout

  def assert_decoded_equal(self):
    for addr in self.addresses:
      for name in dict.keys(self.slim.vl[addr]):
        assert self.slim.vl[addr][name] == self.full.vl[addr][name]
        assert self.slim.ts_nanos[addr][name] == self.full.ts_nanos[addr][name]
        assert self.slim.vl_all[addr][name] == self.full.vl_all[addr][name]

  def test_decodes_only_listed_signals(self):
    for name in MESSAGES:
      state = self.slim.message_states[self.slim.dbc.name_to_msg[name].address]
      # counter and checksum signals are always decoded: they decide whether a frame is accepted
      assert {sig.name for sig in state.signals} == {*KEPT[name], "CHECKSUM", "COUNTER"}
    for cycle in range(1, 200):
      self.step(cycle)
      self.assert_decoded_equal()
    assert sum(len(st.signals) for st in self.slim.message_states.values()) < \
           sum(len(st.signals) for st in self.full.message_states.values())

  def test_registration_unchanged(self):
    parser = CANParser(DBC_NAME, [("STEERING_SENSORS", 100)], 0, decode_only=KEPT)
    assert set(parser.message_states) == {parser.dbc.name_to_msg["STEERING_SENSORS"].address}
    assert parser.message_states[parser.dbc.name_to_msg["STEERING_SENSORS"].address].frequency == 100

  def test_undecoded_signal_read_mid_drive(self):
    for cycle in range(1, 60):
      self.step(cycle)
    with self.assertLogs("carlog", level="WARNING") as logs:
      assert self.slim.vl["WHEEL_SPEEDS"]["WHL_SpdFRVal"] == self.full.vl["WHEEL_SPEEDS"]["WHL_SpdFRVal"]
      assert self.slim.vl_all["WHEEL_SPEEDS"]["WHL_SpdRLVal"] == self.full.vl_all["WHEEL_SPEEDS"]["WHL_SpdRLVal"]
      assert self.slim.ts_nanos["STEERING_SENSORS"]["STEERING_RATE"] == self.full.ts_nanos["STEERING_SENSORS"]["STEERING_RATE"]
      for cycle in range(60, 120):
        self.step(cycle)
        self.assert_decoded_equal()
        self.slim.vl["WHEEL_SPEEDS"]["WHL_SpdFRVal"]
    decode_warnings = [line for line in logs.output if "decode_only" in line]
    assert len(decode_warnings) == 3  # one warning per signal, then it is decoded with the others
    assert "WHL_SpdFRVal" in decode_warnings[0]
    for addr in self.addresses:
      assert list(dict.keys(self.slim.vl[addr])) == [sig.name for sig in self.slim.message_states[addr].signals]

  def test_undecoded_signal_read_before_first_frame(self):
    with self.assertLogs("carlog", level="WARNING"):
      assert self.slim.vl["WHEEL_SPEEDS"]["WHL_SpdFRVal"] == 0.0
      assert self.slim.ts_nanos["WHEEL_SPEEDS"]["WHL_SpdFRVal"] == 0
      assert self.slim.vl_all["STEERING_SENSORS"]["STEERING_RATE"] == []
    for cycle in range(1, 60):
      self.step(cycle)
      self.assert_decoded_equal()

  def test_access_paths(self):
    # every way of reading the vl, vl_all and ts_nanos dicts of a partial message gives what full decoding gives (the
    # undecoded signal is decoded, one warning), and raises in strict mode
    for family in ("vl", "vl_all", "ts_nanos"):
      for path, read in ACCESS_PATHS.items():
        for strict in (False, True):
          with self.subTest(family=family, path=path, strict=strict):
            self.setUp()
            for cycle in range(1, 30):
              self.step(cycle)
            assert self.full.message_states[self.addresses[0]].vals  # frames were accepted
            slim_d = getattr(self.slim, family)["WHEEL_SPEEDS"]
            full_d = getattr(self.full, family)["WHEEL_SPEEDS"]
            CANParser.strict_decode = strict
            if strict:
              with self.assertRaises(UndecodedSignalError):
                read(slim_d, UNDECODED)
              continue
            with self.assertLogs("carlog", level="WARNING") as logs:
              assert read(slim_d, UNDECODED) == read(full_d, UNDECODED)
            assert len([line for line in logs.output if "decode_only" in line]) == 1
            for cycle in range(30, 60):
              self.step(cycle)
              self.assert_decoded_equal()
            # the healed signal is decoded from now on: same value, history and timestamp, no further warning
            assert UNDECODED in dict.keys(slim_d)
            assert slim_d[UNDECODED] == full_d[UNDECODED]

  def test_reads_of_other_keys(self):
    self.step(1)
    assert "NOT_A_SIGNAL" not in self.slim.vl["STEERING_SENSORS"]
    assert self.slim.vl["STEERING_SENSORS"].get("NOT_A_SIGNAL", 1.5) == 1.5
    with self.assertRaises(KeyError) as e:
      self.slim.vl["STEERING_SENSORS"]["NOT_A_SIGNAL"]
    assert not isinstance(e.exception, UndecodedSignalError)
    # vl_all is a defaultdict(list) for keys that are not signals, as for a whole message
    assert self.slim.vl_all["STEERING_SENSORS"]["NOT_A_SIGNAL"] == self.full.vl_all["STEERING_SENSORS"]["NOT_A_SIGNAL"] == []
    CANParser.strict_decode = True
    assert self.slim.vl["WHEEL_SPEEDS"]["WHL_SpdFLVal"] == self.full.vl["WHEEL_SPEEDS"]["WHL_SpdFLVal"]
    assert self.slim.vl_all["WHEEL_SPEEDS"].get("WHL_SpdFLVal") == self.full.vl_all["WHEEL_SPEEDS"].get("WHL_SpdFLVal")

  def test_deepcopy_of_parser_stays_partial(self):
    for cycle in range(1, 20):
      self.step(cycle)
    clone = copy.deepcopy(self.slim)
    addr = self.addresses[0]
    assert clone.message_states[addr].undecoded.keys() == self.slim.message_states[addr].undecoded.keys()
    strings = [(20 * 10_000_000, self.frames())]
    clone.update(strings)
    self.full.update(strings)
    with self.assertLogs("carlog", level="WARNING"):
      assert clone.vl[addr]["WHL_SpdFRVal"] == self.full.vl[addr]["WHL_SpdFRVal"]
    assert "WHL_SpdFRVal" in self.slim.message_states[addr].undecoded

  def test_invalid_names_raise_at_startup(self):
    with self.assertRaises(RuntimeError):
      CANParser(DBC_NAME, [], 0, decode_only={"NOT_A_MESSAGE": ("X",)})
    with self.assertRaises(RuntimeError):
      CANParser(DBC_NAME, [], 0, decode_only={"WHEEL_SPEEDS": ("WHL_SpdFLVal", "NOT_A_SIGNAL")})


if __name__ == "__main__":
  unittest.main()
