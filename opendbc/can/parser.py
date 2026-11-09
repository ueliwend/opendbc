import copy
import math
import numbers
from collections import defaultdict, deque
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from typing import ClassVar

from opendbc.car.carlog import carlog
from opendbc.can.dbc import DBC, Signal, SignalType


MAX_BAD_COUNTER = 5
CAN_INVALID_CNT = 5


@dataclass
class MessageState:
  address: int
  name: str
  size: int
  signals: list[Signal]
  ignore_alive: bool = False
  ignore_checksum: bool = False
  ignore_counter: bool = False
  frequency: float = 0.0
  timeout_threshold: float = 1e5  # default to 1Hz threshold
  vals: list[float] = field(default_factory=list)
  all_vals: list[list[float]] = field(default_factory=list)
  timestamps: deque[int] = field(default_factory=lambda: deque(maxlen=500))
  counter: int = 0
  counter_fail: int = 0
  first_seen_nanos: int = 0
  last_warning_log_nanos: int = 0
  # Signals of the message that are not decoded (see CANParser decode_only), and the frames needed to decode them on demand:
  # the frames accepted in the current update() call and the last frame accepted before it.
  undecoded: dict[str, Signal] = field(default_factory=dict)
  frames: list[bytes] = field(default_factory=list)
  last_frame: bytes = b""

  def rate_limited_log(self, last_update_nanos: int, msg: str) -> None:
    if (last_update_nanos - self.last_warning_log_nanos) >= 1_000_000_000:
      carlog.warning(f"CANParser: {hex(self.address)} {self.name} {msg}")
      self.last_warning_log_nanos = last_update_nanos

  def parse(self, nanos: int, dat: bytes) -> bool:
    tmp_vals: list[float] = [0.0] * len(self.signals)
    checksum_failed = False
    counter_failed = False

    if self.first_seen_nanos == 0:
      self.first_seen_nanos = nanos

    for i, sig in enumerate(self.signals):
      tmp = sig.get_raw_value(dat)
      if sig.is_signed:
        tmp -= ((tmp >> (sig.size - 1)) & 0x1) * (1 << sig.size)

      if not self.ignore_checksum and sig.calc_checksum is not None:
        expected_checksum = sig.calc_checksum(self.address, sig, bytearray(dat))
        if tmp != expected_checksum:
          checksum_failed = True
          self.rate_limited_log(nanos, f"checksum failed: received {hex(tmp)}, calculated {hex(expected_checksum)}")

      if not self.ignore_counter and sig.type == 1:  # COUNTER
        if not self.update_counter(tmp, sig.size):
          counter_failed = True

      tmp_vals[i] = tmp * sig.factor + sig.offset

    # must have good counter and checksum to update data
    if checksum_failed or counter_failed:
      return False

    if not self.vals:
      self.vals = [0.0] * len(self.signals)
      self.all_vals = [[] for _ in self.signals]

    for i, v in enumerate(tmp_vals):
      self.vals[i] = v
      self.all_vals[i].append(v)

    self.timestamps.append(nanos)
    if self.undecoded:
      self.frames.append(dat)

    if self.frequency < 1e-5 and len(self.timestamps) >= 3:
      dt = (self.timestamps[-1] - self.timestamps[0]) * 1e-9
      if (dt > 1.0 or (self.timestamps.maxlen is not None and len(self.timestamps) >= self.timestamps.maxlen)) and dt != 0:
        self.frequency = min(len(self.timestamps) / dt, 100.0)
        self.timeout_threshold = (1_000_000_000 / self.frequency) * 10
    return True

  def update_counter(self, cur_count: int, cnt_size: int) -> bool:
    if ((self.counter + 1) & ((1 << cnt_size) - 1)) != cur_count:
      self.counter_fail = min(self.counter_fail + 1, MAX_BAD_COUNTER)
    elif self.counter_fail > 0:
      self.counter_fail -= 1
    self.counter = cur_count
    return self.counter_fail < MAX_BAD_COUNTER

  def valid(self, current_nanos: int, bus_timeout: bool) -> bool:
    if self.ignore_alive:
      return True
    if not self.timestamps:
      return False
    if (current_nanos - self.timestamps[-1]) > self.timeout_threshold:
      return False
    return True


def decode_signal(sig: Signal, dat: bytes) -> float:
  # same arithmetic as MessageState.parse
  tmp = sig.get_raw_value(dat)
  if sig.is_signed:
    tmp -= ((tmp >> (sig.size - 1)) & 0x1) * (1 << sig.size)
  return tmp * sig.factor + sig.offset


def _plain_copy(d: dict) -> dict:
  # dict.copy() of a dict subclass that overrides iteration goes through its keys(), which decodes all signals
  return dict(zip(dict.keys(d), dict.values(d), strict=True))


class UndecodedSignalError(KeyError):
  """Strict mode (CANParser.strict_decode): a signal outside the parser's decode set was read."""


class _PartialSignals:
  """Reads of the per-message dicts of a message that decodes only part of its signals (CANParser decode_only).

  They behave as if every signal were decoded: a read that names an undecoded signal (d[sig], get, in) decodes it from the
  stored frames, with the values full decoding would have given, and from then on it is decoded with the others.
  Whole-dict reads (iteration, keys/values/items, len, reversed, ==, repr, |, copies, pickling) first decode all signals.
  In strict mode (CANParser.strict_decode) these reads raise UndecodedSignalError instead. Writes (item assignment, del,
  pop, setdefault, update, clear) are the parser's own and are not covered.
  """
  __slots__ = ()
  _parser: "CANParser"
  _address: int

  def _decode(self, key) -> None:
    self._parser._decode_undecoded(self._address, (key,))

  def _decode_all(self) -> None:
    self._parser._decode_undecoded(self._address, None)

  def _new_copy(self, items: dict) -> dict:
    raise NotImplementedError

  def get(self, key, default=None):
    if not dict.__contains__(self, key):
      self._decode(key)
    return dict.get(self, key, default)

  def __contains__(self, key) -> bool:
    if not dict.__contains__(self, key):
      self._decode(key)
    return dict.__contains__(self, key)

  def __iter__(self):
    self._decode_all()
    return dict.__iter__(self)

  def __reversed__(self):
    self._decode_all()
    return dict.__reversed__(self)

  def __len__(self) -> int:
    self._decode_all()
    return dict.__len__(self)

  def keys(self):
    self._decode_all()
    return dict.keys(self)

  def values(self):
    self._decode_all()
    return dict.values(self)

  def items(self):
    self._decode_all()
    return dict.items(self)

  def __eq__(self, other):
    self._decode_all()
    if isinstance(other, _PartialSignals):
      other._decode_all()
    return dict.__eq__(self, other)  # type: ignore[arg-type]

  def __ne__(self, other):
    eq = self.__eq__(other)
    return eq if eq is NotImplemented else not eq

  __hash__ = None  # type: ignore[assignment]

  def __repr__(self) -> str:
    return repr(self.copy())

  def __or__(self, other):
    if not isinstance(other, dict):
      return NotImplemented
    return self.copy() | other

  def __ror__(self, other):
    if not isinstance(other, dict):
      return NotImplemented
    result = self._new_copy(dict(other))
    result.update(self.copy())
    return result

  def copy(self) -> dict:
    self._decode_all()
    return self._new_copy(_plain_copy(self))  # type: ignore[arg-type]

  __copy__ = copy

  def __deepcopy__(self, memo):
    parser = memo.get(id(self._parser))
    if parser is not None:  # part of a deep copy of the whole parser: stays partial, bound to the copy
      return type(self)(parser, self._address, copy.deepcopy(_plain_copy(self), memo))  # type: ignore[call-arg, arg-type]
    return copy.deepcopy(self.copy(), memo)


class PartialSignalDict(_PartialSignals, dict):
  """vl / ts_nanos dict of a message that decodes only part of its signals; copies are plain dicts, as before."""
  __slots__ = ("_parser", "_address")

  def __init__(self, parser: "CANParser", address: int, values: dict):
    super().__init__(values)
    self._parser = parser
    self._address = address

  def _new_copy(self, items: dict) -> dict:
    return items

  def __reduce_ex__(self, protocol):
    return dict, (self.copy(),)

  def __missing__(self, key):
    self._decode(key)
    if dict.__contains__(self, key):
      return dict.__getitem__(self, key)
    raise KeyError(key)


class PartialSignalHistory(_PartialSignals, defaultdict):
  """vl_all dict of a message that decodes only part of its signals; like the defaultdict(list) of a whole message, a
  signal has a history once a frame was accepted, and copies are defaultdicts."""
  __slots__ = ("_parser", "_address")

  def __init__(self, parser: "CANParser", address: int, values: Iterable = ()):
    super().__init__(list, values)
    self._parser = parser
    self._address = address

  def _new_copy(self, items: dict) -> dict:
    return defaultdict(list, items)

  def __reduce_ex__(self, protocol):
    return defaultdict, (list, _plain_copy(self.copy()))

  def __missing__(self, key):
    self._decode(key)
    if dict.__contains__(self, key):
      return dict.__getitem__(self, key)
    return defaultdict.__missing__(self, key)


class VLDict(dict):
  def __init__(self, parser):
    super().__init__()
    self.parser = parser

  def __getitem__(self, key):
    if key not in self:
      self.parser._add_message(key)
    return super().__getitem__(key)


class CANParser:
  # Tests set this: a read of a signal outside a decode_only set raises UndecodedSignalError instead of decoding it.
  strict_decode: ClassVar[bool] = False

  def __init__(self, dbc_name: str, messages: list[tuple[str | int, int]], bus: int,
               decode_only: Mapping[str | int, Iterable[str]] | None = None):
    """decode_only: {message name or address: signal names} limits decoding of those messages (whenever they are registered)
    to the given signals plus their counter and checksum signals; other messages are decoded whole. Which messages are
    registered, frame acceptance, timestamps and validity do not depend on it. A read of a signal that is not decoded
    decodes it on demand and logs one warning (in strict mode it raises), so a missing name never changes a value.
    """
    self.dbc_name: str = dbc_name
    self.bus: int = bus
    self.dbc = DBC(dbc_name)
    self.decode_only: dict[int, frozenset[str]] = {}
    for name_or_addr, signal_names in (decode_only or {}).items():
      if isinstance(name_or_addr, numbers.Number):
        msg = self.dbc.addr_to_msg.get(int(name_or_addr))
      else:
        msg = self.dbc.name_to_msg.get(name_or_addr)
      if msg is None:
        raise RuntimeError(f"decode_only: could not find message {name_or_addr!r} in DBC {dbc_name}")
      names = frozenset(signal_names)
      missing = names - msg.sigs.keys()
      if missing:
        raise RuntimeError(f"decode_only: {msg.name} in DBC {dbc_name} lacks signals {sorted(missing)}")
      self.decode_only[msg.address] = names

    self.vl: dict[int | str, dict[str, float]] = VLDict(self)
    self.vl_all: dict[int | str, dict[str, list[float]]] = {}
    self.ts_nanos: dict[int | str, dict[str, int]] = {}
    self.addresses: set[int] = set()
    self.message_states: dict[int, MessageState] = {}
    self._partial_states: list[MessageState] = []

    for name_or_addr, freq in messages:
      if isinstance(name_or_addr, numbers.Number):
        msg = self.dbc.addr_to_msg.get(int(name_or_addr))
      else:
        msg = self.dbc.name_to_msg.get(name_or_addr)
      if msg is None:
        raise RuntimeError(f"could not find message {name_or_addr!r} in DBC {dbc_name}")
      if msg.address in self.addresses:
        raise RuntimeError("Duplicate Message Check: %d" % msg.address)

      self._add_message(name_or_addr, freq)

    self.can_invalid_cnt: int = CAN_INVALID_CNT
    self.last_nonempty_nanos: int = 0
    self._last_update_nanos: int = 0

  def _add_message(self, name_or_addr: str | int, freq: int | None = None) -> None:
    if isinstance(name_or_addr, numbers.Number):
      msg = self.dbc.addr_to_msg.get(int(name_or_addr))
    else:
      msg = self.dbc.name_to_msg.get(name_or_addr)
    assert msg is not None
    assert msg.address not in self.addresses

    self.addresses.add(msg.address)
    signals = list(msg.sigs.values())
    undecoded: dict[str, Signal] = {}
    kept_names = self.decode_only.get(msg.address)
    if kept_names is not None:
      undecoded = {sig.name: sig for sig in signals if not self._always_decoded(sig, kept_names)}
      signals = [sig for sig in signals if sig.name not in undecoded]

    signal_names = [sig.name for sig in signals]
    if undecoded:
      signals_dict: dict[str, float] = PartialSignalDict(self, msg.address, {s: 0.0 for s in signal_names})
      self.vl_all[msg.address] = PartialSignalHistory(self, msg.address)
      self.ts_nanos[msg.address] = PartialSignalDict(self, msg.address, {s: 0 for s in signal_names})
    else:
      signals_dict = {s: 0.0 for s in signal_names}
      self.vl_all[msg.address] = defaultdict(list)
      self.ts_nanos[msg.address] = {s: 0 for s in signal_names}
    dict.__setitem__(self.vl, msg.address, signals_dict)
    dict.__setitem__(self.vl, msg.name, signals_dict)
    self.vl_all[msg.name] = self.vl_all[msg.address]
    self.ts_nanos[msg.name] = self.ts_nanos[msg.address]

    state = MessageState(
      address=msg.address,
      name=msg.name,
      size=msg.size,
      signals=signals,
      ignore_alive=freq is not None and math.isnan(freq),
      undecoded=undecoded,
    )
    if undecoded:
      self._partial_states.append(state)
    if freq is not None and freq > 0:
      state.frequency = freq
    else:
      # if frequency not specified, assume 1Hz until we learn it
      freq = 1
    state.timeout_threshold = (1_000_000_000 / freq) * 10

    self.message_states[msg.address] = state

  @staticmethod
  def _always_decoded(sig: Signal, kept_names: frozenset[str]) -> bool:
    # counter and checksum signals decide whether a frame is accepted
    return sig.name in kept_names or sig.type != SignalType.DEFAULT or sig.calc_checksum is not None

  def _decode_undecoded(self, address: int, names: Iterable[str] | None) -> bool:
    """Start decoding the given undecoded signals of a message (None: all of them), with the values, value history of
    the current update() call and timestamps that decoding them from the start would have given. False if none of the
    names is an undecoded signal of the message.
    """
    state = self.message_states[address]
    new = list(state.undecoded) if names is None else [n for n in names if isinstance(n, str) and n in state.undecoded]
    if not new:
      return False
    read = f"CANParser: {hex(address)} {state.name} (DBC {self.dbc_name}, bus {self.bus}): {sorted(new)} read"
    if self.strict_decode:
      raise UndecodedSignalError(f"{read}, but not in the parser's decode_only set")
    carlog.warning(f"{read}, but not in the decode_only set; decoding them from now on")

    received = bool(state.vals)  # a frame was accepted, so values, value history and a timestamp exist
    added = [state.undecoded.pop(n) for n in new]
    values = {sig.name: (state.vals[i], state.all_vals[i]) for i, sig in enumerate(state.signals)} if received else {}
    if received:
      last = state.frames[-1] if state.frames else state.last_frame
      for sig in added:
        values[sig.name] = (decode_signal(sig, last), [decode_signal(sig, dat) for dat in state.frames])

    # keep the DBC's signal order in the state and in the dicts; the dicts stay the same objects
    state.signals = [sig for sig in self.dbc.addr_to_msg[address].sigs.values() if sig.name not in state.undecoded]
    if received:
      state.vals = [values[sig.name][0] for sig in state.signals]
      state.all_vals = [values[sig.name][1] for sig in state.signals]
    new_names = {sig.name for sig in added}
    vl, vl_all, ts_nanos = self.vl[address], self.vl_all[address], self.ts_nanos[address]
    old_vl, old_ts, old_all = _plain_copy(vl), _plain_copy(ts_nanos), _plain_copy(vl_all)
    for d in (vl, ts_nanos, vl_all):
      dict.clear(d)
    for sig in state.signals:
      name = sig.name
      dict.__setitem__(vl, name, values[name][0] if received and name in new_names else old_vl.get(name, 0.0))
      dict.__setitem__(ts_nanos, name, state.timestamps[-1] if received and name in new_names else old_ts.get(name, 0))

    # vl_all: the signal histories exist once a frame was accepted (full decoding adds them all at the first frame); keys
    # that are not signals (created by defaultdict reads) keep their place before or after them
    def add_histories():
      for sig in state.signals:
        dict.__setitem__(vl_all, sig.name, values[sig.name][1] if sig.name in new_names else old_all[sig.name])

    dbc_signals = self.dbc.addr_to_msg[address].sigs
    added_histories = False
    for name, history in old_all.items():
      if not received or name not in dbc_signals:
        dict.__setitem__(vl_all, name, history)
      elif not added_histories:
        add_histories()
        added_histories = True
    if received and not added_histories:
      add_histories()
    return True

  @property
  def bus_timeout(self) -> bool:
    ignore_alive = all(s.ignore_alive for s in self.message_states.values())
    bus_timeout_threshold = 500 * 1_000_000
    for st in self.message_states.values():
      if st.timeout_threshold > 0:
        bus_timeout_threshold = min(bus_timeout_threshold, st.timeout_threshold)
    return ((self._last_update_nanos - self.last_nonempty_nanos) > bus_timeout_threshold) and not ignore_alive

  @property
  def can_valid(self) -> bool:
    valid = True
    counters_valid = True
    bus_timeout = self.bus_timeout
    for state in self.message_states.values():
      if state.counter_fail >= MAX_BAD_COUNTER:
        counters_valid = False
        state.rate_limited_log(self._last_update_nanos, f"counter invalid, {state.counter_fail=} {MAX_BAD_COUNTER=}")
      if not state.valid(self._last_update_nanos, bus_timeout):
        valid = False
        state.rate_limited_log(self._last_update_nanos, "not valid (timeout or missing)")

    # TODO: probably only want to increment this once per update() call
    self.can_invalid_cnt = 0 if valid else min(self.can_invalid_cnt + 1, CAN_INVALID_CNT)
    return self.can_invalid_cnt < CAN_INVALID_CNT and counters_valid

  def update(self, strings, sendcan: bool = False):
    if strings and not isinstance(strings[0], list | tuple):
      strings = [strings]

    for addr in self.addresses:
      for history in dict.values(self.vl_all[addr]):  # dict.values: no decoding of a partial message's signals
        history.clear()
    for state in self._partial_states:
      if state.frames:
        state.last_frame = state.frames[-1]
        state.frames.clear()

    updated_addrs: set[int] = set()
    for entry in strings:
      t = entry[0]
      frames = entry[1]
      bus_empty = True
      for address, dat, src in frames:
        if src != self.bus:
          continue
        bus_empty = False
        state = self.message_states.get(address)
        if state is None or len(dat) > 64:
          continue
        if state.parse(t, dat):
          updated_addrs.add(address)

          vl_addr = self.vl[address]
          vl_all_addr = self.vl_all[address]
          ts_addr = self.ts_nanos[address]

          for i, sig in enumerate(state.signals):
            vl_addr[sig.name] = state.vals[i]
            vl_all_addr[sig.name] = state.all_vals[i]
            ts_addr[sig.name] = state.timestamps[-1]

      if not bus_empty:
        self.last_nonempty_nanos = t

      self._last_update_nanos = t

    return updated_addrs


class CANDefine:
  def __init__(self, dbc_name: str):
    dbc = DBC(dbc_name)

    dv = defaultdict(dict)
    for val in dbc.vals:
      sgname = val.name
      address = val.address
      msg = dbc.addr_to_msg.get(address)
      if msg is None:
        raise KeyError(address)
      msgname = msg.name
      parts = val.def_val.split()
      values = [int(v) for v in parts[::2]]
      defs = parts[1::2]
      dv[address][sgname] = dict(zip(values, defs, strict=True))
      dv[msgname][sgname] = dv[address][sgname]

    self.dv = dict(dv)
