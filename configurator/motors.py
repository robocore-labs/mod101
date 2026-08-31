#!/usr/bin/env python3
"""Serial-bus motor layer for the configurator's calibration page.

Talks to the Feetech ST/STS chain directly over a USB-TTL adapter using the
`st3215` library (pip install st3215) — no intermediate firmware. The browser
never touches the serial port; it drives this over HTTP from server.py.

The one class here, `MotorBus`, is a thin normalizing wrapper. `st3215` is
inconsistent about return values and this is where that gets flattened:

  * `ReadSpeed`   returns a 3-tuple `(speed, comm, error)` while every other
                  `Read*` returns a scalar or None.
  * `StartServo`  returns `(comm, error)` while `StopServo` returns True/None.
  * `ChangeId`    returns None on SUCCESS and an error string on failure —
                  but "success" only means the bytes reached the UART, since
                  its EPROM ops are all TxOnly. See change_id(), which pings
                  the new ID to find out what actually happened.
  * `ReadVoltage` returns volts; `ReadLoad` returns a ±1023 duty cycle.

Bus access is serialized under a lock. `socketserver.TCPServer` handles one
request at a time today, but the telemetry poller is chatty enough that this
should not silently depend on that.
"""

import atexit
import fcntl
import glob
import os
import threading
import time

DEFAULT_SCAN_HI = 99      # servo IDs above this aren't scanned (ListServos
                          # pings 0..253, which takes seconds on a real bus)
# EEPROM registers and the comm status code, from st3215/values.py. Mirrored
# here rather than imported because st3215 is imported lazily, in connect().
REG_ID = 5
REG_OFS_L = 31            # position-correction offset, EEPROM, 2 bytes
REG_TORQUE_ENABLE = 40
REG_LOCK = 55
COMM_OK = 0
TICKS = 4096
MAX_OFS = 2047            # the offset register is 11 bits plus a sign bit
LOAD_FULL_SCALE = 1023.0  # ReadLoad duty-cycle range


class BusError(RuntimeError):
    """Any failure that should surface to the browser as a 4xx."""


def port_holders(device: str) -> list[tuple[int, str]]:
    """Every other process holding `device` open, as (pid, cmdline).

    Linux permits several processes to open the same tty, and then splits the
    incoming bytes between them at random — so a stray `miniterm`, a second
    server, or a ROS driver makes every servo reply come back truncated and the
    bus looks dead rather than busy. Nothing in the serial stack reports that,
    so check for it explicitly. Same method as fuser(1): walk /proc/*/fd.
    """
    target = os.path.realpath(device)
    me = os.getpid()
    out: list[tuple[int, str]] = []
    for fddir in glob.glob('/proc/[0-9]*/fd'):
        try:
            pid = int(fddir.split('/')[2])
        except ValueError:
            continue
        if pid == me:
            continue
        try:
            entries = os.listdir(fddir)
        except OSError:
            continue                      # not ours / already gone
        for fd in entries:
            try:
                if os.path.realpath(os.path.join(fddir, fd)) != target:
                    continue
                with open(f'/proc/{pid}/cmdline', 'rb') as fh:
                    cmd = fh.read().replace(b'\0', b' ').decode(
                        'utf-8', 'replace').strip()
                out.append((pid, cmd or f'pid {pid}'))
                break
            except OSError:
                break
    return out


class MotorBus:
    """Owns the serial port and every servo transaction on it."""

    def __init__(self):
        self._st = None
        self._port = None
        self._lock = threading.RLock()
        self._seen: list[int] = []
        atexit.register(self.disarm_all)

    # ---- lifecycle -----------------------------------------------------

    @property
    def connected(self) -> bool:
        return self._st is not None

    def _require(self):
        if self._st is None:
            raise BusError('not connected to a servo bus')
        return self._st

    def connect(self, port: str) -> dict:
        from st3215 import ST3215
        with self._lock:
            if self._st is not None:
                self.disconnect()

            holders = port_holders(port)
            if holders:
                who = '; '.join(f'pid {p} ({c[:70]})' for p, c in holders)
                raise BusError(
                    f'{port} is already open by {who}. Close it first — two '
                    'readers on one serial port split the servo replies '
                    'between them, so every reply arrives corrupt and the bus '
                    'looks empty.')

            try:
                self._st = ST3215(port)
            except Exception as e:
                # ST3215 raises bare ValueError('Could not open port: ...')
                raise BusError(f'could not open {port}: {e}') from e

            # Advisory lock, same mechanism as pyserial's exclusive=True (which
            # st3215 doesn't pass). Stops a *later* opener that also locks.
            try:
                fcntl.flock(self._st.portHandler.ser.fileno(),
                            fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                self._st = None
                raise BusError(f'{port} is locked by another process')

            self._port = port
            return self.scan()

    def disconnect(self) -> None:
        with self._lock:
            if self._st is None:
                return
            try:
                self.disarm_all()
            except Exception:
                pass
            try:
                self._st.portHandler.closePort()
            except Exception:
                pass
            self._st = None
            self._port = None
            self._seen = []

    def status(self) -> dict:
        return {
            'connected': self.connected,
            'port': self._port,
            'servos': list(self._seen),
        }

    # ---- discovery -----------------------------------------------------

    @staticmethod
    def list_ports() -> list[dict]:
        """Candidate serial devices, most likely first.

        Motherboards enumerate ~32 legacy /dev/ttyS* ports that are never a
        USB-TTL adapter; listing them buries the one real device in the UI. So
        keep only ports with actual USB hardware behind them, and fall back to
        the raw list if that filter would leave nothing.
        """
        try:
            from serial.tools import list_ports
        except Exception as e:
            # Returning [] here renders as "no serial devices found", which
            # sends you hunting for a cable when the real problem is a missing
            # dependency. Say which one it is.
            raise BusError(
                f'pyserial is not importable ({e}) — pip install st3215') from e
        every = []
        for p in list_ports.comports():
            # Surface who else has it open, so a port that would silently
            # produce corrupt replies is visibly busy before you pick it.
            holders = port_holders(p.device)
            every.append({
                'device': p.device,
                'description': p.description or '',
                'hwid': p.hwid or '',
                'usb': bool(getattr(p, 'vid', None)),
                'busy': [{'pid': pid, 'cmd': cmd} for pid, cmd in holders],
            })
        usb = [p for p in every if p['usb']]
        out = usb or every
        out.sort(key=lambda d: (not d['usb'], d['device']))
        return out

    def scan(self, hi: int = DEFAULT_SCAN_HI) -> dict:
        """Ping IDs 1..hi. Bounded on purpose — st3215's own ListServos()
        walks 0..253, which is seconds of dead time on a real bus."""
        st = self._require()
        with self._lock:
            found = [i for i in range(1, hi + 1) if st.PingServo(i)]
            self._seen = found
        return self.status()

    # ---- per-servo -----------------------------------------------------

    def telemetry(self, sid: int) -> dict:
        """Shape matches what the calibration page consumes. `online` False
        means the servo did not answer — the page treats that as a skip, not
        an error, so a momentary bus miss can't abort a sweep."""
        st = self._require()
        with self._lock:
            pos = st.ReadPosition(sid)
            if pos is None:
                return {'id': sid, 'online': False, 'pos': -1, 'speed': 0,
                        'current_ma': 0, 'voltage_mv': 0, 'load_dpct': 0,
                        'temp_c': 0}
            speed = st.ReadSpeed(sid)
            if isinstance(speed, tuple):        # (speed, comm, error)
                speed = speed[0]
            load = st.ReadLoad(sid)
            current = st.ReadCurrent(sid)
            voltage = st.ReadVoltage(sid)
            temp = st.ReadTemperature(sid)
        return {
            'id': sid,
            'online': True,
            'pos': int(pos),
            'speed': int(speed or 0),
            'current_ma': round(current or 0.0, 1),
            # ReadVoltage is volts; the page reads millivolts.
            'voltage_mv': int(round((voltage or 0.0) * 1000)),
            # ReadLoad is a ±1023 duty cycle; the page reads tenths of a percent.
            'load_dpct': int(round(abs(load or 0) / LOAD_FULL_SCALE * 1000)),
            'temp_c': int(temp or 0),
        }

    def correction(self, sid: int) -> dict:
        """The servo's stored position-correction offset (EEPROM, reg 31/32).

        Read-only, and separate from telemetry() on purpose: it is one more bus
        round trip per motor and it only changes when set_zero writes it, so
        putting it in the 50 Hz poll would spend the bus on a constant.
        """
        st = self._require()
        with self._lock:
            ofs = st.ReadCorrection(int(sid))
            pos = st.ReadPosition(int(sid))
        if ofs is None:
            raise BusError(f'servo {sid}: could not read its position correction')
        return {'id': int(sid), 'correction': int(ofs),
                'pos': None if pos is None else int(pos)}

    def torque(self, sid: int, on: bool) -> bool:
        st = self._require()
        with self._lock:
            if on:
                # StartServo hands back writeTxRx's raw (comm, error) tuple.
                res = st.StartServo(sid)
                ok = (tuple(res) == (0, 0)) if isinstance(res, tuple) else bool(res)
            else:
                ok = st.StopServo(sid) is True
        if not ok:
            raise BusError(f'servo {sid}: torque {"on" if on else "off"} failed')
        return True

    def move(self, sid: int, pos: int, speed: int | None = None,
             accel: int | None = None) -> bool:
        """Position goal. When speed/accel are omitted this is a single bus
        write — st3215's MoveTo() re-sends mode+accel+speed every call, which
        is three extra round trips per jog press."""
        st = self._require()
        pos = max(0, min(TICKS - 1, int(pos)))
        with self._lock:
            if speed is None and accel is None:
                ok = st.WritePosition(sid, pos) is True
            else:
                ok = st.MoveTo(sid, pos, speed=int(speed or 2400),
                               acc=int(accel or 50)) is True
        if not ok:
            raise BusError(f'servo {sid}: move to {pos} failed')
        return True

    def profile(self, sid: int, speed: int, accel: int) -> bool:
        st = self._require()
        with self._lock:
            ok_s = st.SetSpeed(sid, max(1, min(3400, int(speed)))) is True
            ok_a = st.SetAcceleration(sid, max(1, min(254, int(accel)))) is True
        if not (ok_s and ok_a):
            raise BusError(f'servo {sid}: profile write failed')
        return True

    def change_id(self, old: int, new: int) -> bool:
        """Reassign a servo's ID, with acknowledged writes and verification.

        This deliberately does NOT call st3215's ChangeId, which cannot be made
        reliable from the outside:

          * all three of its EPROM ops are write1ByteTxOnly — bytes go out the
            UART and no status packet is read back — so it returns None
            ("success") even when the servo rejected the write or never saw it;
          * it fires the ID write immediately after the unlock with no settle
            time, so the EEPROM unlock can still be in flight when the ID write
            arrives and gets rejected — which is the failure that actually
            showed up on this bench;
          * its closing LockEprom is addressed to `old`, which stops answering
            the instant the ID byte lands, so every servo it "succeeded" on is
            left with its EEPROM unlocked.

        Here the unlock is acked and read back, the ID write gets settle time
        and is proven by a ping at the new ID, and the re-lock goes to `new`.
        """
        st = self._require()
        old, new = int(old), int(new)
        # 0 is writable on this protocol but scan() starts at 1, so a servo set
        # to 0 vanishes from the configurator; 254 is the broadcast address.
        if not (1 <= new <= 253):
            raise BusError('new id must be 1..253')
        if old == new:
            return True

        with self._lock:
            # Two servos sharing an ID on a half-duplex bus both answer every
            # packet: reads come back corrupted and neither can be addressed on
            # its own to undo it. Only catches servos powered on RIGHT NOW —
            # if IDs are set one servo at a time, uniqueness is on the operator.
            if st.PingServo(new):
                raise BusError(f'id {new} is already in use — move that servo first')
            if not st.PingServo(old):
                raise BusError(f'servo {old} is not responding')

            # 1. Unlock the EEPROM, and insist on a status packet.
            res, _ = st.write1ByteTxRx(old, REG_LOCK, 0)
            if res != COMM_OK:
                raise BusError(f'servo {old}: EEPROM unlock was not acknowledged')

            # 2. Let the write commit, then PROVE it unlocked. Without this the
            #    next write is a coin flip that reports success either way.
            time.sleep(0.05)
            lock, res, _ = st.read1ByteTxRx(old, REG_LOCK)
            if res != COMM_OK:
                raise BusError(f'servo {old}: could not read back the EEPROM lock')
            if lock != 0:
                raise BusError(
                    f'servo {old}: EEPROM is still locked (reg {REG_LOCK} = {lock}) — '
                    f'id not changed')

            # 3. The ID write stays TxOnly on purpose: the servo adopts `new`
            #    as this packet is processed, so which ID its status packet
            #    would carry is ambiguous. The ping below is the real check.
            st.write1ByteTxOnly(old, REG_ID, new)
            time.sleep(0.05)

            for _ in range(10):
                if st.PingServo(new):
                    break
                time.sleep(0.05)
            else:
                raise BusError(
                    f'servo {old}: no response at id {new} after the write — '
                    f'id may be unchanged; rescan the bus')

            # 4. Re-lock at the NEW id — the step the library gets wrong.
            res, _ = st.write1ByteTxRx(new, REG_LOCK, 1)
            if res != COMM_OK:
                raise BusError(
                    f'servo is now at id {new} but the EEPROM did not re-lock')

            self._seen = sorted(set(self._seen) - {old} | {new})
        return True

    @staticmethod
    def _wrap_ofs(x: int) -> int:
        """An offset is only meaningful mod 4096, so fold it into what the
        register can actually hold."""
        v = ((int(x) % TICKS) + TICKS) % TICKS
        if v > TICKS // 2 - 1:
            v -= TICKS
        return max(-MAX_OFS, min(MAX_OFS, v))

    def set_zero(self, sid: int, target: int) -> dict:
        """Make the servo's CURRENT physical position read `target`.

        The servo carries a position-correction offset in EEPROM (REG_OFS):
        what it reports is the raw encoder count minus that offset. Rewriting
        it moves the whole reported frame, permanently and for every later
        reader — this page, the ROS driver, LeRobot — which is the only reason
        it is worth touching at all. A correction that lived in a config file
        would be undone by the next tool that didn't read that file.

        WHY A JOINT NEEDS IT. The encoder is absolute over one turn and its
        count wraps 4095 -> 0 at one physical angle, fixed by nothing but how
        the horn happened to spline on. If that seam lands inside a joint's
        travel, the calibrated range is an arc across it: min_ticks comes out
        numerically GREATER than max_ticks, the clamp and LeRobot's range are
        both nonsense, and st3215_manager refuses the group outright.

        WHY `target` IS A PARAMETER AND NOT 2048. The obvious move is the
        servo's own "define middle" command, which puts the held pose at
        mid-scale. That is right only for a joint whose zero pose sits in the
        middle of its travel. mod101's shoulder, elbow and wrist roll all run
        0..180 degrees FROM their zero pose, so centring their zero pushes the
        far end to 4096 — it creates the seam crossing it was meant to cure.
        The caller knows the joint's range and picks a target that fits the
        whole of it inside 0..4095; here that is just a number to hit.

        Returns the frame shift, which the caller owes to any tick it captured
        before this: every one of them moves by exactly that much.
        """
        st = self._require()
        sid, target = int(sid), int(target)
        if not (0 <= target < TICKS):
            raise BusError(f'target must be 0..{TICKS - 1}')
        with self._lock:
            if not st.PingServo(sid):
                raise BusError(f'servo {sid} is not responding')

            # A servo that is HOLDING a position must not be re-framed under
            # its own goal: the goal register keeps its number while the number
            # comes to mean a different angle, so the joint slews there the
            # instant the offset lands. Refuse rather than disarm — dropping
            # torque on an arm that is holding itself up is its own accident.
            armed, res, _ = st.read1ByteTxRx(sid, REG_TORQUE_ENABLE)
            if res != COMM_OK:
                raise BusError(f'servo {sid}: could not read its torque state')
            if armed:
                raise BusError(
                    f'servo {sid} is under torque — disarm it and hold the joint by '
                    f'hand before setting its zero')

            before = st.ReadPosition(sid)
            if before is None or before < 0:
                raise BusError(f'servo {sid}: could not read its position')
            ofs0 = st.ReadCorrection(sid)
            if ofs0 is None:
                raise BusError(f'servo {sid}: could not read its position correction')

            # reported = raw - ofs, so to report `target` at the raw count we
            # are sitting on now: ofs_new = (before + ofs0) - target.
            ofs_new = self._wrap_ofs(before + ofs0 - target)

            # REG_OFS is in the EEPROM block, so unlock first — and prove it,
            # because a rejected write reports success either way. Same dance
            # as change_id, for the same reason.
            res, _ = st.write1ByteTxRx(sid, REG_LOCK, 0)
            if res != COMM_OK:
                raise BusError(f'servo {sid}: EEPROM unlock was not acknowledged')
            time.sleep(0.05)
            lock, res, _ = st.read1ByteTxRx(sid, REG_LOCK)
            if res != COMM_OK or lock != 0:
                raise BusError(f'servo {sid}: EEPROM is still locked — zero not set')

            res, _ = st.CorrectPosition(sid, ofs_new)
            if res != COMM_OK:
                raise BusError(f'servo {sid}: the offset write was not acknowledged')
            time.sleep(0.1)

            relock, _ = st.write1ByteTxRx(sid, REG_LOCK, 1)
            after = st.ReadPosition(sid)
            correction = st.ReadCorrection(sid)

        if relock != COMM_OK:
            raise BusError(f'servo {sid}: zero was set but the EEPROM did not re-lock')
        if after is None or after < 0:
            raise BusError(f'servo {sid}: zero was set but the position did not read back')
        # Torque is off and the joint is hand-held, so a few ticks of drift are
        # normal; a large miss means the offset did not take.
        drift = ((after - target + TICKS // 2) % TICKS) - TICKS // 2
        if abs(drift) > 50:
            raise BusError(
                f'servo {sid}: after the write it reads {after}, not ~{target} — '
                f'the offset did not take; nothing has been rebased')

        return {'before': int(before), 'after': int(after), 'target': target,
                'shift': target - int(before),
                'correction': None if correction is None else int(correction)}

    # ---- safety --------------------------------------------------------

    def disarm_all(self) -> dict:
        """Torque off every servo we know about. Best-effort and never raises:
        it runs from atexit and from the browser's unload beacon, where there
        is no one left to report an error to."""
        if self._st is None:
            return {'disarmed': []}
        done = []
        with self._lock:
            for sid in list(self._seen):
                try:
                    if self._st.StopServo(sid) is True:
                        done.append(sid)
                except Exception:
                    pass
        return {'disarmed': done}


BUS = MotorBus()
