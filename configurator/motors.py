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
  * `ChangeId`    returns None on SUCCESS and an error string on failure.
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

DEFAULT_SCAN_HI = 20      # servo IDs above this aren't scanned (ListServos
                          # pings 0..253, which takes seconds on a real bus)
TICKS = 4096
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
        """ChangeId returns None on success, an error STRING on failure."""
        st = self._require()
        if not (0 <= int(new) <= 253):
            raise BusError('new id must be 0..253')
        with self._lock:
            err = st.ChangeId(int(old), int(new))
        if err is not None:
            raise BusError(str(err))
        with self._lock:
            self._seen = [new if i == old else i for i in self._seen]
        return True

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
