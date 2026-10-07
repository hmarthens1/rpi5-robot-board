"""
Hiwonder RRC Lite driver: the 0xAA 0x55 serial protocol at 1 Mbaud.

Needs the Lab 03 firmware (rpi5-ros2-rrclite, Lab 03) for the encoder report,
the command watchdog and hard stops. With the factory firmware the robot still
drives, but encoders/odometry stay empty (has_encoders == False).

Frame (both directions):  AA 55 | function | length | data | CRC-8 (Maxim, poly 0x8C)
Motor IDs on the wire are 0-based; this class takes 1..4 like the board's labels.
"""
import struct
import threading
import time

import serial

FUNC_SYS, FUNC_LED, FUNC_BUZZER, FUNC_MOTOR, FUNC_KEY, FUNC_IMU = 0, 1, 2, 3, 6, 7

# Hiwonder motor profiles (Hiwonder/Portings/motors_param.h): name -> (id, counts per turn)
PROFILES = {"JGB520": (0, 3960), "JGB37": (1, 1980), "JGA27": (2, 1040), "JGB528": (3, 5764)}


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc


def frame(func, data):
    body = bytes([func, len(data)]) + bytes(data)
    return b"\xAA\x55" + body + bytes([crc8(body)])


class MotorState:
    __slots__ = ("count", "rps", "target", "pwm")

    def __init__(self):
        self.count, self.rps, self.target, self.pwm = 0, 0.0, 0.0, 0


class RRCLite:
    """Thread-safe link to the board. Readings are updated by a background reader."""

    def __init__(self, port="/dev/rrclite", profile="JGB520", report_ms=20, watchdog_ms=300):
        if profile not in PROFILES:
            raise ValueError("unknown motor profile %r" % profile)
        self.profile_id, self.counts_per_turn = PROFILES[profile]
        self.report_ms, self.watchdog_ms = report_ms, watchdog_ms
        self._s = serial.Serial()
        self._s.port, self._s.baudrate, self._s.timeout = port, 1000000, 0.05
        self._s.dtr, self._s.rts = True, False      # BOOT0 low, no reset (Lab 02, Part 6.2)
        self._s.open()
        self._wlock = threading.Lock()
        self._lock = threading.Lock()
        self.battery_v = None
        self.battery_stamp = 0.0
        self.imu = None                             # (ax, ay, az [g], gx, gy, gz [deg/s])
        self.imu_stamp = 0.0
        self.motors = [MotorState() for _ in range(4)]
        self.motor_stamp = 0.0
        self.on_motor_report = None                 # callback(stamp, motors) from the reader thread
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.configure()

    # ------------------------------------------------------------------ setup
    def configure(self):
        """Profile first (wrong profile = runaway), then hard stop, report and watchdog on."""
        self._motor([0x05, self.profile_id])
        self._motor([0x03, 0x0F])
        self._motor([0x10, self.report_ms])
        self._motor([0x11] + list(struct.pack("<H", self.watchdog_ms)))

    @property
    def has_encoders(self):
        return time.monotonic() - self.motor_stamp < 1.0

    @property
    def connected(self):
        return time.monotonic() - self.battery_stamp < 3.0

    # ------------------------------------------------------------------ commands
    def _write(self, func, data):
        with self._wlock:
            self._s.write(frame(func, data))

    def _motor(self, data):
        self._write(FUNC_MOTOR, data)

    def set_speeds(self, speeds):
        """speeds: {motor 1..4: rev/s}. Sent together. Re-send at least every
        watchdog_ms, or the board stops the motors by itself."""
        data = [0x01, len(speeds)]
        for m, rps in speeds.items():
            data += struct.pack("<Bf", int(m) - 1, float(rps))
        self._motor(data)

    def stop(self):
        self._motor([0x03, 0x0F])                   # hard stop all four

    def beep(self, freq=1000, on_s=0.1, off_s=0.1, times=1):
        self._write(2, struct.pack("<HHHH", int(freq), int(on_s * 1000), int(off_s * 1000), int(times)))

    def led(self, on_s=0.5, off_s=0.5, times=1, led_id=1):
        self._write(1, struct.pack("<BHHH", led_id, int(on_s * 1000), int(off_s * 1000), int(times)))

    def close(self):
        try:
            self.stop()
            self._motor([0x11, 0, 0])
            self._motor([0x10, 0])
            time.sleep(0.05)
        finally:
            self._running = False
            self._s.close()

    # ------------------------------------------------------------------ reader
    def _read_loop(self):
        buf = bytearray()
        while self._running:
            try:
                buf += self._s.read(4096)
            except (serial.SerialException, OSError):
                time.sleep(0.2)                     # USB dropped: keep trying; firmware watchdog stops motors
                continue
            while True:
                i = buf.find(b"\xAA\x55")
                if i < 0:
                    del buf[:-1]
                    break
                if len(buf) < i + 5:
                    del buf[:i]
                    break
                func, n = buf[i + 2], buf[i + 3]
                end = i + 5 + n
                if len(buf) < end:
                    del buf[:i]
                    break
                body, crc = bytes(buf[i + 2:end - 1]), buf[end - 1]
                del buf[:end]
                if crc8(body) == crc:
                    self._handle(func, body[2:])

    def _handle(self, func, d):
        now = time.monotonic()
        if func == FUNC_SYS and len(d) == 3 and d[0] == 0x04:
            self.battery_v = struct.unpack("<H", d[1:])[0] / 1000.0
            self.battery_stamp = now
        elif func == FUNC_IMU and len(d) == 24:
            self.imu = struct.unpack("<6f", d)
            self.imu_stamp = now
        elif func == FUNC_MOTOR and len(d) == 57 and d[0] == 0x10:
            with self._lock:
                for k in range(4):
                    c, rps, tgt, pwm = struct.unpack_from("<iffh", d, 1 + 14 * k)
                    m = self.motors[k]
                    m.count, m.rps, m.target, m.pwm = c, rps, tgt, pwm
                self.motor_stamp = now
            cb = self.on_motor_report
            if cb:
                cb(now, self.motors)
