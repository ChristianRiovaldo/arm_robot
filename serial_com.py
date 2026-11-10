import time
import servo_config as cfg

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    serial = None
    SERIAL_AVAILABLE = False

class SerialCom:
    BOOT_WAIT_S = 2.0
    MIN_INTERVAL_S = 0.05
    STABLE_FRAMES = 3
    HALT_AFTER_LOST_FRAMES = 1

    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.ready_at = 0.0
        self.error = ""
        self.last_reply = ""
        self._rx = b""
        self._last_send_t = 0.0
        self.servo_angles = [None] * 6
        self.reset_cache()

    @property
    def is_open(self):
        return self.ser is not None

    @property
    def ready(self):
        return self.ser is not None and time.time() >= self.ready_at

    def open(self):
        if not SERIAL_AVAILABLE:
            self.error = "pyserial belum terinstall (pip install pyserial)"
            return False
        try:
            self.ser = serial.serial_for_url(self.port, baudrate=self.baud, timeout=0, write_timeout=0.2)
        except (serial.SerialException, OSError, ValueError) as e:
            self.ser = None
            self.error = str(e)
            return False

        self.error = ""
        self.last_reply = ""
        self._rx = b""
        self.servo_angles = [None] * 6
        self.ready_at = time.time() + self.BOOT_WAIT_S
        self.reset_cache()
        return True

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
        self.ser = None
        self.servo_angles = [None] * 6

    def _fail(self, e):
        self.error = f"serial error: {e}"
        self.close()

    def reset_cache(self):
        self.last_all = None
        self.last_single = {}
        self._pending = None
        self._pending_count = 0
        self._moving_servo = None
        self._lost_count = 0

    def _write(self, text):
        try:
            self.ser.write((text + "\n").encode("ascii"))
            return True
        except (serial.SerialException, OSError) as e:
            self._fail(e)
            return False

    def poll(self):
        if self.ser is None:
            return
        try:
            n = self.ser.in_waiting
            if n:
                self._rx += self.ser.read(n)
        except (serial.SerialException, OSError) as e:
            self._fail(e)
            return
        *lines, self._rx = self._rx.split(b"\n")
        for ln in lines:
            ln = ln.decode("ascii", errors="ignore").strip()
            if ln.startswith("P "):
                parts = ln.split()[1:]
                if len(parts) == 6:
                    try:
                        vals = [int(p) for p in parts]
                    except ValueError:
                        continue
                    self.servo_angles = [None if v < 0 else v for v in vals]
            elif ln:
                self.last_reply = ln
        self._rx = self._rx[-200:]

    def send_all(self, angles):
        if not self.ready or angles is None or len(angles) < 6:
            return False
        vals = [cfg.clamp_servo(i, angles[i]) for i in range(6)]
        now = time.time()
        if vals == self.last_all or now - self._last_send_t < self.MIN_INTERVAL_S:
            return False
        if self._write("A " + " ".join(str(v) for v in vals)):
            self.last_all = vals
            self._last_send_t = now
            return True
        return False

    def send_single(self, servo_no, angle):
        if not self.ready or not 1 <= servo_no <= 6:
            return False
        val = cfg.clamp_servo(servo_no - 1, angle)
        now = time.time()
        if self.last_single.get(servo_no) == val or now - self._last_send_t < self.MIN_INTERVAL_S:
            return False
        if self._write(f"S {servo_no} {val}"):
            self.last_single[servo_no] = val
            self._last_send_t = now
            return True
        return False

    def halt(self, servo_no=None):
        if not self.ready:
            return False
        if servo_no is None:
            ok = self._write("H")
            self.last_single.clear()
            self.last_all = None
            self._moving_servo = None
        else:
            ok = self._write(f"H {servo_no}")
            self.last_single.pop(servo_no, None)
            if self._moving_servo == servo_no:
                self._moving_servo = None
        self._pending = None
        self._pending_count = 0
        return ok

    def update_manual(self, cmd):
        if not self.ready:
            return

        valid = bool(cmd.get("active")) and bool(cmd.get("servo_no")) and cmd.get("angle") is not None
        if not valid:
            self._pending = None
            self._pending_count = 0
            self._lost_count += 1
            if self._lost_count >= self.HALT_AFTER_LOST_FRAMES and self._moving_servo is not None:
                self.halt()
            return

        self._lost_count = 0
        servo_no = int(cmd["servo_no"])
        key = (servo_no, int(cmd["angle"]))

        if self._moving_servo is not None and self._moving_servo != servo_no:
            self.halt(self._moving_servo)

        if key == self._pending:
            self._pending_count += 1
        else:
            self._pending = key
            self._pending_count = 1

        if self._pending_count >= self.STABLE_FRAMES:
            self.send_single(*key)
            if servo_no in self.last_single:
                self._moving_servo = servo_no

    def update_pointing(self, pt):
        if self.ready and pt.get("ik_success") and pt.get("servo"):
            self.send_all(pt["servo"])

    def status(self):
        if not SERIAL_AVAILABLE:
            return "pyserial belum terinstall", "error"
        if self.error:
            return self.error, "error"
        if self.ser is None:
            return f"OFF ({self.port})", "off"
        if not self.ready:
            return f"{self.port}: menunggu Arduino siap...", "warn"
        text = f"{self.port} @ {self.baud}"
        if self.last_reply:
            text += f"\n{self.last_reply}"
        return text, "ok"