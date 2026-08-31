"""Resilient, threaded serial transport with command acknowledgements."""
from __future__ import annotations

import glob
import logging
import secrets
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

import serial

from .protocol import ParsedLine, format_command, parse_line
from .types import MotionResult, Telemetry

LOG = logging.getLogger(__name__)


class SerialLink:
    def __init__(self, config: dict[str, Any], dry_run: bool = False) -> None:
        self.config = config
        self.dry_run = dry_run
        self.port_setting = str(config.get("port", "auto"))
        self.baud = int(config.get("baud", 115200))
        self.reconnect_s = float(config.get("reconnect_s", 1.0))
        self.ping_period_s = float(config.get("ping_period_s", 0.5))
        self.stale_link_s = float(config.get("stale_link_s", 2.5))
        self.session_id = secrets.token_hex(4)

        self._serial: serial.Serial | None = None
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._lock = threading.RLock()
        self._write_lock = threading.Lock()

        self._connected = False
        self._last_rx = 0.0
        self._last_ping = 0.0
        self._boot_id: int | None = None
        self._boot_changed = False
        self._armed_event = False
        self._telemetry = Telemetry()
        self._acks: set[int] = set()
        self._done: dict[int, MotionResult] = {}
        self._messages: deque[ParsedLine] = deque(maxlen=500)

    @property
    def connected(self) -> bool:
        if self.dry_run:
            return True
        with self._lock:
            return self._connected and (time.time() - self._last_rx) <= self.stale_link_s

    @property
    def boot_id(self) -> int | None:
        with self._lock:
            return self._boot_id

    def start(self) -> None:
        if self.dry_run:
            self._connected = True
            return
        if self._thread and self._thread.is_alive():
            return
        self._running.set()
        self._thread = threading.Thread(target=self._reader_loop, name="robot-serial", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        try:
            self.send("STOP")
        except Exception:
            pass
        if self._thread:
            self._thread.join(timeout=1.5)
        self._close()

    def _resolve_port(self) -> str:
        if self.port_setting != "auto":
            return self.port_setting
        candidates = sorted(glob.glob("/dev/serial/by-id/*"))
        if candidates:
            return candidates[0]
        for fallback in ("/dev/ttyACM0", "/dev/ttyUSB0"):
            if Path(fallback).exists():
                return fallback
        raise FileNotFoundError("No Arduino serial device found")

    def _open(self) -> None:
        port = self._resolve_port()
        LOG.info("Opening Arduino serial port %s at %d", port, self.baud)
        self._serial = serial.Serial(
            port,
            baudrate=self.baud,
            timeout=0.05,
            write_timeout=0.20,
            dsrdtr=False,
            rtscts=False,
        )
        time.sleep(0.35)
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        with self._lock:
            self._connected = True
            self._last_rx = time.time()
        self.send("HELLO", session=self.session_id)

    def _close(self) -> None:
        serial_obj = self._serial
        self._serial = None
        with self._lock:
            self._connected = False
        if serial_obj:
            try:
                serial_obj.close()
            except Exception:
                pass

    def _reader_loop(self) -> None:
        buffer = b""
        while self._running.is_set():
            if self._serial is None:
                try:
                    self._open()
                    buffer = b""
                except Exception as exc:
                    LOG.warning("Serial connection unavailable: %s", exc)
                    self._close()
                    time.sleep(self.reconnect_s)
                    continue

            try:
                chunk = self._serial.read(512) if self._serial else b""
                if not chunk:
                    time.sleep(0.002)
                    continue
                buffer += chunk
                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    text = raw.decode("utf-8", "ignore").strip()
                    if text:
                        self._handle_line(text)
            except Exception as exc:
                LOG.warning("Serial read failed: %s", exc)
                self._close()
                time.sleep(self.reconnect_s)

    def _handle_line(self, text: str) -> None:
        parsed = parse_line(text)
        now = time.time()
        with self._lock:
            self._last_rx = now
            self._connected = True
            self._messages.append(parsed)

            if parsed.kind == "TLM":
                telemetry: Telemetry = parsed.values["telemetry"]
                self._telemetry = telemetry
                if telemetry.boot_id:
                    self._update_boot_id(telemetry.boot_id)
                if telemetry.armed:
                    self._armed_event = True
            elif parsed.kind in {"READY", "HELLO_ACK", "PONG", "ARMED"}:
                boot = parsed.values.get("boot")
                if isinstance(boot, int) and boot:
                    self._update_boot_id(boot)
                if parsed.kind == "ARMED":
                    self._armed_event = True
            elif parsed.kind == "ACK":
                command_id = parsed.values.get("id")
                if isinstance(command_id, int):
                    self._acks.add(command_id)
            elif parsed.kind == "DONE":
                result: MotionResult = parsed.values["result"]
                self._done[result.command_id] = result

        LOG.debug("ARDUINO << %s", text)

    def _update_boot_id(self, new_boot_id: int) -> None:
        if self._boot_id is None:
            self._boot_id = new_boot_id
        elif new_boot_id != self._boot_id:
            LOG.warning("Arduino boot ID changed: %s -> %s", self._boot_id, new_boot_id)
            self._boot_id = new_boot_id
            self._boot_changed = True
            self._acks.clear()
            self._done.clear()

    def send(self, name: str, **values: Any) -> bool:
        payload = format_command(name, **values)
        if self.dry_run:
            LOG.info("DRY-RUN >> %s", payload.decode().strip())
            return True
        serial_obj = self._serial
        if serial_obj is None:
            return False
        try:
            with self._write_lock:
                serial_obj.write(payload)
                serial_obj.flush()
            LOG.debug("JETSON >> %s", payload.decode().strip())
            return True
        except Exception as exc:
            LOG.warning("Serial write failed: %s", exc)
            self._close()
            return False

    def tick(self) -> None:
        now = time.time()
        if now - self._last_ping >= self.ping_period_s:
            self.send("PING", session=self.session_id)
            self._last_ping = now

    def telemetry(self) -> Telemetry:
        with self._lock:
            return Telemetry(**{field: getattr(self._telemetry, field) for field in self._telemetry.__dataclass_fields__})

    def consume_ack(self, command_id: int) -> bool:
        with self._lock:
            if command_id in self._acks:
                self._acks.remove(command_id)
                return True
            return False

    def has_ack(self, command_id: int) -> bool:
        with self._lock:
            return command_id in self._acks

    def consume_done(self, command_id: int) -> MotionResult | None:
        with self._lock:
            return self._done.pop(command_id, None)

    def consume_boot_change(self) -> bool:
        with self._lock:
            changed = self._boot_changed
            self._boot_changed = False
            return changed

    def consume_armed(self) -> bool:
        with self._lock:
            armed = self._armed_event
            self._armed_event = False
            return armed

    def inject_telemetry(self, telemetry: Telemetry) -> None:
        """Testing helper used only in dry-run scripts."""
        with self._lock:
            self._telemetry = telemetry
            self._last_rx = time.time()
            self._connected = True

    def inject_done(self, result: MotionResult) -> None:
        with self._lock:
            self._done[result.command_id] = result
