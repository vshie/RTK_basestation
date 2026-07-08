#!/usr/bin/env python3
"""BlueOS RTK Base Station extension.

Reads RTCM3 corrections from a u-blox ZED-F9P (configured as a stationary base)
over a USB serial port and pushes them to an NTRIP caster (e.g. RTK2Go) as an
NTRIP *server/source*. This is the opposite direction to a rover NTRIP client:
here we are the source of corrections, not the consumer.
"""

import argparse
import asyncio
import base64
import glob
import json
import logging.handlers
import math
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

import serial
from litestar import Litestar, get, post
from litestar.controller import Controller
from litestar.datastructures import State
from litestar.logging import LoggingConfig
from litestar.static_files.config import StaticFilesConfig
from pydantic import BaseModel

# Defaults discovered on the target BlueOS device (u-blox ZED-F9P via USB).
DEFAULT_SERIAL_DEVICE = (
    "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"
)

_global_config = {
    "config_file": "config/rtk_config.json",
}


# --------------------------------------------------------------------------- #
# Models
# --------------------------------------------------------------------------- #
class RTKConfig(BaseModel):
    caster_host: str = ""
    caster_port: int = 2101  # standard NTRIP port; generic default, not a credential
    mountpoint: str = ""
    # RTK2Go / SNIP calls this the "mount point password". Username is normally
    # unused for NTRIP v1 source pushes, but is kept for NTRIP v2 auth.
    username: str = ""
    password: str = ""
    ntrip_version: str = "v1"  # "v1" (SOURCE / ICY 200 OK) or "v2" (HTTP POST)
    serial_device: str = DEFAULT_SERIAL_DEVICE
    serial_baud: int = 115200
    enabled: bool = False


class RTKStatus(BaseModel):
    streaming: bool = False
    serial_connected: bool = False
    caster_connected: bool = False
    caster_response: Optional[str] = None
    last_error: Optional[str] = None
    last_update: Optional[str] = None
    bytes_pushed: int = 0
    rtcm_messages_pushed: int = 0
    connected_since: Optional[str] = None
    message_counts: Dict[str, int] = {}
    base_position: Optional[Dict[str, float]] = None  # ARP from RTCM 1005/1006 {lat, lon, height, x, y, z}
    gps_position: Optional[Dict[str, float]] = None  # live NMEA GGA {lat, lon, alt}
    satellites: Optional[int] = None  # satellites used in the solution (from NMEA GGA)
    hdop: Optional[float] = None  # horizontal dilution of precision (from NMEA GGA)
    fix_quality: Optional[int] = None  # NMEA GGA quality (0=none, 1=GPS, 2=DGPS, ...)
    survey_in: Optional[Dict[str, Any]] = None  # {active, valid, duration_s, mean_acc_m, observations}
    survey_message: Optional[str] = None
    tmode: Optional[str] = None  # disabled | survey-in | fixed (best-effort from last command)


class SurveyInRequest(BaseModel):
    serial_device: str = DEFAULT_SERIAL_DEVICE
    serial_baud: int = 115200
    min_duration: int = 60  # seconds
    accuracy: float = 2.0   # metres


class FixedPositionRequest(BaseModel):
    serial_device: str = DEFAULT_SERIAL_DEVICE
    serial_baud: int = 115200
    # If omitted, the extension uses the current base_position (RTCM 1005) or live GGA.
    lat: Optional[float] = None
    lon: Optional[float] = None
    height: Optional[float] = None  # metres (ellipsoidal / ARP height)
    accuracy: float = 0.01  # metres — reported fixed-position accuracy


# --------------------------------------------------------------------------- #
# Configuration persistence
# --------------------------------------------------------------------------- #
class ConfigManager:
    def __init__(self, config_file: str = "config/rtk_config.json"):
        self.config_file = Path(config_file)
        self.config_file.parent.mkdir(parents=True, exist_ok=True)

    def load_config(self) -> RTKConfig:
        try:
            if self.config_file.exists():
                with open(self.config_file, "r") as f:
                    return RTKConfig(**json.load(f))
        except Exception as e:  # noqa: BLE001
            print(f"Warning: could not load config from {self.config_file}: {e}")
        return RTKConfig()

    def save_config(self, config: RTKConfig) -> bool:
        try:
            with open(self.config_file, "w") as f:
                json.dump(config.model_dump(), f, indent=2)
            return True
        except Exception as e:  # noqa: BLE001
            print(f"Warning: could not save config to {self.config_file}: {e}")
            return False


# --------------------------------------------------------------------------- #
# RTCM parsing helpers
# --------------------------------------------------------------------------- #
class RTCMParser:
    """Extract complete RTCM 3.x frames from a byte stream.

    The F9P also emits NMEA on the same port; only 0xD3-framed RTCM messages are
    returned so we forward a clean correction stream to the caster.
    """

    def __init__(self) -> None:
        self.buffer = bytearray()

    def add_data(self, data: bytes) -> List[bytes]:
        self.buffer.extend(data)
        messages: List[bytes] = []

        while len(self.buffer) >= 6:
            preamble_idx = self.buffer.find(0xD3)
            if preamble_idx == -1:
                # Keep only a small tail in case a preamble is split across reads.
                if len(self.buffer) > 1024:
                    self.buffer = self.buffer[-8:]
                break
            if preamble_idx > 0:
                del self.buffer[:preamble_idx]
            if len(self.buffer) < 3:
                break

            # 6 reserved bits must be zero, followed by a 10-bit length.
            if (self.buffer[1] & 0xFC) != 0:
                del self.buffer[0]
                continue
            length = ((self.buffer[1] & 0x03) << 8) | self.buffer[2]
            total_size = 3 + length + 3  # header + payload + CRC24

            if len(self.buffer) < total_size:
                break

            message = bytes(self.buffer[:total_size])
            if crc24q(message[:-3]) == int.from_bytes(message[-3:], "big"):
                messages.append(message)
                del self.buffer[:total_size]
            else:
                # Bad CRC: drop the preamble byte and resync.
                del self.buffer[0]

        return messages


# CRC-24Q table (Qualcomm), used by RTCM3.
_CRC24Q_POLY = 0x1864CFB


def crc24q(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= _CRC24Q_POLY
    return crc & 0xFFFFFF


def rtcm_message_type(message: bytes) -> int:
    """Return the 12-bit RTCM message number."""
    return (message[3] << 4) | (message[4] >> 4)


def _read_bits(data: bytes, start: int, length: int, signed: bool = False) -> int:
    value = 0
    for i in range(length):
        bit_index = start + i
        bit = (data[bit_index >> 3] >> (7 - (bit_index & 7))) & 1
        value = (value << 1) | bit
    if signed and (value & (1 << (length - 1))):
        value -= 1 << length
    return value


def decode_1005(message: bytes) -> Optional[Dict[str, float]]:
    """Decode an RTCM 1005/1006 stationary reference-station ARP message."""
    try:
        payload = message[3:-3]
        bits = 0
        msg_num = _read_bits(payload, bits, 12); bits += 12
        if msg_num not in (1005, 1006):
            return None
        bits += 12  # station id
        bits += 6   # ITRF year
        bits += 4   # GPS/GLONASS/Galileo/reference-station indicators
        x = _read_bits(payload, bits, 38, signed=True) * 0.0001; bits += 38
        bits += 2   # single receiver + reserved
        y = _read_bits(payload, bits, 38, signed=True) * 0.0001; bits += 38
        bits += 2   # quarter cycle indicator
        z = _read_bits(payload, bits, 38, signed=True) * 0.0001; bits += 38
        lat, lon, height = ecef_to_llh(x, y, z)
        return {"lat": lat, "lon": lon, "height": height, "x": x, "y": y, "z": z}
    except Exception:  # noqa: BLE001
        return None


def ecef_to_llh(x: float, y: float, z: float) -> tuple:
    """Convert WGS84 ECEF (metres) to geodetic lat/lon (deg) and height (m)."""
    a = 6378137.0
    f = 1 / 298.257223563
    b = a * (1 - f)
    e2 = f * (2 - f)
    ep2 = (a**2 - b**2) / b**2
    p = math.sqrt(x**2 + y**2)
    if p == 0:
        return 0.0, 0.0, 0.0
    theta = math.atan2(z * a, p * b)
    lon = math.atan2(y, x)
    lat = math.atan2(
        z + ep2 * b * math.sin(theta) ** 3,
        p - e2 * a * math.cos(theta) ** 3,
    )
    n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    height = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), height


# --------------------------------------------------------------------------- #
# UBX helpers (u-blox configuration + monitoring)
# --------------------------------------------------------------------------- #
# Configuration-item key IDs (verified against the ZED-F9P interface description).
CFG_TMODE_MODE = 0x20030001          # E1: 0=disabled, 1=survey-in, 2=fixed
CFG_TMODE_POS_TYPE = 0x20030002      # E1: 0=ECEF, 1=LLH
CFG_TMODE_LAT = 0x40030009           # I4: 1e-7 deg
CFG_TMODE_LON = 0x4003000A           # I4: 1e-7 deg
CFG_TMODE_HEIGHT = 0x4003000B        # I4: cm
CFG_TMODE_LAT_HP = 0x2003002B        # I1: 1e-9 deg
CFG_TMODE_LON_HP = 0x2003002C        # I1: 1e-9 deg
CFG_TMODE_HEIGHT_HP = 0x2003002D     # I1: 0.1 mm
CFG_TMODE_FIXED_POS_ACC = 0x4003000F # U4: 0.1 mm
CFG_TMODE_SVIN_MIN_DUR = 0x40030010  # U4: seconds
CFG_TMODE_SVIN_ACC_LIMIT = 0x40030011  # U4: 0.1 mm units
CFG_MSGOUT_UBX_NAV_SVIN_USB = 0x2091008B  # U1: output rate on USB


def ubx_frame(cls_id: int, msg_id: int, payload: bytes) -> bytes:
    length = len(payload)
    body = bytes([cls_id, msg_id, length & 0xFF, (length >> 8) & 0xFF]) + payload
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b"\xB5\x62" + body + bytes([ck_a, ck_b])


def ubx_valset(items, layers: int = 0x07) -> bytes:
    """Build a UBX-CFG-VALSET. items = list of (key, value, size_bytes).

    layers bitmask: RAM=0x01, BBR=0x02, Flash=0x04 (0x07 = all/persistent).
    Values may be signed; encode with two's complement via signed=True.
    """
    payload = bytes([0x00, layers & 0xFF, 0x00, 0x00])
    for key, value, size in items:
        payload += key.to_bytes(4, "little")
        payload += int(value).to_bytes(size, "little", signed=True)
    return ubx_frame(0x06, 0x8A, payload)


def build_survey_in_valset(min_duration: int, accuracy_m: float, layers: int = 0x07) -> bytes:
    """Configure survey-in mode and enable NAV-SVIN status output on USB."""
    acc_limit = max(1, int(round(accuracy_m * 10000)))  # metres -> 0.1 mm
    items = [
        (CFG_TMODE_MODE, 1, 1),
        (CFG_TMODE_SVIN_MIN_DUR, max(1, int(min_duration)), 4),
        (CFG_TMODE_SVIN_ACC_LIMIT, acc_limit, 4),
        (CFG_MSGOUT_UBX_NAV_SVIN_USB, 1, 1),
    ]
    return ubx_valset(items, layers)


def _llh_to_tmode_fields(lat: float, lon: float, height_m: float):
    """Encode WGS84 LLH into u-blox TMODE LAT/LON/HEIGHT (+ HP) fields."""
    lat_scaled = lat * 1e7
    lon_scaled = lon * 1e7
    lat_i = int(lat_scaled)
    lon_i = int(lon_scaled)
    lat_hp = int(round((lat_scaled - lat_i) * 100))  # residual as 1e-9 deg
    lon_hp = int(round((lon_scaled - lon_i) * 100))
    # Clamp HP to signed I1 range
    lat_hp = max(-128, min(127, lat_hp))
    lon_hp = max(-128, min(127, lon_hp))

    height_cm = int(height_m * 100)
    height_hp = int(round((height_m * 100 - height_cm) * 10))  # 0.1 mm
    height_hp = max(-128, min(127, height_hp))
    return lat_i, lat_hp, lon_i, lon_hp, height_cm, height_hp


def build_fixed_position_valset(
    lat: float, lon: float, height_m: float, accuracy_m: float = 0.01, layers: int = 0x07
) -> bytes:
    """Configure fixed (stationary) timing mode at a known LLH position."""
    lat_i, lat_hp, lon_i, lon_hp, height_cm, height_hp = _llh_to_tmode_fields(lat, lon, height_m)
    acc = max(1, int(round(accuracy_m * 10000)))  # metres -> 0.1 mm
    items = [
        (CFG_TMODE_MODE, 2, 1),          # FIXED
        (CFG_TMODE_POS_TYPE, 1, 1),      # LLH
        (CFG_TMODE_LAT, lat_i, 4),
        (CFG_TMODE_LON, lon_i, 4),
        (CFG_TMODE_HEIGHT, height_cm, 4),
        (CFG_TMODE_LAT_HP, lat_hp, 1),
        (CFG_TMODE_LON_HP, lon_hp, 1),
        (CFG_TMODE_HEIGHT_HP, height_hp, 1),
        (CFG_TMODE_FIXED_POS_ACC, acc, 4),
        (CFG_MSGOUT_UBX_NAV_SVIN_USB, 1, 1),
    ]
    return ubx_valset(items, layers)


class UBXScanner:
    """Extract complete, checksum-valid UBX frames from a byte stream."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def add_data(self, data: bytes):
        self.buffer.extend(data)
        out = []
        while True:
            idx = self.buffer.find(b"\xB5\x62")
            if idx == -1:
                if len(self.buffer) > 4096:
                    self.buffer = self.buffer[-2:]
                break
            if idx > 0:
                del self.buffer[:idx]
            if len(self.buffer) < 6:
                break
            length = self.buffer[4] | (self.buffer[5] << 8)
            total = 6 + length + 2
            if len(self.buffer) < total:
                break
            frame = bytes(self.buffer[:total])
            ck_a = ck_b = 0
            for b in frame[2 : total - 2]:
                ck_a = (ck_a + b) & 0xFF
                ck_b = (ck_b + ck_a) & 0xFF
            if ck_a == frame[total - 2] and ck_b == frame[total - 1]:
                out.append((frame[2], frame[3], frame[6 : total - 2]))
                del self.buffer[:total]
            else:
                del self.buffer[0]
        return out


class NMEAScanner:
    """Extract NMEA sentences and pull sats / HDOP from GGA (and HDOP from GSA)."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def add_data(self, data: bytes) -> List[Dict[str, Any]]:
        self.buffer.extend(data)
        updates: List[Dict[str, Any]] = []
        while True:
            start = self.buffer.find(b"$")
            if start == -1:
                if len(self.buffer) > 4096:
                    self.buffer.clear()
                break
            if start > 0:
                del self.buffer[:start]
            end = self.buffer.find(b"\n")
            if end == -1:
                if len(self.buffer) > 512:
                    # Incomplete / garbage line — drop the leading '$' and resync.
                    del self.buffer[0]
                break
            line = bytes(self.buffer[: end + 1])
            del self.buffer[: end + 1]
            parsed = _parse_nmea_line(line)
            if parsed:
                updates.append(parsed)
        return updates


def _nmea_checksum_ok(sentence: bytes) -> bool:
    """Validate NMEA XOR checksum for a full sentence including $...*CS\\r\\n."""
    try:
        text = sentence.decode("ascii", errors="ignore").strip()
        if not text.startswith("$") or "*" not in text:
            return False
        body, cs = text[1:].rsplit("*", 1)
        if len(cs) < 2:
            return False
        expected = int(cs[:2], 16)
        actual = 0
        for ch in body:
            actual ^= ord(ch)
        return actual == expected
    except Exception:  # noqa: BLE001
        return False


def _nmea_dm_to_deg(dm: str, hemi: str) -> Optional[float]:
    """Convert NMEA ddmm.mmmm / dddmm.mmmm + hemisphere to decimal degrees."""
    if not dm or not hemi:
        return None
    try:
        val = float(dm)
    except ValueError:
        return None
    deg = int(val // 100)
    minutes = val - deg * 100
    decimal = deg + minutes / 60.0
    if hemi in ("S", "W"):
        decimal = -decimal
    return decimal


def _parse_nmea_line(line: bytes) -> Optional[Dict[str, Any]]:
    """Parse GGA (position, sats, HDOP, quality) or GSA (HDOP) into a status update."""
    if not _nmea_checksum_ok(line):
        return None
    text = line.decode("ascii", errors="ignore").strip()
    body = text[1:].split("*", 1)[0]
    fields = body.split(",")
    if not fields:
        return None
    talker = fields[0]  # e.g. GNGGA / GPGGA / GNGSA
    if talker.endswith("GGA") and len(fields) >= 10:
        out: Dict[str, Any] = {}
        try:
            if fields[6]:
                out["fix_quality"] = int(fields[6])
            if fields[7]:
                out["satellites"] = int(fields[7])
            if fields[8]:
                out["hdop"] = float(fields[8])
            lat = _nmea_dm_to_deg(fields[2], fields[3]) if len(fields) > 3 else None
            lon = _nmea_dm_to_deg(fields[4], fields[5]) if len(fields) > 5 else None
            alt = float(fields[9]) if fields[9] else None
            if lat is not None and lon is not None:
                out["gps_position"] = {
                    "lat": lat,
                    "lon": lon,
                    "alt": alt if alt is not None else 0.0,
                }
        except ValueError:
            return None
        return out or None
    if talker.endswith("GSA") and len(fields) >= 17:
        # GSA: HDOP is field 16 (0-based index 15) after the satellite PRN list.
        try:
            if fields[16]:
                return {"hdop": float(fields[16])}
        except (ValueError, IndexError):
            return None
    return None


def find_serial_device(preferred: str) -> Optional[str]:
    """Return the preferred serial device if present, else auto-detect a u-blox."""
    if preferred and os.path.exists(preferred):
        return preferred
    for pattern in ("/dev/serial/by-id/*u-blox*", "/dev/serial/by-id/*u_blox*"):
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    acm = sorted(glob.glob("/dev/ttyACM*"))
    if acm:
        return acm[0]
    return None


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


# --------------------------------------------------------------------------- #
# Controller
# --------------------------------------------------------------------------- #
class RTKController(Controller):
    def __init__(self, owner: "Litestar") -> None:
        super().__init__(owner)
        self._config_manager = ConfigManager(_global_config["config_file"])
        self._config = self._config_manager.load_config()
        self._status = RTKStatus()
        self._task: Optional[asyncio.Task] = None
        self._serial: Optional[serial.Serial] = None  # active handle while streaming
        self._ubx = UBXScanner()
        self._nmea = NMEAScanner()

    # ---- validation -------------------------------------------------------- #
    def _validate(self, cfg: RTKConfig) -> List[str]:
        errors: List[str] = []
        if not cfg.caster_host.strip():
            errors.append("Caster host is required")
        if not (0 < cfg.caster_port < 65536):
            errors.append("Caster port must be between 1 and 65535")
        if not cfg.mountpoint.strip():
            errors.append("Mountpoint is required")
        if not cfg.password.strip():
            errors.append("Mountpoint password is required")
        if cfg.ntrip_version not in ("v1", "v2"):
            errors.append("NTRIP version must be 'v1' or 'v2'")
        if not cfg.serial_device.strip():
            errors.append("Serial device is required")
        return errors

    # ---- lifecycle --------------------------------------------------------- #
    async def auto_start_if_enabled(self) -> None:
        if self._config.enabled and not self._validate(self._config):
            print("Auto-starting NTRIP base station stream from saved settings...")
            self._start_stream()
        elif self._config.enabled:
            self._status.last_error = "Invalid configuration - check settings"

    def _start_stream(self) -> None:
        if self._task and not self._task.done():
            self._task.cancel()
        self._task = asyncio.create_task(self._stream_loop())

    def _stop_stream(self) -> None:
        if self._task and not self._task.done():
            self._task.cancel()
        self._task = None
        self._status.streaming = False
        self._status.caster_connected = False
        self._status.serial_connected = False

    # ---- HTTP API ---------------------------------------------------------- #
    @get("/config", sync_to_thread=False)
    def get_config(self) -> Dict[str, Any]:
        return self._config.model_dump()

    @post("/config", sync_to_thread=False)
    def set_config(self, data: RTKConfig) -> Dict[str, Any]:
        errors = self._validate(data)
        self._config = data
        self._config_manager.save_config(data)

        if data.enabled:
            if errors:
                self._stop_stream()
                self._status.last_error = "Invalid configuration: " + "; ".join(errors)
                return {"status": "error", "errors": errors, "config": data.model_dump()}
            self._start_stream()
        else:
            self._stop_stream()
            self._status.last_error = None

        return {"status": "success", "config": data.model_dump()}

    @get("/status", sync_to_thread=False)
    def get_status(self) -> Dict[str, Any]:
        return self._status.model_dump()

    def _apply_ubx(self, cls_id: int, msg_id: int, payload: bytes) -> None:
        """Update status from monitored UBX messages (NAV-SVIN, ACK/NAK)."""
        if cls_id == 0x01 and msg_id == 0x3B and len(payload) >= 40:  # UBX-NAV-SVIN
            self._status.survey_in = {
                "active": bool(payload[37]),
                "valid": bool(payload[36]),
                "duration_s": int.from_bytes(payload[8:12], "little"),
                "mean_acc_m": round(int.from_bytes(payload[28:32], "little") / 10000.0, 4),
                "observations": int.from_bytes(payload[32:36], "little"),
            }
        elif cls_id == 0x05 and msg_id in (0x00, 0x01) and len(payload) >= 2:  # ACK/NAK
            if payload[0] == 0x06 and payload[1] == 0x8A:  # to a CFG-VALSET
                self._status.survey_message = (
                    "Survey-in command acknowledged"
                    if msg_id == 0x01
                    else "Survey-in command rejected (NAK)"
                )

    def _apply_nmea(self, update: Dict[str, Any]) -> None:
        """Update sats / HDOP / fix quality / live position from a parsed NMEA sentence."""
        if "satellites" in update:
            self._status.satellites = update["satellites"]
        if "hdop" in update:
            self._status.hdop = update["hdop"]
        if "fix_quality" in update:
            self._status.fix_quality = update["fix_quality"]
        if "gps_position" in update:
            self._status.gps_position = update["gps_position"]

    async def _send_ubx_command(self, cmd: bytes, device: str, baud: int) -> Dict[str, Any]:
        """Write a UBX command on the live stream handle or a temporary serial open."""
        if self._serial is not None and getattr(self._serial, "is_open", False):
            try:
                await asyncio.to_thread(self._serial.write, cmd)
            except Exception as e:  # noqa: BLE001
                return {"success": False, "message": f"Serial write failed: {e}"}
            return {"success": True, "acked": None, "device": device}

        def _run():
            try:
                with serial.Serial(device, baud, timeout=1) as ser:
                    ser.write(cmd)
                    ser.flush()
                    scanner = UBXScanner()
                    acked: Optional[bool] = None
                    end = time.time() + 4
                    while time.time() < end:
                        chunk = ser.read(4096)
                        if not chunk:
                            continue
                        for c, i, pl in scanner.add_data(chunk):
                            self._apply_ubx(c, i, pl)
                            if c == 0x05 and i in (0x00, 0x01) and len(pl) >= 2 \
                                    and pl[0] == 0x06 and pl[1] == 0x8A:
                                acked = i == 0x01
                        if acked is not None:
                            break
                    return acked
            except Exception as e:  # noqa: BLE001
                return e

        res = await asyncio.to_thread(_run)
        if isinstance(res, Exception):
            return {"success": False, "message": f"Serial error: {res}", "device": device}
        return {"success": res is not False, "acked": res, "device": device}

    @post("/survey_in")
    async def survey_in(self, data: SurveyInRequest) -> Dict[str, Any]:
        """Command the ZED-F9P to (re)start a survey-in over the serial port."""
        device = find_serial_device(data.serial_device)
        if not device:
            return {"success": False, "message": f"Serial device not found: {data.serial_device}"}

        cmd = build_survey_in_valset(data.min_duration, data.accuracy)
        self._status.survey_message = None
        self._status.survey_in = None
        result = await self._send_ubx_command(cmd, device, data.serial_baud)
        if not result.get("success"):
            return result

        self._status.tmode = "survey-in"
        acked = result.get("acked")
        if acked is True:
            msg = f"Survey-in started (min {data.min_duration}s, {data.accuracy} m) and acknowledged."
        elif acked is False:
            msg = "Receiver rejected the survey-in command (NAK)."
            return {"success": False, "message": msg, "device": device}
        else:
            msg = (
                f"Survey-in started (min {data.min_duration}s, {data.accuracy} m accuracy). "
                "Progress will appear in status while streaming."
            )
        return {
            "success": True,
            "message": msg,
            "device": device,
            "survey_in": self._status.survey_in,
        }

    @post("/fixed_position")
    async def fixed_position(self, data: FixedPositionRequest) -> Dict[str, Any]:
        """Lock the ZED-F9P into fixed (stationary) timing mode at a known position.

        Prefer an explicit lat/lon/height. If omitted, use the surveyed ARP from
        RTCM 1005, else the latest live GGA position.
        """
        device = find_serial_device(data.serial_device)
        if not device:
            return {"success": False, "message": f"Serial device not found: {data.serial_device}"}

        lat, lon, height = data.lat, data.lon, data.height
        source = "provided"
        if lat is None or lon is None or height is None:
            if self._status.base_position:
                lat = self._status.base_position["lat"]
                lon = self._status.base_position["lon"]
                height = self._status.base_position["height"]
                source = "RTCM 1005 ARP"
            elif self._status.gps_position:
                lat = self._status.gps_position["lat"]
                lon = self._status.gps_position["lon"]
                height = self._status.gps_position["alt"]
                source = "live GGA"
            else:
                return {
                    "success": False,
                    "message": (
                        "No position available. Enter lat/lon/height, wait for a base "
                        "position (RTCM 1005), or wait for a live GPS fix."
                    ),
                }

        cmd = build_fixed_position_valset(lat, lon, height, data.accuracy)
        result = await self._send_ubx_command(cmd, device, data.serial_baud)
        if not result.get("success"):
            return result

        self._status.tmode = "fixed"
        self._status.survey_message = (
            f"Fixed position locked from {source}: {lat:.7f}, {lon:.7f}, {height:.2f} m"
        )
        acked = result.get("acked")
        if acked is False:
            return {"success": False, "message": "Receiver rejected fixed-position command (NAK).", "device": device}
        return {
            "success": True,
            "message": (
                f"Fixed (stationary) mode set from {source}: "
                f"{lat:.7f}, {lon:.7f}, alt {height:.2f} m "
                f"(accuracy {data.accuracy} m)."
            ),
            "device": device,
            "position": {"lat": lat, "lon": lon, "height": height},
            "source": source,
            "tmode": "fixed",
        }

    @post("/test_serial")
    async def test_serial(self, data: RTKConfig) -> Dict[str, Any]:
        """Open the serial device briefly and report the RTCM messages seen."""
        device = find_serial_device(data.serial_device)
        if not device:
            return {"success": False, "message": f"Serial device not found: {data.serial_device}"}

        def _probe() -> Dict[str, Any]:
            parser = RTCMParser()
            nmea = NMEAScanner()
            counts: Dict[str, int] = {}
            base_pos = None
            total = 0
            satellites = None
            hdop = None
            fix_quality = None
            gps_position = None
            try:
                with serial.Serial(device, data.serial_baud, timeout=1) as ser:
                    end = time.time() + 6
                    while time.time() < end:
                        chunk = ser.read(4096)
                        if not chunk:
                            continue
                        for update in nmea.add_data(chunk):
                            if "satellites" in update:
                                satellites = update["satellites"]
                            if "hdop" in update:
                                hdop = update["hdop"]
                            if "fix_quality" in update:
                                fix_quality = update["fix_quality"]
                            if "gps_position" in update:
                                gps_position = update["gps_position"]
                        for msg in parser.add_data(chunk):
                            total += 1
                            mt = rtcm_message_type(msg)
                            counts[str(mt)] = counts.get(str(mt), 0) + 1
                            if mt in (1005, 1006) and base_pos is None:
                                base_pos = decode_1005(msg)
            except Exception as e:  # noqa: BLE001
                return {"success": False, "message": f"Serial error: {e}", "device": device}
            extras = []
            if satellites is not None:
                extras.append(f"{satellites} sats")
            if hdop is not None:
                extras.append(f"HDOP {hdop}")
            if gps_position is not None:
                extras.append(
                    f"{gps_position['lat']:.6f},{gps_position['lon']:.6f} alt {gps_position['alt']:.1f}m"
                )
            extra_txt = f" | {', '.join(extras)}" if extras else ""
            return {
                "success": total > 0,
                "device": device,
                "rtcm_messages": total,
                "message_counts": counts,
                "base_position": base_pos,
                "gps_position": gps_position,
                "satellites": satellites,
                "hdop": hdop,
                "fix_quality": fix_quality,
                "message": (
                    f"Received {total} RTCM messages ({', '.join(sorted(counts)) or 'none'}){extra_txt}"
                    if total
                    else "No RTCM messages received - is the F9P in base mode?"
                ),
            }

        return await asyncio.to_thread(_probe)

    @post("/test_caster")
    async def test_caster(self, data: RTKConfig) -> Dict[str, Any]:
        """Probe the caster with the SOURCE/POST handshake without streaming."""
        if self._status.streaming and self._config.mountpoint == data.mountpoint:
            return {
                "success": self._status.caster_connected,
                "message": "Stream is already live; using current connection status.",
                "response": self._status.caster_response,
            }
        errors = self._validate(data)
        if errors:
            return {"success": False, "message": "; ".join(errors)}
        try:
            reader, writer = await asyncio.wait_for(
                asyncio.open_connection(data.caster_host, data.caster_port), timeout=15
            )
        except Exception as e:  # noqa: BLE001
            return {"success": False, "message": f"Cannot reach {data.caster_host}:{data.caster_port}: {e}"}
        try:
            writer.write(self._build_handshake(data))
            await writer.drain()
            resp = await asyncio.wait_for(reader.readline(), timeout=15)
            text = resp.decode(errors="replace").strip()
            ok = ("200 OK" in text) or ("ICY 200" in text)
            msg = "Caster accepted credentials." if ok else f"Caster rejected: {text}"
            return {"success": ok, "message": msg, "response": text}
        except Exception as e:  # noqa: BLE001
            return {"success": False, "message": f"Handshake failed: {e}"}
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except Exception:  # noqa: BLE001
                pass

    # ---- NTRIP server core ------------------------------------------------- #
    def _build_handshake(self, cfg: RTKConfig) -> bytes:
        agent = "NTRIP BlueOS-RTK-Base/1.0"
        if cfg.ntrip_version == "v2":
            auth = base64.b64encode(f"{cfg.username}:{cfg.password}".encode()).decode()
            request = (
                f"POST /{cfg.mountpoint} HTTP/1.1\r\n"
                f"Host: {cfg.caster_host}:{cfg.caster_port}\r\n"
                f"Ntrip-Version: Ntrip/2.0\r\n"
                f"User-Agent: {agent}\r\n"
                f"Authorization: Basic {auth}\r\n"
                f"Content-Type: application/octet-stream\r\n"
                f"Ntrip-STR: \r\n"
                f"Connection: close\r\n"
                f"\r\n"
            )
        else:
            # NTRIP v1 source push (verified working against RTK2Go / SNIP).
            request = (
                f"SOURCE {cfg.password} /{cfg.mountpoint}\r\n"
                f"Source-Agent: {agent}\r\n"
                f"\r\n"
            )
        return request.encode()

    async def _stream_loop(self) -> None:
        delay = 5
        max_delay = 120
        while True:
            try:
                await self._stream_once()
                raise RuntimeError("Stream ended unexpectedly")
            except asyncio.CancelledError:
                self._status.streaming = False
                self._status.caster_connected = False
                self._status.serial_connected = False
                self._status.last_error = "Stopped by user"
                print("NTRIP base station stream cancelled")
                return
            except Exception as e:  # noqa: BLE001
                self._status.streaming = False
                self._status.caster_connected = False
                self._status.last_error = str(e)
                self._status.last_update = _now_iso()
                print(f"Stream error: {e}. Reconnecting in {delay}s")
                try:
                    await asyncio.sleep(delay)
                except asyncio.CancelledError:
                    return
                delay = min(delay * 2, max_delay)
            else:
                delay = 5

    async def _stream_once(self) -> None:
        cfg = self._config
        device = find_serial_device(cfg.serial_device)
        if not device:
            raise RuntimeError(f"Serial device not found: {cfg.serial_device}")

        loop = asyncio.get_running_loop()
        ser = await asyncio.to_thread(serial.Serial, device, cfg.serial_baud, timeout=0.5)
        self._serial = ser
        self._status.serial_connected = True
        print(f"Opened serial device {device} @ {cfg.serial_baud}")

        reader = writer = None
        try:
            reader, writer = await asyncio.wait_for(
                asyncio.open_connection(cfg.caster_host, cfg.caster_port), timeout=15
            )
            writer.write(self._build_handshake(cfg))
            await writer.drain()
            resp = await asyncio.wait_for(reader.readline(), timeout=15)
            text = resp.decode(errors="replace").strip()
            self._status.caster_response = text
            if not (("200 OK" in text) or ("ICY 200" in text)):
                raise RuntimeError(f"Caster rejected connection: {text or '(no response)'}")

            # Reset per-connection stats.
            self._status.caster_connected = True
            self._status.streaming = True
            self._status.last_error = None
            self._status.connected_since = _now_iso()
            self._status.bytes_pushed = 0
            self._status.rtcm_messages_pushed = 0
            self._status.message_counts = {}
            print(f"Caster connected ({text}); streaming RTCM from base station")

            parser = RTCMParser()
            self._ubx = UBXScanner()
            self._nmea = NMEAScanner()
            while True:
                chunk = await loop.run_in_executor(None, ser.read, 4096)
                if chunk:
                    for c, i, pl in self._ubx.add_data(chunk):
                        self._apply_ubx(c, i, pl)
                    for update in self._nmea.add_data(chunk):
                        self._apply_nmea(update)
                    for msg in parser.add_data(chunk):
                        writer.write(msg)
                        self._status.bytes_pushed += len(msg)
                        self._status.rtcm_messages_pushed += 1
                        mt = str(rtcm_message_type(msg))
                        self._status.message_counts[mt] = (
                            self._status.message_counts.get(mt, 0) + 1
                        )
                        if mt in ("1005", "1006"):
                            pos = decode_1005(msg)
                            if pos:
                                self._status.base_position = pos
                    await writer.drain()
                    self._status.last_update = _now_iso()

                # Detect caster-side disconnects without blocking the stream.
                try:
                    peek = await asyncio.wait_for(reader.read(256), timeout=0.001)
                    if peek == b"":
                        raise RuntimeError("Caster closed the connection")
                except asyncio.TimeoutError:
                    pass
        finally:
            self._serial = None
            self._status.serial_connected = False
            self._status.caster_connected = False
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass
            if writer is not None:
                writer.close()
                try:
                    await writer.wait_closed()
                except Exception:  # noqa: BLE001
                    pass


# --------------------------------------------------------------------------- #
# App wiring
# --------------------------------------------------------------------------- #
_rtk_controller: Optional[RTKController] = None


async def startup_hook() -> None:
    if _rtk_controller:
        await _rtk_controller.auto_start_if_enabled()


def setup_logging():
    log_dir = Path("./logs")
    log_dir.mkdir(parents=True, exist_ok=True)
    logging_config = LoggingConfig(
        loggers={__name__: dict(level="INFO", handlers=["queue_listener"])}
    )
    fh = logging.handlers.RotatingFileHandler(
        log_dir / "rtk.log", maxBytes=2**16, backupCount=1
    )
    return logging_config, fh


def create_app(args) -> Litestar:
    global _rtk_controller
    _global_config["config_file"] = args.config_file

    logging_config, fh = setup_logging()

    static_dirs = []
    if Path("./static").exists():
        static_dirs = ["./static"]
    elif Path("./app/static").exists():
        static_dirs = ["./app/static"]

    class RTKControllerSingleton(RTKController):
        def __init__(self, owner: "Litestar") -> None:
            global _rtk_controller
            if _rtk_controller is None:
                super().__init__(owner)
                _rtk_controller = self
            self.__dict__ = _rtk_controller.__dict__

    app = Litestar(
        route_handlers=[RTKControllerSingleton],
        state=State({}),
        static_files_config=[
            StaticFilesConfig(directories=static_dirs, path="/", html_mode=True)
        ]
        if static_dirs
        else [],
        logging_config=logging_config,
        on_startup=[startup_hook],
    )
    app.logger.addHandler(fh)
    return app


def parse_arguments():
    parser = argparse.ArgumentParser(description="BlueOS RTK Base Station Extension")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--config-file", default="config/rtk_config.json")
    parser.add_argument("--reload", action="store_true")
    return parser.parse_args()


def main():
    args = parse_arguments()
    print("RTK Base Station Extension")
    print("=" * 40)
    print(f"Config file: {args.config_file}")
    print(f"Web interface: http://{args.host}:{args.port}")
    app = create_app(args)
    try:
        import uvicorn

        uvicorn.run(app, host=args.host, port=args.port, reload=args.reload)
    except KeyboardInterrupt:
        print("Shutting down RTK Base Station Extension")
    except Exception as e:  # noqa: BLE001
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
