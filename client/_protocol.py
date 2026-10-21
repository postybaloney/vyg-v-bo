"""Host protocol for the 8+8 EEG/EMG firmware.

The same bytes go over USB or Bluetooth. firmware/main/config.h DATA_LINK picks one.

Wire layout, little-endian (same as firmware/main/main.c and main.py):

    magic u32 = 0x574F4F44
    type  u8
        0x01 EMG: one 61-byte frame (8 channels, its own sample index)
        0x03 EEG: same frame layout
        0x02 log: u16 length, then that many text bytes
        0x04 impedance: 1062-byte z_packet_t (see client/_impedance.py)
        0x05 audio: u32 seq, 160 int16 samples, u32 crc (16 kHz mic frames)

Host → device commands start with 0xA5 0x5A. Then 0x01 (impedance on),
0x00 (back to normal acquisition), or 0x10 plus one playback frame
(u32 seq, u16 count = 160, int16 samples[160]).

The inner frame is magic, version 3, length, chip, time, index, 8 int32
channels, CRC32 (zlib / esp_crc32_le over the preceding bytes).

The parser emits one 16-channel sample once every enabled ADC has a fresh
frame. A disabled ADC contributes zeros. Order is EEG1..EEG8, then EMG1..EMG8.
The sample index counts emitted samples, so the host buffer stays contiguous.
"""

from __future__ import annotations

import os
import re
import struct
import sys
import time
import zlib
from dataclasses import dataclass

import numpy as np

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - client requires pyserial
    serial = None  # type: ignore[assignment]
    list_ports = None  # type: ignore[assignment]

MAGIC = 0x574F4F44
MAGIC_BYTES = struct.pack("<I", MAGIC)
VERSION = 3
TYPE_EMG = 0x01
TYPE_LOG = 0x02
TYPE_EEG = 0x03
TYPE_Z = 0x04
TYPE_AUDIO = 0x05
HOST_CMD_MAGIC = bytes((0xA5, 0x5A))
HOST_CMD_Z_OFF = 0x00
HOST_CMD_Z_ON = 0x01
HOST_CMD_AUDIO = 0x10
AUDIO_RATE_HZ = 16000
AUDIO_SAMPLES = 160
AUDIO_PACKET = struct.Struct("<I160hI")
AUDIO_PACKET_SIZE = AUDIO_PACKET.size
assert AUDIO_PACKET_SIZE == 328
HOST_AUDIO_PAYLOAD = struct.Struct("<IH160h")
assert HOST_AUDIO_PAYLOAD.size == 326


def data_link() -> str:
    """usb or ble, from firmware/main/config.h DATA_LINK. One switch for the whole link."""
    path = os.path.join(os.path.dirname(__file__), "..", "firmware", "main", "config.h")
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    match = re.search(r"(?m)^#define DATA_LINK\s+DATA_LINK_(USB|BLE)\s*$", text)
    if not match:
        raise RuntimeError(f"DATA_LINK missing in {path}")
    return match.group(1).lower()


DATA_LINK = data_link()


def audio_enabled() -> bool:
    """True when firmware/main/config.h sets AUDIO_ENABLED to 1."""
    path = os.path.join(os.path.dirname(__file__), "..", "firmware", "main", "config.h")
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    match = re.search(r"(?m)^#define AUDIO_ENABLED\s+([01])\s*$", text)
    if not match:
        raise RuntimeError(f"AUDIO_ENABLED missing in {path}")
    return match.group(1) == "1"


AUDIO_ENABLED = audio_enabled()


def _config_text() -> str:
    path = os.path.join(os.path.dirname(__file__), "..", "firmware", "main", "config.h")
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def _flag(text: str, name: str) -> int:
    match = re.search(rf"(?m)^#define {name}\s+([01])\s*$", text)
    if not match:
        raise RuntimeError(f"{name} missing in firmware/main/config.h")
    return int(match.group(1))


def _electrode_name(text: str, define: str) -> str:
    match = re.search(rf'(?m)^#define {define}\s+"((?:EMG|EEG)[1-8])"\s*$', text)
    if not match:
        raise RuntimeError(f"{define} must be \"EMG1\"..\"EMG8\" or \"EEG1\"..\"EEG8\"")
    return match.group(1)


def _adc_of_name(name: str) -> int:
    number = int(name[3]) - 1
    return number if name.startswith("EMG") else 8 + number


def _load_chip_config() -> tuple[tuple[bool, bool], tuple[bool, ...], tuple[bool, ...], int, int, bool]:
    """ADC enables, sampling mask, impedance mask, reference ADC, bias ADC.

    Masks and the two ids are ADC order: 0..7 EMG, 8..15 EEG.
    """
    text = _config_text()
    adc = (_flag(text, "EMG_ADC_ENABLED") == 1, _flag(text, "EEG_ADC_ENABLED") == 1)
    if not any(adc):
        raise RuntimeError("both ADCs disabled")
    sample = []
    impedance = []
    for chip, prefix in ((0, "EMG"), (1, "EEG")):
        for n in range(1, 9):
            on = _flag(text, f"{prefix}{n}_ENABLED") == 1
            z_on = _flag(text, f"{prefix}{n}_Z_ENABLED") == 1
            if (on or z_on) and not adc[chip]:
                kind = "impedance" if z_on and not on else "sampling"
                if on and z_on:
                    kind = "sampling and impedance"
                raise RuntimeError(f"{prefix}{n} {kind} on a disabled ADC")
            sample.append(on)
            impedance.append(z_on)
    ref_name = _electrode_name(text, "Z_REF_ELECTRODE")
    bias_name = _electrode_name(text, "Z_BIAS_ELECTRODE")
    if ref_name == bias_name:
        raise RuntimeError("reference and bias are the same electrode")
    ids = []
    for role, name in (("reference", ref_name), ("bias", bias_name)):
        index = _adc_of_name(name)
        chip = 1 if index >= 8 else 0
        if not adc[chip]:
            raise RuntimeError(f"{role} electrode {name} is on a disabled ADC")
        if role == "reference" and not sample[index]:
            raise RuntimeError(f"{role} electrode {name} is not enabled for sampling")
        if role == "reference" and not impedance[index]:
            raise RuntimeError(f"{role} electrode {name} is not enabled for impedance")
        ids.append(index)
    ref_i, bias_i = ids
    if ref_i // 8 != bias_i // 8:
        raise RuntimeError("reference and bias must be on the same ADC")
    lo, hi = sorted((ref_i & 7, bias_i & 7))
    if (lo % 2) != 0 or hi != lo + 1:
        raise RuntimeError("reference and bias must be channels 1-2, 3-4, 5-6, or 7-8")
    read_ref_bias = _flag(text, "Z_REF_BIAS_ENABLED") == 1
    return adc, tuple(sample), tuple(impedance), ids[0], ids[1], read_ref_bias


ADC_ENABLED, SAMPLE_ENABLED, Z_ENABLED, Z_REF_ADC, Z_BIAS_ADC, Z_REF_BIAS_ENABLED = _load_chip_config()
# firmware/main/impedance.h z_packet_t
Z_PACKET = struct.Struct("<BB16H256iI")
Z_PACKET_SIZE = Z_PACKET.size
assert Z_PACKET_SIZE == 1062
CHIP_EMG = 0
CHIP_EEG = 1
CHIP_OF_TYPE = {TYPE_EMG: CHIP_EMG, TYPE_EEG: CHIP_EEG}
INNER_FRAME = struct.Struct("<IHHBQQ8iI")
INNER_FRAME_SIZE = INNER_FRAME.size
DATA_PACKET_SIZE = 5 + INNER_FRAME_SIZE
LOG_MAX = 512
ESPRESSIF_VID = 0x303A

EEG_COUNT = 8
EMG_COUNT = 8
CHANNEL_COUNT = EEG_COUNT + EMG_COUNT

# Session file: sample_index, mcu_time_us, 16 channels in microvolts (float32).
EEG_RECORD_FORMAT = "<QQ16f"
# Older headset recordings (16 EEG + 16 EMG). Read-only.
EEG_RECORD_FORMAT_32 = "<QQ32f"
EEG_RECORD_FORMAT_CODES_LEGACY = "<QQ32i"
CHANNEL_UNITS = "uV"

ADS_VREF_VOLTS = 4.5
_ADS_GAINS = {1, 2, 4, 6, 8, 12, 24}


def _gain(text: str, name: str) -> float:
    match = re.search(rf"(?m)^#define {name}\s+(\d+)\s*$", text)
    if not match:
        raise RuntimeError(f"{name} missing in firmware/main/config.h")
    gain = int(match.group(1))
    if gain not in _ADS_GAINS:
        raise RuntimeError(f"{name} must be 1, 2, 4, 6, 8, 12, or 24")
    return float(gain)


_config = _config_text()
EEG_GAIN = _gain(_config, "EEG_GAIN")
EMG_GAIN = _gain(_config, "EMG_GAIN")

assert INNER_FRAME_SIZE == 61
assert CHANNEL_COUNT == 16


def lsb_volts(gain: float) -> float:
    return (2.0 * ADS_VREF_VOLTS / gain) / ((2**24) - 1)


EEG_LSB_UV = lsb_volts(EEG_GAIN) * 1e6
EMG_LSB_UV = lsb_volts(EMG_GAIN) * 1e6


def ads_calibration_metadata() -> dict[str, float | str]:
    """Calibration used by codes_to_uv(); stored in session_meta.json."""
    return {
        "units": CHANNEL_UNITS,
        "ads_vref_volts": ADS_VREF_VOLTS,
        "eeg_gain": EEG_GAIN,
        "emg_gain": EMG_GAIN,
        "eeg_lsb_uv": EEG_LSB_UV,
        "emg_lsb_uv": EMG_LSB_UV,
        "formula": "uv = code * (2 * Vref / gain) / (2**24 - 1) * 1e6",
        "eeg_channels": "0..7",
        "emg_channels": "8..15",
    }


def codes_to_uv(channels_i32: np.ndarray) -> np.ndarray:
    """Convert ADS1299 signed codes to microvolts (single conversion path)."""
    out = np.asarray(channels_i32, dtype=np.float64)
    if out.shape != (CHANNEL_COUNT,):
        raise ValueError(f"Expected {CHANNEL_COUNT} channels, got shape {out.shape}")
    out[:EEG_COUNT] *= EEG_LSB_UV
    out[EEG_COUNT:] *= EMG_LSB_UV
    return out


@dataclass(slots=True)
class SampleFrame:
    mcu_time_us: int
    sample_index: int
    channels_i32: np.ndarray  # shape=(16,), int32, EEG then EMG

    def channels_uv(self) -> np.ndarray:
        return codes_to_uv(self.channels_i32)


@dataclass(slots=True)
class AudioFrame:
    seq: int
    samples: np.ndarray  # int16, shape=(160,)


@dataclass(slots=True)
class ImpedanceWave:
    """One firmware impedance cycle, still in ADS codes at gain 1.

    dt_us: (4 phases, 4 samples) microseconds from the first sample of that phase.
    code: (4 phases, 16 ADC channels, 4 samples). Phase order is vr, vf, ir, if.
    ADC 0..7 are EMG channels 1..8, ADC 8..15 are EEG channels 9..16.
    Parity 0 reads SRB1 and BIAS on channels 13 and 14. Parity 1 uses 15 and 16.
    """

    parity: int
    dt_us: np.ndarray
    code: np.ndarray


@dataclass(slots=True)
class _ChipHold:
    mcu_time_us: int
    sample_index: int
    channels: np.ndarray


def _decode_inner(payload: bytes, typ: int):
    magic, version, nbytes, chip, t_us, index, *chs, crc = INNER_FRAME.unpack(payload)
    if magic != MAGIC or version != VERSION or nbytes != INNER_FRAME_SIZE:
        return None
    if chip != CHIP_OF_TYPE.get(typ):
        return None
    if (zlib.crc32(payload[:-4]) & 0xFFFFFFFF) != crc:
        return None
    return chip, int(t_us), int(index), chs


class FrameParser:
    """Byte parser. feed() returns combined 16-channel frames.

    Firmware log lines are queued and pulled with take_logs().
    """

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._logs: list[str] = []
        self._impedance: list[ImpedanceWave] = []
        self._audio: list[AudioFrame] = []
        self._hold: dict[int, _ChipHold] = {}
        self._fresh: set[int] = set()
        self._out_index = 0
        self.desync_count = 0
        self.bad_checksum_count = 0

    def take_logs(self) -> list[str]:
        logs = self._logs
        self._logs = []
        return logs

    def take_impedance(self) -> list[ImpedanceWave]:
        waves = self._impedance
        self._impedance = []
        return waves

    def take_audio(self) -> list[AudioFrame]:
        frames = self._audio
        self._audio = []
        return frames

    def _assemble(self, chip: int, t_us: int, index: int, chs: list[int]) -> SampleFrame | None:
        self._hold[chip] = _ChipHold(
            mcu_time_us=t_us,
            sample_index=index,
            channels=np.asarray(chs, dtype=np.int32),
        )
        self._fresh.add(chip)
        needed = [i for i, on in enumerate(ADC_ENABLED) if on]
        if any(i not in self._fresh for i in needed):
            return None
        self._fresh.clear()
        zeros = np.zeros(8, dtype=np.int32)
        eeg = self._hold[CHIP_EEG].channels if ADC_ENABLED[1] else zeros
        emg = self._hold[CHIP_EMG].channels if ADC_ENABLED[0] else zeros
        present = [self._hold[i] for i in needed]
        channels = np.empty(CHANNEL_COUNT, dtype=np.int32)
        channels[:EEG_COUNT] = eeg
        channels[EEG_COUNT:] = emg
        frame = SampleFrame(
            mcu_time_us=max(h.mcu_time_us for h in present),
            sample_index=self._out_index,
            channels_i32=channels,
        )
        self._out_index += 1
        return frame

    def feed(self, data: bytes) -> list[SampleFrame]:
        self._buffer.extend(data)
        frames: list[SampleFrame] = []
        while True:
            i = self._buffer.find(MAGIC_BYTES)
            if i < 0:
                if len(self._buffer) > 3:
                    self.desync_count += 1
                    del self._buffer[:-3]
                break
            if i:
                self.desync_count += 1
                del self._buffer[:i]
            if len(self._buffer) < 5:
                break
            typ = self._buffer[4]
            if typ in CHIP_OF_TYPE:
                if len(self._buffer) < DATA_PACKET_SIZE:
                    break
                payload = bytes(self._buffer[5:DATA_PACKET_SIZE])
                crc = struct.unpack_from("<I", payload, INNER_FRAME_SIZE - 4)[0]
                if (zlib.crc32(payload[:-4]) & 0xFFFFFFFF) != crc:
                    self.bad_checksum_count += 1
                    self.desync_count += 1
                    del self._buffer[0]
                    continue
                decoded = _decode_inner(payload, typ)
                if decoded is None:
                    self.desync_count += 1
                    del self._buffer[0]
                    continue
                del self._buffer[:DATA_PACKET_SIZE]
                frame = self._assemble(*decoded)
                if frame is not None:
                    frames.append(frame)
            elif typ == TYPE_LOG:
                if len(self._buffer) < 7:
                    break
                ln = struct.unpack_from("<H", self._buffer, 5)[0]
                if ln > LOG_MAX:
                    self.desync_count += 1
                    del self._buffer[0]
                    continue
                if len(self._buffer) < 7 + ln:
                    break
                text = bytes(self._buffer[7 : 7 + ln]).decode("utf-8", "replace")
                del self._buffer[: 7 + ln]
                self._logs.append(text)
            elif typ == TYPE_Z:
                if len(self._buffer) < 5 + Z_PACKET_SIZE:
                    break
                payload = bytes(self._buffer[5 : 5 + Z_PACKET_SIZE])
                crc = struct.unpack_from("<I", payload, Z_PACKET_SIZE - 4)[0]
                if (zlib.crc32(payload[:-4]) & 0xFFFFFFFF) != crc:
                    self.bad_checksum_count += 1
                    self.desync_count += 1
                    del self._buffer[0]
                    continue
                del self._buffer[: 5 + Z_PACKET_SIZE]
                if payload[1] != 1:
                    continue
                dt = np.frombuffer(payload[2:34], dtype="<u2").reshape(4, 4).copy()
                code = np.frombuffer(payload[34 : 34 + 1024], dtype="<i4").reshape(4, 16, 4).copy()
                self._impedance.append(ImpedanceWave(parity=int(payload[0]), dt_us=dt, code=code))
            elif typ == TYPE_AUDIO:
                if len(self._buffer) < 5 + AUDIO_PACKET_SIZE:
                    break
                payload = bytes(self._buffer[5 : 5 + AUDIO_PACKET_SIZE])
                crc = struct.unpack_from("<I", payload, AUDIO_PACKET_SIZE - 4)[0]
                if (zlib.crc32(payload[:-4]) & 0xFFFFFFFF) != crc:
                    self.bad_checksum_count += 1
                    self.desync_count += 1
                    del self._buffer[0]
                    continue
                del self._buffer[: 5 + AUDIO_PACKET_SIZE]
                seq = struct.unpack_from("<I", payload, 0)[0]
                samples = np.frombuffer(payload, dtype="<i2", count=AUDIO_SAMPLES, offset=4).copy()
                self._audio.append(AudioFrame(seq=int(seq), samples=samples))
            else:
                self.desync_count += 1
                del self._buffer[0]
        return frames


def _require_serial():
    if serial is None or list_ports is None:
        raise RuntimeError("pyserial is not installed (pip install pyserial)")


def open_port(path: str, baud: int) -> serial.Serial:
    """Open USB-Serial/JTAG with RTS already low.

    pyserial asserts RTS in open(), and RTS holds EN in reset.
    """
    _require_serial()
    ser = serial.Serial()
    ser.port = path
    ser.baudrate = baud
    ser.timeout = 0.05
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def wait_port(path: str, baud: int, seconds: float = 5.0) -> serial.Serial:
    _require_serial()
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        found = path if os.path.exists(path) else None
        if found is None:
            esp = [p.device for p in list_ports.comports() if p.vid == ESPRESSIF_VID]
            if len(esp) == 1:
                found = esp[0]
        if found:
            try:
                return open_port(found, baud)
            except serial.SerialException:
                pass
        time.sleep(0.05)
    raise RuntimeError(f"{path} did not come back after reset")


def reset_chip(path: str, baud: int) -> serial.Serial:
    """Pulse EN the way esptool does, so the boot link test runs again."""
    _require_serial()
    print(f"reset {path}", file=sys.stderr)
    ser = open_port(path, baud)
    try:
        ser.rts = True
        time.sleep(0.2)
        try:
            ser.rts = False
        except OSError:
            pass
        time.sleep(0.2)
    finally:
        try:
            ser.close()
        except OSError:
            pass
    return wait_port(path, baud)


def find_esp_port() -> str | None:
    """The Espressif USB port when exactly one is present."""
    _require_serial()
    esp = [p.device for p in list_ports.comports() if p.vid == ESPRESSIF_VID]
    if len(esp) == 1:
        return esp[0]
    return None


@dataclass(slots=True)
class BootSession:
    port: str
    passed: bool
    summary: str
    serial: serial.Serial
    parser: FrameParser


def run_boot_test(port: str, baud: int, timeout_s: float = 8.0) -> BootSession:
    """Reset the chip and print the firmware link test before streaming.

    Firmware prints LINK PASS / LINK FAIL from link_test() during ads_bringup.
    The returned port stays open; the parser keeps any partial tail byte.
    """
    print("--- boot test ---", flush=True)
    ser = reset_chip(port, baud)
    parser = FrameParser()
    passed = False
    saw_result = False
    frames = 0
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            try:
                data = ser.read(4096)
            except serial.SerialException:
                try:
                    ser.close()
                except OSError:
                    pass
                ser = wait_port(port, baud)
                continue
            if not data:
                continue
            frames += len(parser.feed(data))
            for line in parser.take_logs():
                sys.stdout.write(line)
                sys.stdout.flush()
                if "LINK PASS" in line:
                    passed = True
                    saw_result = True
                elif "LINK FAIL" in line:
                    passed = False
                    saw_result = True
            if passed and frames > 0:
                break
    except BaseException:
        try:
            ser.close()
        except OSError:
            pass
        raise

    if passed and frames > 0:
        summary = f"LINK PASS, {frames} paired frames"
    elif passed:
        summary = "LINK PASS, no paired frames yet"
    elif saw_result:
        summary = "LINK FAIL"
    else:
        summary = "no link-test line in boot log"
        passed = False
    print(f"--- boot test {summary} ---", flush=True)
    return BootSession(
        port=port,
        passed=passed,
        summary=summary,
        serial=ser,
        parser=parser,
    )
