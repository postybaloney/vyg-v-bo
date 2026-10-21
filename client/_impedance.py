"""Contact impedance from one ADS1299 ramp cycle.

The firmware captures four 4-sample waveforms per cycle:

    vr, vf   electrode voltage, INxP vs SRB1, up-ramp then down-ramp
    ir, if   voltage across the 51 kΩ sense resistor, same two ramps

Current is that sense voltage / 51 kΩ. The two slopes cancel the body
voltage. What is left is fit to a parallel RC charged by the mean current:

    V(t) = I·(Rs + Rp) − I·Rp·exp(−t / (Rp·C))

ADC channels 1..8 are EMG1..8, 9..16 are EEG1..8. Parity 0 steers channels
13 and 14 onto SRB1 and BIAS. Parity 1 steers channels 15 and 16. Channels
1..12 are electrodes every cycle; the other pair is its own electrodes.
Z_REF_BIAS_ENABLED 0 drops the SRB1/BIAS rows. A channel that is not
impedance-enabled is left unchanged. All-zero codes are a chip that was
not read.
"""

from __future__ import annotations

import math
import struct
import zlib

import numpy as np

from _protocol import (
    HOST_CMD_MAGIC,
    MAGIC_BYTES,
    TYPE_Z,
    Z_ENABLED,
    Z_PACKET_SIZE,
    Z_REF_BIAS_ENABLED,
    ADS_VREF_VOLTS,
    FrameParser,
    ImpedanceWave,
)

Z_SENSE_OHM = 51_000.0
Z_T0_S = 200e-6
Z_LSB_V = (2.0 * ADS_VREF_VOLTS) / ((2**24) - 1)  # gain 1
Z_OPEN_OHM = 1.0e9
def _redirected(parity: int) -> tuple[int, int]:
    """ADC indexes reading SRB1 then BIAS this cycle.

    Parity 0 is channels 13 and 14. Parity 1 is channels 15 and 16.
    """
    if parity & 1:
        return 14, 15
    return 12, 13


def _sense_adc(parity: int = 0) -> tuple[int, int] | None:
    """Redirected indexes when Reference and Bias are reported, else None."""
    if not Z_REF_BIAS_ENABLED:
        return None
    return _redirected(parity)


def _has_codes(code: np.ndarray, adc: int) -> bool:
    return bool(np.any(np.asarray(code)[:, adc, :] != 0))
_TAUS = np.geomspace(30e-6, 20e-3, 64)

Z_ELECTRODES: tuple[str, ...] = (
    *(f"EEG{i}" for i in range(1, 9)),
    *(f"EMG{i}" for i in range(1, 9)),
    "Reference",
    "Bias",
)
Z_COUNT = len(Z_ELECTRODES)


def _ui_index(adc: int) -> int:
    # ADC 1..8 (0..7) → EMG slots 8..15. ADC 9..16 (8..15) → EEG slots 0..7.
    if adc < 8:
        return 8 + adc
    return adc - 8


def _adc_of_ui(index: int) -> int:
    if index < 8:
        return index + 8
    return index - 8


def _timebase(dt_us: np.ndarray) -> np.ndarray:
    nominal = Z_T0_S + np.arange(4, dtype=np.float64) * 62.5e-6
    a = np.asarray(dt_us[0], dtype=np.float64)
    b = np.asarray(dt_us[1], dtype=np.float64)
    if a.shape != (4,) or b.shape != (4,):
        return nominal
    if np.all(np.diff(a) > 0) and np.all(np.diff(b) > 0):
        return Z_T0_S + 0.5 * (a + b) * 1e-6
    if np.all(np.diff(a) > 0):
        return Z_T0_S + a * 1e-6
    return nominal


def _fit_parallel_rc(t: np.ndarray, volts: np.ndarray, amps: float) -> tuple[float | None, float | None]:
    if not math.isfinite(amps) or abs(amps) < 2e-9:
        return Z_OPEN_OHM, None
    y = np.asarray(volts, dtype=np.float64)
    if y.shape != (4,) or not np.all(np.isfinite(y)):
        return None, None
    r_mean = abs(float(np.mean(y)) / amps)
    spread = float(np.max(y) - np.min(y))
    if spread < 5e-6 or spread < 0.02 * max(r_mean, 1.0) * abs(amps):
        return (Z_OPEN_OHM if r_mean > 5e6 else r_mean), None

    e = np.exp(-t[None, :] / _TAUS[:, None])
    se = e.sum(axis=1)
    see = np.sum(e * e, axis=1)
    sy = float(y.sum())
    sye = e @ y
    det = 4.0 * see - se * se
    good = np.abs(det) > 1e-12
    a_coef = np.full(_TAUS.shape, np.nan)
    b_coef = np.full(_TAUS.shape, np.nan)
    a_coef[good] = (sy * see[good] - se[good] * sye[good]) / det[good]
    b_coef[good] = (4.0 * sye[good] - se[good] * sy) / det[good]
    pred = a_coef[:, None] + b_coef[:, None] * e
    resid = np.sum((pred - y[None, :]) ** 2, axis=1)
    resid[~good] = np.inf
    k = int(np.argmin(resid))
    const_resid = float(np.sum((y - np.mean(y)) ** 2))
    if not np.isfinite(resid[k]) or resid[k] > 0.35 * max(const_resid, 1e-18):
        return (Z_OPEN_OHM if r_mean > 5e6 else r_mean), None

    r_dc = abs(float(a_coef[k]) / amps)
    rp = -float(b_coef[k]) / amps
    cap = None
    if rp > 50.0:
        c_try = float(_TAUS[k]) / rp
        if 1e-12 <= c_try <= 1e-4:
            cap = c_try
    if r_dc > 5e6:
        return Z_OPEN_OHM, None
    if r_dc < 1.0:
        return None, None
    return r_dc, cap


def _driven(up: np.ndarray, down: np.ndarray, floor: float) -> float | None:
    """Demodulated mean of a reversed ramp, or None when it is noise.

    Up and down are opposite polarity, so their difference keeps the ramp and
    cancels a DC offset. On an open pin both sides are noise: the difference
    is no larger than the scatter inside each side, and |V|/|I| wanders.
    """
    up = np.asarray(up, dtype=np.float64)
    down = np.asarray(down, dtype=np.float64)
    if up.shape != (4,) or down.shape != (4,):
        return None
    if not np.all(np.isfinite(up)) or not np.all(np.isfinite(down)):
        return None
    demod = float(np.mean(0.5 * (up - down)))
    scatter = float(np.std(up) + np.std(down))
    if abs(demod) < floor or abs(demod) < scatter:
        return None
    return demod


def _fit_channel(vu: np.ndarray, vd: np.ndarray, iu: np.ndarray, id_: np.ndarray, t: np.ndarray):
    amps = _driven(iu, id_, 20e-9)
    volts = _driven(vu, vd, 5e-6)
    if amps is None or volts is None:
        return Z_OPEN_OHM, None
    return _fit_parallel_rc(t, 0.5 * (vu - vd), amps)


def impedances_from_codes(
    parity: int,
    dt_us: np.ndarray,
    code: np.ndarray,
) -> list[tuple[float | None, float | None, bool]]:
    """18 electrodes: (resistance_ohm, capacitance_F, updated_this_cycle)."""
    rows: list[tuple[float | None, float | None, bool]] = [(None, None, False)] * Z_COUNT
    if parity not in (0, 1):
        return rows
    codes = np.asarray(code)
    if codes.shape != (4, 16, 4):
        return rows
    volts = codes.astype(np.float64) * Z_LSB_V
    amps = volts / Z_SENSE_OHM
    t = _timebase(np.asarray(dt_us))
    redirected = _redirected(parity)
    for adc in range(16):
        if adc in redirected:
            # These inputs are SRB1 and BIAS, not their own electrodes.
            # Zeros mean that ADC was not read; leave the last reading alone.
            if Z_REF_BIAS_ENABLED and _has_codes(codes, adc):
                r_ohm, c_f = _fit_channel(volts[0, adc], volts[1, adc], amps[2, adc], amps[3, adc], t)
                rows[16 if adc == redirected[0] else 17] = (r_ohm, c_f, True)
            continue
        if not Z_ENABLED[adc]:
            continue
        r_ohm, c_f = _fit_channel(volts[0, adc], volts[1, adc], amps[2, adc], amps[3, adc], t)
        rows[_ui_index(adc)] = (r_ohm, c_f, True)
    return rows


def impedances_from_wave(wave: ImpedanceWave) -> list[tuple[float | None, float | None, bool]]:
    return impedances_from_codes(wave.parity, wave.dt_us, wave.code)


def _encode_wave(parity: int, dt_us: np.ndarray, code: np.ndarray) -> bytes:
    payload = bytearray(Z_PACKET_SIZE)
    payload[0] = parity & 0xFF
    payload[1] = 1
    flat_dt = np.asarray(dt_us, dtype="<u2").reshape(-1)
    flat_code = np.asarray(code, dtype="<i4").reshape(-1)
    payload[2:34] = flat_dt.tobytes()
    payload[34 : 34 + 1024] = flat_code.tobytes()
    crc = zlib.crc32(payload[:-4]) & 0xFFFFFFFF
    struct.pack_into("<I", payload, Z_PACKET_SIZE - 4, crc)
    return MAGIC_BYTES + bytes((TYPE_Z,)) + bytes(payload)


def _synth(rs: float, rp: float, cap: float, *, resistive: bool = False):
    i_amp = 1.5e-6
    v_body = 0.015
    t = Z_T0_S + np.arange(4) * 62.5e-6
    if resistive:
        v = np.full(4, i_amp * rs)
    else:
        tau = rp * cap
        v = i_amp * rs + i_amp * rp * (1.0 - np.exp(-t / tau))
    vu = np.round((v + v_body) / Z_LSB_V).astype(np.int32)
    vd = np.round((-v + v_body) / Z_LSB_V).astype(np.int32)
    iu = np.round(np.full(4, i_amp * Z_SENSE_OHM) / Z_LSB_V).astype(np.int32)
    id_ = np.round(np.full(4, -i_amp * Z_SENSE_OHM) / Z_LSB_V).astype(np.int32)
    code = np.zeros((4, 16, 4), dtype=np.int32)
    code[0, :, :] = vu
    code[1, :, :] = vd
    code[2, :, :] = iu
    code[3, :, :] = id_
    dt = np.tile(np.array([0, 63, 125, 188], dtype=np.uint16), (4, 1))
    return dt, code


def _self_test() -> None:
    rs, rp, cap = 2_000.0, 18_000.0, 47e-9
    dt, code = _synth(rs, rp, cap)
    packet = _encode_wave(0, dt, code)
    parser = FrameParser()
    assert parser.feed(packet[:80]) == []
    assert parser.feed(packet[80:]) == []
    waves = parser.take_impedance()
    assert len(waves) == 1 and waves[0].parity == 0
    rows = impedances_from_wave(waves[0])
    assert _redirected(0) == (12, 13)  # channels 13, 14
    assert _sense_adc(0) == (12, 13)
    assert rows[_ui_index(12)][2] is False and rows[_ui_index(13)][2] is False
    assert rows[16][2] is True and rows[17][2] is True
    assert rows[_ui_index(0)][2] is True  # EMG1 is an electrode on every cycle
    assert rows[0][2] is False  # EEG1 impedance is off
    r_ohm, c_f, _ = rows[16]
    assert r_ohm is not None and abs(r_ohm - (rs + rp)) / (rs + rp) < 0.15, r_ohm
    assert c_f is not None and abs(c_f - cap) / cap < 0.25, c_f
    emg_r, emg_c, emg_ok = rows[_ui_index(0)]
    assert emg_ok and emg_r is not None and abs(emg_r - (rs + rp)) / (rs + rp) < 0.15, emg_r
    assert emg_c is not None

    dt_r, code_r = _synth(8_000.0, 0.0, 0.0, resistive=True)
    assert _sense_adc(1) == (14, 15)  # channels 15, 16
    later = impedances_from_codes(1, dt_r, code_r)
    r_only, c_only, updated = later[_ui_index(0)]
    assert updated and r_only is not None and abs(r_only - 8_000.0) / 8_000.0 < 0.05, r_only
    assert c_only is None
    bias_r, bias_c, bias_updated = later[_ui_index(1)]
    assert bias_updated and bias_r is not None and abs(bias_r - 8_000.0) / 8_000.0 < 0.05, bias_r
    assert bias_c is None
    assert later[16][2] is True and later[17][2] is True
    assert later[_ui_index(14)][2] is False and later[_ui_index(15)][2] is False
    assert later[0][2] is False  # EEG1

    rng = np.random.default_rng(1)
    dt_n = np.tile(np.array([0, 63, 125, 188], dtype=np.uint16), (4, 1))
    i_code = int(round(1.5e-6 * Z_SENSE_OHM / Z_LSB_V))
    for _ in range(40):
        code_n = np.clip(np.round(rng.normal(0, 300e-6 / Z_LSB_V, size=(4, 16, 4))), -8_000_000, 8_000_000)
        floating = impedances_from_codes(0, dt_n, code_n.astype(np.int32))
        assert floating[16][0] == Z_OPEN_OHM and floating[17][0] == Z_OPEN_OHM
        code_n[2, :, :] = i_code
        code_n[3, :, :] = -i_code
        driven_open = impedances_from_codes(0, dt_n, code_n.astype(np.int32))
        assert driven_open[16][0] == Z_OPEN_OHM and driven_open[17][0] == Z_OPEN_OHM
    assert HOST_CMD_MAGIC == bytes((0xA5, 0x5A))
    print("impedance self-test ok", round(r_ohm), round(c_f * 1e9, 2), "nF")


if __name__ == "__main__":
    _self_test()
