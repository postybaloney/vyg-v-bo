"""Bluetooth transport for the whole USB packet stream.

Used when firmware DATA_LINK is DATA_LINK_BLE. Notifications are chunks of the
same byte stream USB sends (EXG, logs, impedance, audio). Writes are the same
host command bytes. The laptop parser does not change.
"""

from __future__ import annotations

import asyncio
import queue
import struct
import sys
import threading
from typing import Callable

import numpy as np

from _protocol import (
    AUDIO_SAMPLES,
    HOST_CMD_AUDIO,
    HOST_CMD_MAGIC,
    AudioFrame,
    FrameParser,
    ImpedanceWave,
    SampleFrame,
)

BLE_NAME = "Voyage"
BLE_TX = "594f5901-0000-1000-8000-00805f9b34fb"  # device → host
BLE_RX = "594f5902-0000-1000-8000-00805f9b34fb"  # host → device


class BleReader(threading.Thread):
    def __init__(
        self,
        out_queue: queue.Queue,
        z_queue: queue.Queue[ImpedanceWave],
        audio_queue: queue.Queue[AudioFrame],
        wrap_frame: Callable[[SampleFrame], object],
    ) -> None:
        super().__init__(daemon=True)
        self._out_queue = out_queue
        self._z_queue = z_queue
        self._audio_queue = audio_queue
        self._wrap_frame = wrap_frame
        self._stop_event = threading.Event()
        self._write_q: queue.Queue[bytes] = queue.Queue(maxsize=32)
        self.parser = FrameParser()
        self.last_error: str | None = None
        self.boot_passed = False
        self.boot_summary = "connecting"

    def stop(self) -> None:
        self._stop_event.set()

    def send_host_cmd(self, cmd: int) -> None:
        self._enqueue_write(HOST_CMD_MAGIC + bytes((cmd & 0xFF,)))

    def send_audio_frame(self, seq: int, samples: np.ndarray) -> None:
        pcm = np.ascontiguousarray(samples, dtype="<i2")
        if pcm.shape != (AUDIO_SAMPLES,):
            raise ValueError(f"expected {AUDIO_SAMPLES} samples, got {pcm.shape}")
        payload = (
            HOST_CMD_MAGIC
            + bytes((HOST_CMD_AUDIO,))
            + struct.pack("<IH", seq & 0xFFFFFFFF, AUDIO_SAMPLES)
            + pcm.tobytes()
        )
        self._enqueue_write(payload)

    def _enqueue_write(self, payload: bytes) -> None:
        try:
            self._write_q.put(payload, timeout=0.2)
        except queue.Full:
            self.last_error = "bluetooth write stalled"
            raise RuntimeError("bluetooth write stalled")

    def _on_notify(self, _handle: int, data: bytearray) -> None:
        frames = self.parser.feed(bytes(data))
        for frame in frames:
            try:
                self._out_queue.put_nowait(self._wrap_frame(frame))
            except queue.Full:
                pass
        if frames and not self.boot_passed:
            self.boot_passed = True
            self.boot_summary = "bluetooth connected"
        for wave in self.parser.take_impedance():
            try:
                self._z_queue.put_nowait(wave)
            except queue.Full:
                try:
                    self._z_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._z_queue.put_nowait(wave)
                except queue.Full:
                    pass
        for audio in self.parser.take_audio():
            try:
                self._audio_queue.put_nowait(audio)
            except queue.Full:
                try:
                    self._audio_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._audio_queue.put_nowait(audio)
                except queue.Full:
                    pass
        for line in self.parser.take_logs():
            sys.stdout.write(line)
            sys.stdout.flush()
            if "LINK PASS" in line:
                self.boot_passed = True
                self.boot_summary = "LINK PASS"
            elif "LINK FAIL" in line:
                self.boot_passed = False
                self.boot_summary = "LINK FAIL"

    def run(self) -> None:
        try:
            asyncio.run(self._loop())
        except Exception as exc:  # pylint: disable=broad-except
            self.last_error = str(exc)

    async def _loop(self) -> None:
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError:
            self.last_error = "bleak is not installed"
            return
        while not self._stop_event.is_set():
            try:
                device = await BleakScanner.find_device_by_name(BLE_NAME, timeout=8.0)
            except Exception as exc:  # pylint: disable=broad-except
                self.last_error = str(exc)
                await asyncio.sleep(2.0)
                continue
            if device is None:
                self.last_error = "Voyage is not advertising"
                self.boot_summary = "connecting"
                await asyncio.sleep(2.0)
                continue
            try:
                async with BleakClient(device) as client:
                    self.last_error = None
                    await client.start_notify(BLE_TX, self._on_notify)
                    while client.is_connected and not self._stop_event.is_set():
                        try:
                            payload = await asyncio.to_thread(self._write_q.get, True, 0.05)
                        except queue.Empty:
                            continue
                        mtu = getattr(client, "mtu_size", 23) or 23
                        step = max(20, min(mtu - 3, 244))
                        for off in range(0, len(payload), step):
                            await client.write_gatt_char(
                                BLE_RX, payload[off : off + step], response=False
                            )
            except Exception as exc:  # pylint: disable=broad-except
                self.last_error = str(exc)
                self.boot_passed = False
                self.boot_summary = "connecting"
                await asyncio.sleep(1.0)
