from __future__ import annotations

import argparse
import csv
import struct
import sys
import zlib

from _protocol import (
    CHIP_OF_TYPE,
    DATA_PACKET_SIZE,
    INNER_FRAME,
    INNER_FRAME_SIZE,
    LOG_MAX,
    MAGIC_BYTES,
    TYPE_LOG,
    find_esp_port,
    run_boot_test,
)

CHIP_NAME = {0: "EMG", 1: "EEG"}


def main() -> int:
    ap = argparse.ArgumentParser(description="Print the per-chip int32 samples exactly as they arrive over USB")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=2_000_000)
    ap.add_argument("--every", type=int, default=250, help="print every Nth packet per chip (1 = all)")
    ap.add_argument("--chip", choices=["emg", "eeg", "both"], default="both")
    ap.add_argument("--csv", default=None, help="also write every packet to this CSV")
    args = ap.parse_args()

    port = args.port or find_esp_port()
    if not port:
        print("no port; pass --port COM3")
        return 1
    boot = run_boot_test(port, args.baud)
    ser = boot.serial
    want = {"emg": {0}, "eeg": {1}, "both": {0, 1}}[args.chip]

    out = None
    if args.csv:
        f = open(args.csv, "w", newline="")
        out = csv.writer(f)
        out.writerow(["chip", "sample_index", "mcu_time_us"] + [f"ch{i}" for i in range(1, 9)])

    buf = bytearray()
    seen = {0: 0, 1: 0}
    bad = 0
    try:
        while True:
            buf.extend(ser.read(4096))
            while True:
                i = buf.find(MAGIC_BYTES)
                if i < 0:
                    del buf[:-3]
                    break
                del buf[:i]
                if len(buf) < 5:
                    break
                typ = buf[4]
                if typ in CHIP_OF_TYPE:
                    if len(buf) < DATA_PACKET_SIZE:
                        break
                    payload = bytes(buf[5:DATA_PACKET_SIZE])
                    _, _, _, chip, t_us, index, *chs, crc = INNER_FRAME.unpack(payload)
                    if (zlib.crc32(payload[:-4]) & 0xFFFFFFFF) != crc:
                        bad += 1
                        del buf[0]
                        continue
                    del buf[:DATA_PACKET_SIZE]
                    if chip not in want:
                        continue
                    if out:
                        out.writerow([CHIP_NAME[chip], index, t_us] + chs)
                    if seen[chip] % args.every == 0:
                        cells = " ".join(f"{v:+9d}/{v & 0xFFFFFF:06X}" for v in chs)
                        print(f"{CHIP_NAME[chip]} #{index:<8d} {cells}  crc_bad={bad}")
                    seen[chip] += 1
                elif typ == TYPE_LOG:
                    if len(buf) < 7:
                        break
                    n = struct.unpack_from("<H", buf, 5)[0]
                    if n > LOG_MAX:
                        del buf[0]
                        continue
                    if len(buf) < 7 + n:
                        break
                    sys.stdout.write(bytes(buf[7 : 7 + n]).decode("utf-8", "replace"))
                    del buf[: 7 + n]
                else:
                    del buf[0]
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if out:
            f.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
