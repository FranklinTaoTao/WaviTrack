# dw3000_cir_viewer.py
# Live viewer for DW3000 mag16 frames over serial.
# Protocol:
#   magic 4B: b"DW3M"
#   len   2B: little-endian payload length
#   data     : samples * 2 bytes, uint16 little-endian

import time
import numpy as np
import matplotlib.pyplot as plt
import serial

# ====== params ======
PORT = "COM3"           # Set to your serial port, e.g. "/dev/ttyACM0"
BAUD = 115200
SAMPLES = 128
MAGIC = b"DW3M"
FRAME_TIMEOUT_S = None   # None = wait forever, or set e.g. 2.0
SAVE_CSV = None          # e.g. "mag16.csv" or None
LIVE = True              # True = live plot, False = one-shot plot
# ====================


def read_exact(ser: serial.Serial, n: int, t0: float | None, timeout_s: float | None) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            if timeout_s is not None and t0 is not None and (time.time() - t0) > timeout_s:
                raise TimeoutError("Timeout while reading frame")
            continue
        buf.extend(chunk)
    return bytes(buf)


def read_frame(ser: serial.Serial, timeout_s: float | None) -> bytes:
    t0 = time.time() if timeout_s is not None else None

    # sync on MAGIC
    sync = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            if timeout_s is not None and (time.time() - t0) > timeout_s:
                raise TimeoutError("Timeout waiting for magic")
            continue
        sync += b
        if len(sync) > 4:
            sync.pop(0)
        if bytes(sync) == MAGIC:
            break

    length = int.from_bytes(read_exact(ser, 2, t0, timeout_s), "little")
    return read_exact(ser, length, t0, timeout_s)


def parse_mag16(payload: bytes, samples: int) -> np.ndarray:
    exp = samples * 2
    if len(payload) != exp:
        raise ValueError(f"payload {len(payload)} != expected {exp}")
    return np.frombuffer(payload, dtype="<u2").copy()


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Opened {PORT} @ {BAUD}. Waiting for {MAGIC!r} frames...")

    if LIVE:
        plt.ion()
        fig, ax = plt.subplots()
        line, = ax.plot([], [])
        ax.set_title("DW3000 CIR Magnitude (mag16)")
        ax.set_xlabel("Sample")
        ax.set_ylabel("mag16")

        while True:
            try:
                mag16 = parse_mag16(read_frame(ser, FRAME_TIMEOUT_S), SAMPLES)

                x = np.arange(SAMPLES)
                line.set_data(x, mag16)
                ax.relim()
                ax.autoscale_view()
                plt.pause(0.001)

                if SAVE_CSV:
                    np.savetxt(SAVE_CSV, mag16, delimiter=",", header="mag16", comments="")
            except (TimeoutError, ValueError) as e:
                print("Frame error:", e)

    else:
        mag16 = parse_mag16(read_frame(ser, FRAME_TIMEOUT_S), SAMPLES)
        print(mag16)

        if SAVE_CSV:
            np.savetxt(SAVE_CSV, mag16, delimiter=",", header="mag16", comments="")
            print(f"Saved to {SAVE_CSV}")

        plt.figure()
        plt.plot(np.arange(SAMPLES), mag16)
        plt.title("DW3000 CIR Magnitude (mag16)")
        plt.xlabel("Sample")
        plt.ylabel("mag16")
        plt.show()


if __name__ == "__main__":
    main()
