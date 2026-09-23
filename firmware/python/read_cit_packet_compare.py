# dw3000_cir_viewer.py
# Compare true vs estimated CIR magnitude from DW3000 I/Q frames

import time
import numpy as np
import matplotlib.pyplot as plt
import serial

# ===== params =====
PORT = "COM3"           # Set to your serial port, e.g. "/dev/ttyACM0"
BAUD = 115200
SAMPLES = 128
MAGIC = b"DW3C"
FRAME_TIMEOUT_S = None
# ==================


def sign_extend_24(x: np.ndarray) -> np.ndarray:
    x = x.astype(np.int32, copy=False)
    x[x & 0x800000 != 0] -= 1 << 24
    return x


def parse_cir(payload: bytes) -> tuple[np.ndarray, np.ndarray]:
    b = np.frombuffer(payload, np.uint8).reshape(SAMPLES, 6)

    Iu = b[:, 0].astype(np.uint32) | (b[:, 1].astype(np.uint32) << 8) | (b[:, 2].astype(np.uint32) << 16)
    Qu = b[:, 3].astype(np.uint32) | (b[:, 4].astype(np.uint32) << 8) | (b[:, 5].astype(np.uint32) << 16)

    return sign_extend_24(Iu), sign_extend_24(Qu)


def mag_true(I, Q):
    return np.sqrt(I.astype(np.float64)**2 + Q.astype(np.float64)**2)


def mag_est(I, Q):
    a = np.abs(I)
    b = np.abs(Q)
    mx = np.maximum(a, b)
    mn = np.minimum(a, b)
    return mx + (mn >> 2)


def read_exact(ser, n, t0):
    buf = bytearray()
    while len(buf) < n:
        c = ser.read(n - len(buf))
        if not c:
            if FRAME_TIMEOUT_S and time.time() - t0 > FRAME_TIMEOUT_S:
                raise TimeoutError
            continue
        buf.extend(c)
    return bytes(buf)


def read_frame(ser):
    t0 = time.time()
    sync = bytearray()

    while True:
        b = ser.read(1)
        if not b:
            continue
        sync += b
        if len(sync) > 4:
            sync.pop(0)
        if bytes(sync) == MAGIC:
            break

    length = int.from_bytes(read_exact(ser, 2, t0), "little")
    return read_exact(ser, length, t0)


class Stats:
    def __init__(self):
        self.n = 0
        self.abs_sum = 0.0
        self.sq_sum = 0.0
        self.abs_pct_sum = 0.0
        self.pct_n = 0

    def update(self, t, e):
        d = e - t
        a = np.abs(d)

        self.n += a.size
        self.abs_sum += a.sum()
        self.sq_sum += (d * d).sum()

        m = t > 0
        self.abs_pct_sum += (a[m] / t[m] * 100).sum()
        self.pct_n += m.sum()

    def report(self):
        print("\n=== Estimator accuracy summary ===")
        print(f"Samples: {self.n}")
        print(f"MAE:     {self.abs_sum / self.n:.3f}")
        print(f"RMSE:    {(self.sq_sum / self.n) ** 0.5:.3f}")
        if self.pct_n:
            print(f"MAPE:    {self.abs_pct_sum / self.pct_n:.3f} %")


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Opened {PORT} @ {BAUD}, waiting for {MAGIC!r}")

    stats = Stats()

    plt.ion()
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)

    l1, = ax1.plot([])
    l2, = ax2.plot([])
    l3, = ax3.plot([])

    ax1.set_title("True magnitude")
    ax2.set_title("Estimated magnitude (max + min>>2)")
    ax3.set_title("Error (est - true)")
    ax3.set_xlabel("Sample")

    x = np.arange(SAMPLES)

    try:
        while True:
            payload = read_frame(ser)
            I, Q = parse_cir(payload)

            t = mag_true(I, Q)
            e = mag_est(I, Q)

            stats.update(t, e)

            l1.set_data(x, t)
            l2.set_data(x, e)
            l3.set_data(x, e - t)

            for ax in (ax1, ax2, ax3):
                ax.relim()
                ax.autoscale_view()

            plt.pause(0.001)

    except KeyboardInterrupt:
        stats.report()


if __name__ == '__main__':
    main()
