#!/usr/bin/env python3
"""
Receive WaviTrack UDP packets, assemble per-round data, save to NPZ, and show live loss table.

Expected firmware side:
- num_nodes = 6
- UDP ports: UDP_PORT_BASE + node_idx (default 20000..20005)
- round_id is uint16 in both UWB and UDP packet headers

NPZ outputs (first dimension = n_frames):
- round_id:        (n_frames,)
- accel:           (n_frames, 6, 3)
- gyro:            (n_frames, 6, 3)
- quat:            (n_frames, 6, 4)
- quat_accuracy:   (n_frames, 6)
- uwb_error_flags: (n_frames, 6)
- imu_error_flags: (n_frames, 6)
- imu_i2c_ms:      (n_frames, 6)
- ranging:         (n_frames, 15)       # pairs in pair_nodes
- cir:             (n_frames, 15, 96)   # pairs in pair_nodes
- node_status:     (n_frames, 6)        # 0=missing, 1=partial, 2=ok
- pair_nodes:      (15, 2)              # (i, j), i < j
"""

from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import select
import socket
import struct
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np

# -------------------- Packet constants --------------------
UDP_MAGIC = int.from_bytes(b"UWBM", "little")
IMU_MAGIC = int.from_bytes(b"IMU!", "little")

# data_collection.ino packed UDP header
# uint32 magic
# uint64 timestamp_us
# uint32 frame_no
# uint16 round_id
# uint8 node_idx
# uint8 num_nodes
# uint8 max_sender_seen
# uint32 error_flags
# uint8 dist_count
# uint8 cir_count
# uint8 dist_valid_mask
# uint8 cir_valid_mask
UDP_HDR_FMT = "<IQIHBBBI4B"
UDP_HDR_SIZE = struct.calcsize(UDP_HDR_FMT)  # 29 bytes

# IMU tail header in data_collection.ino
# uint32 imu_magic
# uint32 imu_error_flags
# uint16 i2c_ms_f16
# uint16 reserved
IMU_TAIL_HDR_FMT = "<IIHH"
IMU_TAIL_HDR_SIZE = struct.calcsize(IMU_TAIL_HDR_FMT)  # 12 bytes

IMU_FLOAT_COUNT = 11  # ax,ay,az,gx,gy,gz,qx,qy,qz,qw,quat_accuracy
IMU_FLOAT_BYTES = IMU_FLOAT_COUNT * 4

CIR_SAMPLES = 96

# Status codes for live table
STATUS_MISSING = 0
STATUS_PARTIAL = 1
STATUS_OK = 2

# UWB firmware error bits from data_collection.ino
UWB_ERROR_BIT_NAMES: Dict[int, str] = {
    0: "ERR_ROUND_TIMEOUT",
    1: "ERR_POLL_TX_START_FAIL",
    2: "ERR_POLL_TXFRS_TIMEOUT",
    3: "ERR_NODE_TX_START_FAIL",
    4: "ERR_NODE_TXFRS_TIMEOUT",
    5: "ERR_RX_FRAME_TOO_LONG",
    6: "ERR_RX_APP_CODE_MISMATCH",
    7: "ERR_RX_STALE_ROUND",
    8: "ERR_RX_UNKNOWN_MSG_TYPE",
    9: "ERR_IRQ_UNEXPECTED_STATUS",
    10: "ERR_WIFI_NOT_CONNECTED",
    11: "ERR_UDP_SEND_FAIL",
}

UWB_BITS = tuple(range(0, 10))
TRANSPORT_BITS = (10, 11)


@dataclasses.dataclass
class PacketData:
  round_id: int
  node_idx: int
  num_nodes: int
  frame_no: int
  timestamp_us: int
  max_sender_seen: int
  error_flags: int
  dist_count: int
  cir_count: int
  dist_valid_mask: int
  cir_valid_mask: int
  imu_error_flags: int
  imu_i2c_ms: float
  accel: np.ndarray  # (3,)
  gyro: np.ndarray   # (3,)
  quat: np.ndarray   # (4,)
  quat_accuracy: float
  distances: np.ndarray  # (dist_count,)
  cir: np.ndarray        # (cir_count, 96)
  parse_ok: bool


@dataclasses.dataclass
class FrameBucket:
  round_id: int
  first_rx_time: float
  packets: Dict[int, PacketData]


def parse_packet(payload: bytes) -> Optional[PacketData]:
  if len(payload) < UDP_HDR_SIZE:
    return None

  (
      magic,
      timestamp_us,
      frame_no,
      round_id,
      node_idx,
      num_nodes,
      max_sender_seen,
      error_flags,
      dist_count,
      cir_count,
      dist_valid_mask,
      cir_valid_mask,
  ) = struct.unpack_from(UDP_HDR_FMT, payload, 0)

  if magic != UDP_MAGIC:
    return None

  offset = UDP_HDR_SIZE
  dist_bytes = dist_count * 4
  cir_bytes = cir_count * CIR_SAMPLES * 2
  min_needed = offset + dist_bytes + cir_bytes + IMU_TAIL_HDR_SIZE + IMU_FLOAT_BYTES

  # Keep header info even if payload is truncated so we can mark this packet as partial.
  parse_ok = len(payload) >= min_needed

  distances = np.empty((0,), dtype=np.float32)
  cir = np.empty((0, CIR_SAMPLES), dtype=np.float32)
  imu_error_flags = 0
  imu_i2c_ms = np.nan
  accel = np.full((3,), np.nan, dtype=np.float32)
  gyro = np.full((3,), np.nan, dtype=np.float32)
  quat = np.full((4,), np.nan, dtype=np.float32)
  quat_accuracy = np.nan

  if parse_ok:
    if dist_count > 0:
      distances = np.frombuffer(payload, dtype="<f4", count=dist_count, offset=offset).copy()
    offset += dist_bytes

    if cir_count > 0:
      cir_raw = np.frombuffer(payload, dtype="<u2", count=cir_count * CIR_SAMPLES, offset=offset).copy()
      cir = cir_raw.reshape(cir_count, CIR_SAMPLES).astype(np.float32)
    offset += cir_bytes

    imu_magic, imu_error_flags, i2c_ms_f16_bits, _reserved = struct.unpack_from(IMU_TAIL_HDR_FMT, payload, offset)
    offset += IMU_TAIL_HDR_SIZE

    if imu_magic != IMU_MAGIC:
      parse_ok = False
    else:
      i2c_ms_u16 = np.array([i2c_ms_f16_bits], dtype=np.uint16)
      imu_i2c_ms = i2c_ms_u16.view(np.float16).astype(np.float32)[0].item()

      imu_vals = np.frombuffer(payload, dtype="<f4", count=IMU_FLOAT_COUNT, offset=offset).copy()
      # ax,ay,az,gx,gy,gz,qx,qy,qz,qw,quat_accuracy
      accel = imu_vals[0:3]
      gyro = imu_vals[3:6]
      quat = imu_vals[6:10]
      quat_accuracy = float(imu_vals[10])

  return PacketData(
      round_id=int(round_id),
      node_idx=int(node_idx),
      num_nodes=int(num_nodes),
      frame_no=int(frame_no),
      timestamp_us=int(timestamp_us),
      max_sender_seen=int(max_sender_seen),
      error_flags=int(error_flags),
      dist_count=int(dist_count),
      cir_count=int(cir_count),
      dist_valid_mask=int(dist_valid_mask),
      cir_valid_mask=int(cir_valid_mask),
      imu_error_flags=int(imu_error_flags),
      imu_i2c_ms=float(imu_i2c_ms),
      accel=accel,
      gyro=gyro,
      quat=quat,
      quat_accuracy=quat_accuracy,
      distances=distances,
      cir=cir,
      parse_ok=parse_ok,
  )


def build_pairs(num_nodes: int) -> List[Tuple[int, int]]:
  pairs: List[Tuple[int, int]] = []
  for i in range(num_nodes):
    for j in range(i + 1, num_nodes):
      pairs.append((i, j))
  return pairs


def expected_masks(node_idx: int, num_nodes: int) -> Tuple[int, int, int, int]:
  exp_dist_count = max(0, num_nodes - node_idx - 1)
  exp_cir_count = node_idx

  exp_dist_mask = 0
  for j in range(node_idx + 1, num_nodes):
    if j < 8:
      exp_dist_mask |= (1 << j)

  exp_cir_mask = 0
  for j in range(node_idx):
    if j < 8:
      exp_cir_mask |= (1 << j)

  return exp_dist_count, exp_cir_count, exp_dist_mask, exp_cir_mask


def packet_status(pkt: PacketData, num_nodes: int) -> int:
  if not pkt.parse_ok:
    return STATUS_PARTIAL

  exp_dist_count, exp_cir_count, exp_dist_mask, exp_cir_mask = expected_masks(pkt.node_idx, num_nodes)

  counts_ok = (pkt.dist_count == exp_dist_count) and (pkt.cir_count == exp_cir_count)
  masks_ok = ((pkt.dist_valid_mask & exp_dist_mask) == exp_dist_mask) and ((pkt.cir_valid_mask & exp_cir_mask) == exp_cir_mask)
  no_err = (pkt.error_flags == 0) and (pkt.imu_error_flags == 0)

  if counts_ok and masks_ok and no_err:
    return STATUS_OK
  return STATUS_PARTIAL


def forward_round_distance(prev_round: int, next_round: int) -> int:
  return (next_round - prev_round) & 0xFFFF


def decode_error_bit_totals(error_flags_arr: np.ndarray, bits: Tuple[int, ...]) -> Dict[int, int]:
  totals: Dict[int, int] = {}
  if error_flags_arr.size == 0:
    for bit in bits:
      totals[bit] = 0
    return totals
  v = error_flags_arr.astype(np.uint32, copy=False).reshape(-1)
  for bit in bits:
    totals[bit] = int(np.count_nonzero((v & (np.uint32(1) << np.uint32(bit))) != 0))
  return totals


def format_bit_totals(bit_totals: Dict[int, int]) -> str:
  parts: List[str] = []
  for bit, count in bit_totals.items():
    if count <= 0:
      continue
    name = UWB_ERROR_BIT_NAMES.get(bit, f"BIT_{bit}")
    parts.append(f"{name}={count}")
  if not parts:
    return "none"
  return ", ".join(parts)


class LiveStatusTable:
  def __init__(self, num_nodes: int, window_frames: int) -> None:
    self.num_nodes = num_nodes
    self.window_frames = window_frames
    self.round_ids: deque[int] = deque(maxlen=window_frames)
    self.status_rows: deque[np.ndarray] = deque(maxlen=window_frames)  # each shape (num_nodes,)

    plt.ion()
    self.fig, self.ax = plt.subplots(figsize=(12, 4))
    self.last_draw = 0.0

  def append(self, round_id: int, status_row: np.ndarray) -> None:
    self.round_ids.append(int(round_id))
    self.status_rows.append(status_row.astype(np.uint8, copy=True))

  def draw(self, force: bool = False) -> None:
    now = time.time()
    if not force and (now - self.last_draw) < 0.1:
      return
    self.last_draw = now

    self.ax.clear()

    n = len(self.round_ids)
    if n == 0:
      self.ax.set_title("Waiting for packets...")
      self.fig.canvas.draw_idle()
      plt.pause(0.001)
      return

    mat = np.array(self.status_rows, dtype=np.uint8).T  # (num_nodes, n_frames)
    # Draw dots for each status
    for status_code, color in ((STATUS_MISSING, "red"), (STATUS_PARTIAL, "yellow"), (STATUS_OK, "green")):
      ys, xs = np.where(mat == status_code)
      if xs.size > 0:
        self.ax.scatter(xs, ys, c=color, s=28, marker="o")

    self.ax.set_ylim(self.num_nodes - 0.5, -0.5)
    self.ax.set_xlim(-0.5, n - 0.5)
    self.ax.set_yticks(np.arange(self.num_nodes))
    self.ax.set_ylabel("Node")
    self.ax.set_xlabel("Frame (round_id)")
    self.ax.set_title("Live Frame Completeness: red=missing, yellow=partial, green=ok")
    self.ax.grid(True, which="both", color="0.85", linewidth=0.5)

    # X labels as round_id (subsampled)
    step = max(1, n // 12)
    ticks = np.arange(0, n, step, dtype=int)
    labels = [str(self.round_ids[i]) for i in ticks]
    self.ax.set_xticks(ticks)
    self.ax.set_xticklabels(labels, rotation=45, ha="right")

    self.fig.tight_layout()
    self.fig.canvas.draw_idle()
    plt.pause(0.001)


def make_empty_frame_row(
    num_nodes: int, num_pairs: int
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
  status = np.full((num_nodes,), STATUS_MISSING, dtype=np.uint8)
  ranging = np.full((num_pairs,), np.nan, dtype=np.float32)
  cir = np.full((num_pairs, CIR_SAMPLES), np.nan, dtype=np.float32)
  accel = np.full((num_nodes, 3), np.nan, dtype=np.float32)
  gyro = np.full((num_nodes, 3), np.nan, dtype=np.float32)
  quat = np.full((num_nodes, 4), np.nan, dtype=np.float32)
  quat_acc = np.full((num_nodes,), np.nan, dtype=np.float32)
  imu_error_flags = np.zeros((num_nodes,), dtype=np.uint32)
  imu_i2c_ms = np.full((num_nodes,), np.nan, dtype=np.float32)
  return status, ranging, cir, accel, gyro, quat, quat_acc, imu_error_flags, imu_i2c_ms


def assemble_frame(
    frame: FrameBucket,
    num_nodes: int,
    pairs: List[Tuple[int, int]],
    pair_to_idx: Dict[Tuple[int, int], int],
 ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
  num_pairs = len(pairs)
  status, ranging, cir, accel, gyro, quat, quat_acc, imu_error_flags, imu_i2c_ms = make_empty_frame_row(num_nodes, num_pairs)

  # Node status table
  for node in range(num_nodes):
    pkt = frame.packets.get(node)
    if pkt is None:
      status[node] = STATUS_MISSING
    else:
      status[node] = packet_status(pkt, num_nodes)

  # Ranging values: pair(i,j) is sent by node i at index (j-i-1)
  for i, j in pairs:
    k = pair_to_idx[(i, j)]
    pkt = frame.packets.get(i)
    if pkt is None or (not pkt.parse_ok):
      continue
    dist_idx = j - i - 1
    if 0 <= dist_idx < pkt.distances.shape[0]:
      ranging[k] = pkt.distances[dist_idx]

  # CIR values: pair(i,j) with i<j is in node j packet CIR block i
  for i, j in pairs:
    k = pair_to_idx[(i, j)]
    pkt = frame.packets.get(j)
    if pkt is None or (not pkt.parse_ok):
      continue
    if i < pkt.cir.shape[0]:
      cir[k, :] = pkt.cir[i, :]

  # IMU data from each node packet
  for node in range(num_nodes):
    imu_pkt = frame.packets.get(node)
    if imu_pkt is None:
      continue
    imu_error_flags[node] = np.uint32(imu_pkt.imu_error_flags)
    imu_i2c_ms[node] = np.float32(imu_pkt.imu_i2c_ms)
    if imu_pkt.parse_ok:
      accel[node, :] = imu_pkt.accel.astype(np.float32, copy=False)
      gyro[node, :] = imu_pkt.gyro.astype(np.float32, copy=False)
      quat[node, :] = imu_pkt.quat.astype(np.float32, copy=False)
      quat_acc[node] = np.float32(imu_pkt.quat_accuracy)

  return status, ranging, cir, accel, gyro, quat, quat_acc, imu_error_flags, imu_i2c_ms


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Capture mesh UDP and save NPZ with live loss table.")
  parser.add_argument("--bind-ip", default="0.0.0.0", help="IP to bind receiver sockets.")
  parser.add_argument("--udp-port-base", type=int, default=20000, help="Base UDP port (node i uses base+i).")
  parser.add_argument("--num-nodes", type=int, default=6, help="Node count. Safe default is 6.")
  parser.add_argument("--frame-timeout", type=float, default=0.25, help="Seconds to wait before finalizing incomplete frame.")
  parser.add_argument("--window-frames", type=int, default=240, help="Live table history width.")
  parser.add_argument("--duration", type=float, default=0.0, help="Capture duration in seconds (0 = run until Ctrl+C).")
  parser.add_argument("--output", default="", help="Output .npz path. Default: timestamped file in current folder.")
  parser.add_argument("--no-plot", action="store_true", help="Disable live matplotlib table.")
  return parser.parse_args()


def main() -> None:
  args = parse_args()

  if args.num_nodes != 6:
    raise ValueError("This script is currently configured for 6 nodes (15 pairs).")

  pairs = build_pairs(args.num_nodes)
  if len(pairs) != 15:
    raise ValueError("Expected 15 pairs for 6 nodes.")
  pair_to_idx = {p: i for i, p in enumerate(pairs)}

  # Round-id consistency check note:
  # Firmware behavior is consistent: node 0 increments round_id and sends POLL with it.
  # Other nodes adopt that POLL round_id and include the same value in NODE_TX and UDP.
  print("[info] round_id is expected to be consistent across all nodes per frame (from node 0 poll).")

  # Open one socket per node port, matching firmware destination ports.
  sockets: List[socket.socket] = []
  for node in range(args.num_nodes):
    port = args.udp_port_base + node
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((args.bind_ip, port))
    s.setblocking(False)
    sockets.append(s)
    print(f"[listen] {args.bind_ip}:{port}")

  live = None if args.no_plot else LiveStatusTable(args.num_nodes, args.window_frames)

  # Pending frames by round_id
  pending: Dict[int, FrameBucket] = {}

  # Output accumulators
  out_round_id: List[int] = []
  out_status: List[np.ndarray] = []
  out_ranging: List[np.ndarray] = []
  out_cir: List[np.ndarray] = []
  out_accel: List[np.ndarray] = []
  out_gyro: List[np.ndarray] = []
  out_quat: List[np.ndarray] = []
  out_quat_acc: List[np.ndarray] = []
  out_uwb_error_flags: List[np.ndarray] = []
  out_imu_error_flags: List[np.ndarray] = []
  out_imu_i2c_ms: List[np.ndarray] = []

  # Counters
  pkt_rx = 0
  pkt_parse_fail = 0
  pkt_bad_magic = 0
  frame_missing_synth = 0
  pending_stale_drop = 0
  last_committed_round: Optional[int] = None
  last_live_err_print = time.time()

  def append_synthetic_missing(round_id: int) -> None:
    nonlocal frame_missing_synth, last_committed_round
    status, ranging, cir, accel, gyro, quat, quat_acc, imu_error_flags, imu_i2c_ms = make_empty_frame_row(
        args.num_nodes, len(pairs)
    )
    out_round_id.append(round_id)
    out_status.append(status)
    out_ranging.append(ranging)
    out_cir.append(cir)
    out_accel.append(accel)
    out_gyro.append(gyro)
    out_quat.append(quat)
    out_quat_acc.append(quat_acc)
    out_uwb_error_flags.append(np.zeros((args.num_nodes,), dtype=np.uint32))
    out_imu_error_flags.append(imu_error_flags)
    out_imu_i2c_ms.append(imu_i2c_ms)
    frame_missing_synth += 1
    if live is not None:
      live.append(round_id, status)
    last_committed_round = round_id

  def append_real_frame(frame: FrameBucket) -> None:
    nonlocal last_committed_round
    status, ranging, cir, accel, gyro, quat, quat_acc, imu_error_flags, imu_i2c_ms = assemble_frame(
        frame, args.num_nodes, pairs, pair_to_idx
    )
    out_round_id.append(frame.round_id)
    out_status.append(status)
    out_ranging.append(ranging)
    out_cir.append(cir)
    out_accel.append(accel)
    out_gyro.append(gyro)
    out_quat.append(quat)
    out_quat_acc.append(quat_acc)
    out_uwb_error_flags.append(np.array([frame.packets.get(node).error_flags if frame.packets.get(node) is not None else 0 for node in range(args.num_nodes)], dtype=np.uint32))
    out_imu_error_flags.append(imu_error_flags)
    out_imu_i2c_ms.append(imu_i2c_ms)
    if live is not None:
      live.append(frame.round_id, status)
    last_committed_round = frame.round_id

  def frame_ready(fb: FrameBucket, now: float) -> bool:
    if len(fb.packets) >= args.num_nodes:
      return True
    return (now - fb.first_rx_time) >= args.frame_timeout

  def drop_stale_pending() -> None:
    nonlocal pending_stale_drop
    if last_committed_round is None:
      return
    stale_ids: List[int] = []
    for rid in pending.keys():
      d = forward_round_distance(last_committed_round, rid)
      # d==0: duplicate of committed frame, d>=32768: older/out-of-order.
      if d == 0 or d >= 32768:
        stale_ids.append(rid)
    for rid in stale_ids:
      pending.pop(rid, None)
      pending_stale_drop += 1

  def pick_next_action(now: float) -> Optional[Tuple[str, int]]:
    """
    Returns:
      ("real", round_id)      -> commit a real pending frame
      ("synthetic", round_id) -> insert a missing frame for expected round_id
      None                    -> wait for more packets/time
    """
    if not pending:
      return None

    if last_committed_round is None:
      # Bootstrap timeline from the earliest ready frame.
      ready_ids = [rid for rid, fb in pending.items() if frame_ready(fb, now)]
      if not ready_ids:
        return None
      rid = min(ready_ids, key=lambda r: pending[r].first_rx_time)
      return ("real", rid)

    expected = (last_committed_round + 1) & 0xFFFF
    expected_fb = pending.get(expected)

    # Strict in-order commit: only accept expected next round.
    if expected_fb is not None:
      if frame_ready(expected_fb, now):
        return ("real", expected)
      return None

    # If expected round has not appeared but future rounds have,
    # wait one timeout window from earliest future evidence, then
    # synthesize one missing expected frame.
    future_ids: List[int] = []
    future_first_rx_times: List[float] = []
    for rid, fb in pending.items():
      d = forward_round_distance(last_committed_round, rid)
      if 2 <= d < 32768:
        future_ids.append(rid)
        future_first_rx_times.append(fb.first_rx_time)

    if not future_ids:
      return None

    earliest_future_rx = min(future_first_rx_times)
    if (now - earliest_future_rx) >= args.frame_timeout:
      return ("synthetic", expected)

    return None

  start_t = time.time()
  print("[run] capturing... press Ctrl+C to stop")

  try:
    while True:
      now = time.time()
      if args.duration > 0 and (now - start_t) >= args.duration:
        break

      readable, _, _ = select.select(sockets, [], [], 0.05)
      for s in readable:
        try:
          payload, _addr = s.recvfrom(4096)
        except BlockingIOError:
          continue

        pkt = parse_packet(payload)
        if pkt is None:
          # Different traffic or bad header/magic.
          if len(payload) >= 4 and payload[:4] != b"UWBM":
            pkt_bad_magic += 1
          else:
            pkt_parse_fail += 1
          continue

        pkt_rx += 1

        if not (0 <= pkt.node_idx < args.num_nodes):
          pkt_parse_fail += 1
          continue

        if pkt.num_nodes != args.num_nodes:
          # Keep packet, but status will likely go partial.
          pass

        fb = pending.get(pkt.round_id)
        if fb is None:
          fb = FrameBucket(round_id=pkt.round_id, first_rx_time=now, packets={})
          pending[pkt.round_id] = fb

        # One packet per node per round expected. Replace if duplicate arrives.
        fb.packets[pkt.node_idx] = pkt

      # Commit in strict sequence (expected round only), so we do not
      # over-create missing frames from reordering.
      while True:
        drop_stale_pending()
        action = pick_next_action(time.time())
        if action is None:
          break
        mode, rid = action
        if mode == "real":
          fb = pending.pop(rid, None)
          if fb is None:
            break
          append_real_frame(fb)
        elif mode == "synthetic":
          append_synthetic_missing(rid)
        else:
          break

      if live is not None:
        live.draw(force=False)

      now_live = time.time()
      if (now_live - last_live_err_print) >= 5.0:
        if len(out_uwb_error_flags) > 0:
          live_uwb_flags = np.stack(out_uwb_error_flags, axis=0).astype(np.uint32, copy=False)
          uwb_totals = decode_error_bit_totals(live_uwb_flags, UWB_BITS)
          tx_totals = decode_error_bit_totals(live_uwb_flags, TRANSPORT_BITS)
          print(
              "[live-errors] "
              f"uwb({format_bit_totals(uwb_totals)}); "
              f"transport({format_bit_totals(tx_totals)}); "
              f"rx(parse_fail={pkt_parse_fail}, bad_magic={pkt_bad_magic}, "
              f"synthetic={frame_missing_synth}, stale_drop={pending_stale_drop})"
          )
        last_live_err_print = now_live

  except KeyboardInterrupt:
    print("\n[stop] Ctrl+C received")
  finally:
    # Flush pending buckets using same strict-order logic.
    flush_now = time.time() + args.frame_timeout + 1.0
    while pending:
      drop_stale_pending()
      action = pick_next_action(flush_now)
      if action is None:
        # Nothing actionable remains (likely stale/out-of-order leftovers).
        # Drop oldest remaining bucket to avoid hanging flush.
        oldest_rid = min(pending.keys(), key=lambda r: pending[r].first_rx_time)
        pending.pop(oldest_rid, None)
        pending_stale_drop += 1
        continue

      mode, rid = action
      if mode == "real":
        fb = pending.pop(rid, None)
        if fb is not None:
          append_real_frame(fb)
      elif mode == "synthetic":
        append_synthetic_missing(rid)

    if live is not None:
      live.draw(force=True)

    # Build final arrays
    n_frames = len(out_round_id)
    if n_frames == 0:
      round_arr = np.empty((0,), dtype=np.uint16)
      accel_arr = np.empty((0, args.num_nodes, 3), dtype=np.float32)
      gyro_arr = np.empty((0, args.num_nodes, 3), dtype=np.float32)
      quat_arr = np.empty((0, args.num_nodes, 4), dtype=np.float32)
      quat_acc_arr = np.empty((0, args.num_nodes), dtype=np.float32)
      uwb_error_flags_arr = np.empty((0, args.num_nodes), dtype=np.uint32)
      imu_error_flags_arr = np.empty((0, args.num_nodes), dtype=np.uint32)
      imu_i2c_ms_arr = np.empty((0, args.num_nodes), dtype=np.float32)
      ranging_arr = np.empty((0, len(pairs)), dtype=np.float32)
      cir_arr = np.empty((0, len(pairs), CIR_SAMPLES), dtype=np.float32)
      status_arr = np.empty((0, args.num_nodes), dtype=np.uint8)
    else:
      round_arr = np.asarray(out_round_id, dtype=np.uint16)
      accel_arr = np.stack(out_accel, axis=0).astype(np.float32, copy=False)
      gyro_arr = np.stack(out_gyro, axis=0).astype(np.float32, copy=False)
      quat_arr = np.stack(out_quat, axis=0).astype(np.float32, copy=False)
      quat_acc_arr = np.stack(out_quat_acc, axis=0).astype(np.float32, copy=False)
      uwb_error_flags_arr = np.stack(out_uwb_error_flags, axis=0).astype(np.uint32, copy=False)
      imu_error_flags_arr = np.stack(out_imu_error_flags, axis=0).astype(np.uint32, copy=False)
      imu_i2c_ms_arr = np.stack(out_imu_i2c_ms, axis=0).astype(np.float32, copy=False)
      ranging_arr = np.stack(out_ranging, axis=0).astype(np.float32, copy=False)
      cir_arr = np.stack(out_cir, axis=0).astype(np.float32, copy=False)
      status_arr = np.stack(out_status, axis=0).astype(np.uint8, copy=False)

    pair_nodes_arr = np.asarray(pairs, dtype=np.uint8)

    out_path = args.output.strip()
    if out_path == "":
      ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
      out_path = f"wavitrack_capture_{ts}.npz"

    np.savez_compressed(
        out_path,
        round_id=round_arr,
        accel=accel_arr,
        gyro=gyro_arr,
        quat=quat_arr,
        quat_accuracy=quat_acc_arr,
        uwb_error_flags=uwb_error_flags_arr,
        imu_error_flags=imu_error_flags_arr,
        imu_i2c_ms=imu_i2c_ms_arr,
        ranging=ranging_arr,
        cir=cir_arr,
        node_status=status_arr,
        pair_nodes=pair_nodes_arr,
        num_nodes=np.int32(args.num_nodes),
    )

    total_ok = int(np.sum(status_arr == STATUS_OK))
    total_partial = int(np.sum(status_arr == STATUS_PARTIAL))
    total_missing = int(np.sum(status_arr == STATUS_MISSING))

    print(f"[save] {out_path}")
    print(f"[stats] frames={n_frames}, packets_rx={pkt_rx}, parse_fail={pkt_parse_fail}, bad_magic={pkt_bad_magic}")
    print(f"[stats] synthetic_missing_frames={frame_missing_synth}")
    print(f"[stats] stale_pending_dropped={pending_stale_drop}")
    print(f"[stats] status counts -> ok={total_ok}, partial={total_partial}, missing={total_missing}")
    uwb_totals_final = decode_error_bit_totals(uwb_error_flags_arr, UWB_BITS)
    tx_totals_final = decode_error_bit_totals(uwb_error_flags_arr, TRANSPORT_BITS)
    print(f"[stats] sender uwb errors -> {format_bit_totals(uwb_totals_final)}")
    print(f"[stats] sender transport errors -> {format_bit_totals(tx_totals_final)}")
    print(
        "[stats] receiver udp/assembly -> "
        f"parse_fail={pkt_parse_fail}, bad_magic={pkt_bad_magic}, "
        f"synthetic_missing={frame_missing_synth}, stale_pending_dropped={pending_stale_drop}"
    )

    for s in sockets:
      s.close()


if __name__ == "__main__":
  main()
