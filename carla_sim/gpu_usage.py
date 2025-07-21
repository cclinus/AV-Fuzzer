#!/usr/bin/env python3
import os
import subprocess
import time
import argparse
from collections import deque

def ensure_smi_on_path():
    nvsi = r"C:\Program Files\NVIDIA Corporation\NVSMI"
    if os.name == 'nt' and os.path.isdir(nvsi):
        os.environ["PATH"] += os.pathsep + nvsi

def query_smi():

    cmd = [
        "nvidia-smi",
        "--query-gpu=utilization.gpu,memory.used,memory.total",
        "--format=csv,noheader,nounits"
    ]
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        raise RuntimeError(f"nvidia-smi failed: {p.stderr.strip()}")
    lines = p.stdout.strip().splitlines()
    stats = []
    for line in lines:
        util, used, total = [float(x) for x in line.split(",")]
        stats.append((util, used, total))
    return stats

def monitor(interval: float, window: float):
    ensure_smi_on_path()
    buffers = []  # one deque per GPU
    first = query_smi()
    gpu_count = len(first)
    buffers = [deque() for _ in range(gpu_count)]

    start = time.time()
    try:
        while True:
            now = time.time()
            readings = query_smi()
            # append to buffers and prune old
            for i, (util, used, total) in enumerate(readings):
                buf = buffers[i]
                buf.append((now, util, used))
                # drop samples older than window
                while buf and (now - buf[0][0] > window):
                    buf.popleft()

            # compute averages
            parts = []
            for i, buf in enumerate(buffers):
                if buf:
                    avg_util = sum(x[1] for x in buf) / len(buf)
                    avg_mem  = sum(x[2] for x in buf) / len(buf)
                    parts.append(
                        f"GPU{i}: avg util {avg_util:.1f}%  avg mem {avg_mem:.1f} MiB"
                    )
                else:
                    parts.append(f"GPU{i}: no data")
            print(f"[rolling {window:.0f}s]  " + "   ".join(parts))

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\nInterrupted by user, shutting down.")

if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description="Rolling-average GPU monitor via nvidia-smi"
    )
    p.add_argument(
        "-i","--interval", type=float, default=1.0,
        help="sample every N seconds (default: 1.0)"
    )
    p.add_argument(
        "-w","--window", type=float, default=15.0,
        help="rolling window in seconds (default: 15.0)"
    )
    args = p.parse_args()
    monitor(args.interval, args.window)
