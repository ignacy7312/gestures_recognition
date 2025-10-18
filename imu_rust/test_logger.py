#!/usr/bin/env python3
import sys
import argparse
import math
import csv
import statistics
from typing import List, Tuple, Optional

import matplotlib
matplotlib.use("Agg")  # switch to non-interactive; we enable interactive only if --show
import matplotlib.pyplot as plt

def parse_args():
    p = argparse.ArgumentParser(description="Analyze BNO085 CSV stream and plot |a|, |ω| over time.")
    p.add_argument("--input", type=str, default="-", help="CSV file path or '-' for stdin (default '-')")
    p.add_argument("--expect-hz", type=float, default=100.0, help="Expected sampling rate in Hz (default 100)")
    p.add_argument("--show", action="store_true", help="Show plots interactively (requires GUI)")
    p.add_argument("--save-prefix", type=str, default=None, help="If set, save figures as '<prefix>_acc.png' and '<prefix>_gyro.png'")
    p.add_argument("--max-rows", type=int, default=None, help="Optional limit of rows to analyze")
    return p.parse_args()

def read_csv(path: str, max_rows: Optional[int] = None):
    fh = sys.stdin if path == "-" else open(path, "r", newline="")
    with fh:
        rdr = csv.reader(fh)
        header = next(rdr, None)
        # Accept either exact or extra columns; map indices by header names
        idx = {name: i for i, name in enumerate(header)} if header else {}
        def col(name, default_index):
            return idx.get(name, default_index)

        t_idx = col("t", 0)
        ax_idx = col("ax", 1); ay_idx = col("ay", 2); az_idx = col("az", 3)
        gx_idx = col("gx", 4); gy_idx = col("gy", 5); gz_idx = col("gz", 6)
        qw_idx = col("qw", 7); qi_idx = col("qi", 8); qj_idx = col("qj", 9); qk_idx = col("qk", 10)

        T=[]; AX=[]; AY=[]; AZ=[]; GX=[]; GY=[]; GZ=[]; QW=[]; QI=[]; QJ=[]; QK=[]
        count = 0
        for row in rdr:
            if not row or len(row) < 11:
                continue
            try:
                T.append(float(row[t_idx]))
                AX.append(float(row[ax_idx])); AY.append(float(row[ay_idx])); AZ.append(float(row[az_idx]))
                GX.append(float(row[gx_idx])); GY.append(float(row[gy_idx])); GZ.append(float(row[gz_idx]))
                QW.append(float(row[qw_idx])); QI.append(float(row[qi_idx])); QJ.append(float(row[qj_idx])); QK.append(float(row[qk_idx]))
                count += 1
                if max_rows is not None and count >= max_rows:
                    break
            except Exception:
                # Skip unparsable lines
                continue
        return T, AX, AY, AZ, GX, GY, GZ, QW, QI, QJ, QK

def analyze(T: List[float], AX, AY, AZ, GX, GY, GZ, expect_hz: float):
    if len(T) < 2:
        return {}

    # dt statistics
    dts = [t2 - t1 for t1, t2 in zip(T[:-1], T[1:]) if t2 >= t1]
    duration = (T[-1] - T[0]) if T[-1] >= T[0] else None
    eff_hz = (len(T) / duration) if duration and duration > 0 else (1.0 / statistics.mean(dts))

    mean_dt = statistics.mean(dts) if dts else float("nan")
    stdev_dt = statistics.pstdev(dts) if len(dts) > 1 else 0.0
    jitter_ms_mean = (mean_dt - (1.0/expect_hz)) * 1000.0
    jitter_ms_std = stdev_dt * 1000.0

    # magnitude
    amag = [math.sqrt(ax*ax + ay*ay + az*az) for ax,ay,az in zip(AX, AY, AZ)]
    gmag = [math.sqrt(gx*gx + gy*gy + gz*gz) for gx,gy,gz in zip(GX, GY, GZ)]

    # drop estimate: how many samples are "missing" relative to expect_hz over duration
    drops_pct = None
    if duration and duration > 0 and expect_hz > 0:
        expected = int(duration * expect_hz)
        if expected > 0:
            drops = max(0, expected - len(T))
            drops_pct = (drops / expected) * 100.0

    return {
        "duration": duration,
        "n": len(T),
        "eff_hz": eff_hz,
        "mean_dt": mean_dt,
        "stdev_dt": stdev_dt,
        "jitter_ms_mean": jitter_ms_mean,
        "jitter_ms_std": jitter_ms_std,
        "amag": amag,
        "gmag": gmag,
    }, dts

def main():
    args = parse_args()
    T, AX, AY, AZ, GX, GY, GZ, QW, QI, QJ, QK = read_csv(args.input, args.max_rows)
    stats, dts = analyze(T, AX, AY, AZ, GX, GY, GZ, args.expect_hz)

    if not stats:
        print("No data parsed.", file=sys.stderr)
        sys.exit(1)

    # Console summary
    print("=== IMU CSV Analysis ===")
    print(f"rows              : {stats['n']}")
    print(f"duration [s]      : {stats['duration']:.3f}")
    print(f"effective Hz      : {stats['eff_hz']:.2f}")
    print(f"mean dt [ms]      : {stats['mean_dt']*1000.0:.3f}")
    print(f"jitter mean [ms]  : {stats['jitter_ms_mean']:.3f}")
    print(f"jitter std  [ms]  : {stats['jitter_ms_std']:.3f}")
    # estimate drop percent versus expected
    if args.expect_hz > 0 and stats['duration'] and stats['duration'] > 0:
        expected = int(stats['duration'] * args.expect_hz)
        drops = max(0, expected - stats['n'])
        drops_pct = (drops / expected) * 100.0 if expected > 0 else 0.0
        print(f"drops (est)       : {drops} ({drops_pct:.2f}%)")

    # Plots
    # 1) |a| over time
    fig1 = plt.figure()
    plt.title("|a| over time")
    plt.xlabel("t [s]")
    plt.ylabel("|a| [m/s^2]")
    plt.plot(T, stats["amag"])
    if args.save_prefix:
        fig1.savefig(f"{args.save_prefix}_acc.png", dpi=150, bbox_inches="tight")

    # 2) |ω| over time
    fig2 = plt.figure()
    plt.title("|ω| over time")
    plt.xlabel("t [s]")
    plt.ylabel("|ω| [rad/s]")
    plt.plot(T, stats["gmag"])
    if args.save_prefix:
        fig2.savefig(f"{args.save_prefix}_gyro.png", dpi=150, bbox_inches="tight")

    if args.show:
        # Switch backend to interactive if available
        try:
            import matplotlib
            matplotlib.use("TkAgg", force=True)
        except Exception:
            pass
        plt.show()

if __name__ == "__main__":
    main()
