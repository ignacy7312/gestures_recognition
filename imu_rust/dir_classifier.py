#!/usr/bin/env python3
import sys
import csv
import argparse
import math
from collections import deque

def parse_args():
    p = argparse.ArgumentParser(description="Direction classifier from BNO085 CSV (uses GRV + (Linear) Accel)")
    p.add_argument("--input", type=str, default="-", help="CSV path or '-' for stdin")
    p.add_argument("--acc-thr", type=float, default=1.5, help="Acceleration threshold [m/s^2] for gesture onset (default 1.5)")
    p.add_argument("--gyro-max", type=float, default=3.0, help="Ignore frames where |ω| > this [rad/s] (default 3.0)")
    p.add_argument("--win-ms", type=float, default=120.0, help="Window length for peak detection [ms] (default 120)")
    p.add_argument("--hz", type=float, default=100.0, help="Sampling rate used to size the window (default 100)")
    p.add_argument("--print-samples", action="store_true", help="Print per-sample world-frame accel components")
    return p.parse_args()

def quat_to_rot(qw, qx, qy, qz):
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if n == 0: n = 1.0
    w, x, y, z = qw/n, qx/n, qy/n, qz/n
    R = [[0]*3 for _ in range(3)]
    R[0][0] = 1 - 2*(y*y + z*z)
    R[0][1] = 2*(x*y - z*w)
    R[0][2] = 2*(x*z + y*w)
    R[1][0] = 2*(x*y + z*w)
    R[1][1] = 1 - 2*(x*x + z*z)
    R[1][2] = 2*(y*z - x*w)
    R[2][0] = 2*(x*z - y*w)
    R[2][1] = 2*(y*z + x*w)
    R[2][2] = 1 - 2*(x*x + y*y)
    return R

def rotate(R, v):
    return (
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    )

def classify(ax_w, ay_w, az_w, thr):
    comps = [abs(ax_w), abs(ay_w), abs(az_w)]
    idx = max(range(3), key=lambda i: comps[i])
    if comps[idx] < thr:
        return None

    # Diagonal if two components are strong
    sorted_comps = sorted(comps, reverse=True)
    if len(sorted_comps) >= 2 and sorted_comps[1] >= 0.7*sorted_comps[0] and sorted_comps[1] >= thr:
        dir_h = None
        if abs(ax_w) >= thr:
            dir_h = "right" if ax_w > 0 else "left"
        if abs(ay_w) >= thr and (dir_h is None or abs(ay_w) > abs(ax_w)):
            dir_h = "forward" if ay_w > 0 else "back"
        dir_v = None
        if abs(az_w) >= thr:
            dir_v = "up" if az_w > 0 else "down"
        if dir_h and dir_v:
            return f"diagonal {dir_v}-{dir_h}"

    if idx == 2:
        return "up" if az_w > 0 else "down"
    elif idx == 0:
        return "right" if ax_w > 0 else "left"
    else:
        return "forward" if ay_w > 0 else "back"

def main():
    args = parse_args()
    fh = sys.stdin if args.input == "-" else open(args.input, "r", newline="")
    with fh:
        rdr = csv.reader(fh)
        header = next(rdr, None)
        idx = {name: i for i, name in enumerate(header)} if header else {}
        def col(name, default_index):
            return idx.get(name, default_index)

        t_idx = col("t", 0)
        ax_idx = col("ax", 1); ay_idx = col("ay", 2); az_idx = col("az", 3)
        gx_idx = col("gx", 4); gy_idx = col("gy", 5); gz_idx = col("gz", 6)
        qw_idx = col("qw", 7); qi_idx = col("qi", 8); qj_idx = col("qj", 9); qk_idx = col("qk", 10)

        win_len = max(3, int((args.win_ms/1000.0) * args.hz))
        from collections import deque
        buf = deque(maxlen=win_len)

        last_label = None
        cool = 0

        print("# t, ax_w, ay_w, az_w, |a_w|, label")
        for row in rdr:
            if len(row) < 11:
                continue
            try:
                t = float(row[t_idx])
                ax = float(row[ax_idx]); ay = float(row[ay_idx]); az = float(row[az_idx])
                gx = float(row[gx_idx]); gy = float(row[gy_idx]); gz = float(row[gz_idx])
                qw = float(row[qw_idx]); qi = float(row[qi_idx]); qj = float(row[qj_idx]); qk = float(row[qk_idx])
            except Exception:
                continue

            gmag = math.sqrt(gx*gx + gy*gy + gz*gz)
            if gmag > args.gyro_max:
                buf.clear()
                if args.print_samples:
                    print(f"{t:.6f},,,,,# skip rotation |ω|={gmag:.2f}")
                continue

            R = quat_to_rot(qw, qi, qj, qk)
            ax_w, ay_w, az_w = rotate(R, (ax, ay, az))
            amag = math.sqrt(ax_w*ax_w + ay_w*ay_w + az_w*az_w)

            buf.append((t, ax_w, ay_w, az_w, amag))

            if args.print_samples:
                print(f"{t:.6f},{ax_w:.3f},{ay_w:.3f},{az_w:.3f},{amag:.3f},")

            if len(buf) >= win_len and cool == 0:
                tpk, xpk, ypk, zpk, _ = max(buf, key=lambda it: abs(it[1])+abs(it[2])+abs(it[3]))
                label = classify(xpk, ypk, zpk, args.acc_thr)
                if label and label != last_label:
                    print(f"{tpk:.6f},{xpk:.3f},{ypk:.3f},{zpk:.3f},,{label}")
                    last_label = label
                    cool = int(0.25 * args.hz)  # 250 ms cooldown

            if cool > 0:
                cool -= 1

if __name__ == "__main__":
    main()
