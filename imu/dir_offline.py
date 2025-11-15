#!/usr/bin/env python3
"""
Offline'owy analizator kierunku ruchu dłoni na podstawie logów z imu_read_cpp.

Wejście: pliki CSV z kolumnami:
    t,ax,ay,az,gx,gy,gz,qw,qi,qj,qk

Metoda (wersja z opcją A):
1. Wczytujemy sygnał oraz kwaternion Game Rotation Vector.
2. Dla każdej próbki obracamy przyspieszenie z układu sensora do układu "świata":
       a_world = R(q) * a_sensor
3. Z pierwszych ~0.2 s liczymy bazowy wektor grawitacji a0_world.
4. Liczymy dynamiczne przyspieszenie: a_dyn = a_world - a0_world.
5. Znajdujemy indeks piku |a_dyn| (moment gestu).
6. Wycinamy okno [t_peak - 0.3s, t_peak + 0.3s].
7. Na tym oknie integrujemy a_dyn -> przybliżone Δv_world na każdej osi.
8. Dominująca oś + znak = kierunek gestu (UP/DOWN/LEFT/RIGHT/FORWARD/BACKWARD).
"""

import argparse
import csv
import math
import os
from typing import List, Tuple, Optional


# ---------- Wczytywanie danych ----------

def load_csv(path: str) -> Tuple[
    List[float],  # t
    List[float],  # ax
    List[float],  # ay
    List[float],  # az
    List[float],  # qw
    List[float],  # qi
    List[float],  # qj
    List[float],  # qk
]:
    """Wczytaj kolumny t, ax, ay, az, qw, qi, qj, qk z pliku CSV."""
    t_list: List[float] = []
    ax_list: List[float] = []
    ay_list: List[float] = []
    az_list: List[float] = []
    qw_list: List[float] = []
    qi_list: List[float] = []
    qj_list: List[float] = []
    qk_list: List[float] = []

    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            raise ValueError(f"{path}: pusty plik")

        # Zakładamy header jak z imu_read_cpp
        # t,ax,ay,az,gx,gy,gz,qw,qi,qj,qk
        try:
            idx_t = header.index("t")
            idx_ax = header.index("ax")
            idx_ay = header.index("ay")
            idx_az = header.index("az")
            idx_qw = header.index("qw")
            idx_qi = header.index("qi")
            idx_qj = header.index("qj")
            idx_qk = header.index("qk")
        except ValueError as e:
            raise ValueError(f"{path}: nie znaleziono wymaganych kolumn w headerze: {header}") from e

        for row in reader:
            if not row or len(row) <= idx_qk:
                continue
            try:
                t = float(row[idx_t])
                ax = float(row[idx_ax])
                ay = float(row[idx_ay])
                az = float(row[idx_az])
                qw = float(row[idx_qw])
                qi = float(row[idx_qi])
                qj = float(row[idx_qj])
                qk = float(row[idx_qk])
            except ValueError:
                continue

            t_list.append(t)
            ax_list.append(ax)
            ay_list.append(ay)
            az_list.append(az)
            qw_list.append(qw)
            qi_list.append(qi)
            qj_list.append(qj)
            qk_list.append(qk)

    if len(t_list) < 3:
        raise ValueError(f"{path}: za mało próbek ({len(t_list)})")

    return t_list, ax_list, ay_list, az_list, qw_list, qi_list, qj_list, qk_list


# ---------- Operacje na kwaternionach ----------

def rotate_vector_by_quat(vx: float, vy: float, vz: float,
                          qw: float, qi: float, qj: float, qk: float) -> Tuple[float, float, float]:
    """
    Obrót wektora v przez kwaternion q (zakładamy q znormalizowane).

    Używamy formuły:
        t = 2 * cross(q_vec, v)
        v' = v + w * t + cross(q_vec, t)
    gdzie q_vec = (qi, qj, qk), w = qw.
    """
    # wektor części urojonej kwaternionu
    qx, qy, qz = qi, qj, qk
    # t = 2 * q_vec x v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    # v' = v + w * t + q_vec x t
    vpx = vx + qw * tx + (qy * tz - qz * ty)
    vpy = vy + qw * ty + (qz * tx - qx * tz)
    vpz = vz + qw * tz + (qx * ty - qy * tx)

    return vpx, vpy, vpz


def accel_world_from_sensor(
    ax: List[float],
    ay: List[float],
    az: List[float],
    qw: List[float],
    qi: List[float],
    qj: List[float],
    qk: List[float],
) -> Tuple[List[float], List[float], List[float]]:
    """
    Przekonwertuj przyspieszenia z układu sensora do układu "świata"
    przy użyciu kwaternionu Game Rotation Vector.

    Uwaga: jeśli okaże się, że osie są "odwrócone" (np. UP wychodzi jako DOWN),
    wystarczy podmienić znak w tym miejscu albo użyć koniugatu q (qw, -qi, -qj, -qk).
    """
    assert len(ax) == len(ay) == len(az) == len(qw) == len(qi) == len(qj) == len(qk)

    awx: List[float] = []
    awy: List[float] = []
    awz: List[float] = []

    for vx, vy, vz, w, x, y, z in zip(ax, ay, az, qw, qi, qj, qk):
        # obracamy wektor za pomocą q
        wx, wy, wz = rotate_vector_by_quat(vx, vy, vz, w, x, y, z)
        awx.append(wx)
        awy.append(wy)
        awz.append(wz)

    return awx, awy, awz


# ---------- Baseline, okno gestu, integracja ----------

def estimate_baseline(
    t: List[float],
    ax: List[float],
    ay: List[float],
    az: List[float],
    window_s: float = 0.2,
) -> Tuple[float, float, float]:
    """
    Oszacuj wektor bazowy przyspieszenia (a0) z pierwszych `window_s` sekund.
    """
    t0 = t[0]
    count = 0
    sum_x = sum_y = sum_z = 0.0

    for ti, xi, yi, zi in zip(t, ax, ay, az):
        if ti - t0 > window_s:
            break
        sum_x += xi
        sum_y += yi
        sum_z += zi
        count += 1

    if count == 0:
        count = len(t)
        sum_x = sum(ax)
        sum_y = sum(ay)
        sum_z = sum(az)

    return sum_x / count, sum_y / count, sum_z / count


def find_gesture_window(
    t: List[float],
    ax: List[float],
    ay: List[float],
    az: List[float],
    a0x: float,
    a0y: float,
    a0z: float,
    half_window_s: float = 0.3,
) -> Tuple[int, int]:
    """
    Znajdź okno gestu wokół maksimum |a_dyn|.

    - liczymy a_dyn = a - a0
    - znajdujemy indeks i_peak, gdzie |a_dyn| jest największe
    - ustawiamy okno: [t_peak - half_window_s, t_peak + half_window_s]
    - zamieniamy na indeksy (start, end)

    Zwraca (start_idx, end_idx) – end_idx jest EXCLUSIVE.
    """
    if len(t) < 3:
        return 0, len(t)

    # znajdź pik |a_dyn|
    max_mag = -1.0
    i_peak = 0
    for i, (ti, xi, yi, zi) in enumerate(zip(t, ax, ay, az)):
        dx = xi - a0x
        dy = yi - a0y
        dz = zi - a0z
        mag = math.sqrt(dx * dx + dy * dy + dz * dz)
        if mag > max_mag:
            max_mag = mag
            i_peak = i

    t_peak = t[i_peak]
    t_start = t_peak - half_window_s
    t_end = t_peak + half_window_s

    # zamiana na indeksy
    start_idx = 0
    while start_idx < len(t) and t[start_idx] < t_start:
        start_idx += 1

    end_idx = start_idx
    while end_idx < len(t) and t[end_idx] <= t_end:
        end_idx += 1

    # sanity-check: minimum kilka próbek
    if end_idx - start_idx < 3:
        return 0, len(t)

    return start_idx, end_idx


def integrate_dynamic_velocity(
    t: List[float],
    ax: List[float],
    ay: List[float],
    az: List[float],
    a0x: float,
    a0y: float,
    a0z: float,
    min_dyn_threshold: float = 0.5,
) -> Tuple[float, float, float, float]:
    """
    Policz przybliżone Δv na każdej osi, integrując dynamiczne przyspieszenie a_dyn = a - a0.

    - min_dyn_threshold: minimalna wartość |a_dyn| [m/s^2], żeby wliczyć próbkę.
    Zwraca: (dvx, dvy, dvz, duration).
    """
    dvx = dvy = dvz = 0.0
    if len(t) < 2:
        return dvx, dvy, dvz, 0.0

    duration = t[-1] - t[0]
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt <= 0:
            continue

        dx = ax[i] - a0x
        dy = ay[i] - a0y
        dz = az[i] - a0z

        if abs(dx) < min_dyn_threshold and abs(dy) < min_dyn_threshold and abs(dz) < min_dyn_threshold:
            continue

        dvx += dx * dt
        dvy += dy * dt
        dvz += dz * dt

    return dvx, dvy, dvz, duration


# ---------- Klasyfikacja kierunku ----------

def classify_direction(
    dvx: float, dvy: float, dvz: float
) -> Tuple[str, str, float]:
    """
    Wybierz dominującą oś i znak.

    Zwraca:
        axis (X/Y/Z),
        sign ('+' / '-'),
        magnitude (abs(Δv) tej osi)
    """
    absx, absy, absz = abs(dvx), abs(dvy), abs(dvz)
    if absx >= absy and absx >= absz:
        axis = "X"
        sign = "+" if dvx >= 0 else "-"
        mag = absx
    elif absy >= absx and absy >= absz:
        axis = "Y"
        sign = "+" if dvy >= 0 else "-"
        mag = absy
    else:
        axis = "Z"
        sign = "+" if dvz >= 0 else "-"
        mag = absz

    return axis, sign, mag


def axis_sign_to_label(axis: str, sign: str) -> str:
    """
    Zamiana (oś, znak) na opis słowny.

    DOSTOSOWANE do Twojego montażu czujnika i intuicji z UP/DOWN/LEFT:
    - X+ = UP,      X- = DOWN
    - Z- = LEFT,    Z+ = RIGHT   (na podstawie Twoich logów LEFT)
    - Y+ = FORWARD, Y- = BACKWARD
    """
    if axis == "X":
        return "UP" if sign == "+" else "DOWN"
    if axis == "Z":
        return "RIGHT" if sign == "+" else "LEFT"
    if axis == "Y":
        return "FORWARD" if sign == "+" else "BACKWARD"
    return "UNKNOWN"


def infer_true_label_from_filename(path: str) -> Optional[str]:
    """
    Próbujemy wyciągnąć "prawdziwą" etykietę z nazwy pliku.

    Obsługiwane:
      up, down, left, right, forward, back, backward
    """
    name = os.path.basename(path).lower()

    if "up" in name:
        return "UP"
    if "down" in name:
        return "DOWN"
    if "left" in name:
        return "LEFT"
    if "right" in name:
        return "RIGHT"
    if "forward" in name:
        return "FORWARD"
    if "backward" in name or "back" in name:
        return "BACKWARD"

    return None


# ---------- Analiza pojedynczego pliku ----------

def analyze_file(path: str) -> None:
    t, ax, ay, az, qw, qi, qj, qk = load_csv(path)

    # 1) sensor -> world
    awx, awy, awz = accel_world_from_sensor(ax, ay, az, qw, qi, qj, qk)

    # 2) baseline w układzie świata
    a0x, a0y, a0z = estimate_baseline(t, awx, awy, awz)

    # 3) okno gestu wokół piku |a_dyn|
    start_idx, end_idx = find_gesture_window(t, awx, awy, awz, a0x, a0y, a0z, half_window_s=0.3)

    t_win = t[start_idx:end_idx]
    awx_win = awx[start_idx:end_idx]
    awy_win = awy[start_idx:end_idx]
    awz_win = awz[start_idx:end_idx]

    # 4) integracja dynamicznego przyspieszenia w tym oknie
    dvx, dvy, dvz, duration = integrate_dynamic_velocity(
        t_win, awx_win, awy_win, awz_win,
        a0x, a0y, a0z,
        min_dyn_threshold=0.5,
    )

    axis, sign, mag = classify_direction(dvx, dvy, dvz)
    pred_label = axis_sign_to_label(axis, sign)
    true_label = infer_true_label_from_filename(path)

    print(f"=== {path} ===")
    print(f"  baseline_world a0: ({a0x:.3f}, {a0y:.3f}, {a0z:.3f}) m/s^2")
    print(f"  window: samples {start_idx}..{end_idx} (n={len(t_win)}), duration={duration:.3f} s")
    print(f"  Δv_world: dvx={dvx:.3f}, dvy={dvy:.3f}, dvz={dvz:.3f} (m/s)")
    print(f"  dominant axis: {axis} {sign}  |Δv|={mag:.3f}")
    print(f"  predicted: {pred_label}")
    if true_label:
        ok = (true_label == pred_label)
        print(f"  true label (from filename): {true_label}  ->  {'OK' if ok else 'MISMATCH'}")
    print()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Offline analizator kierunku gestu z plików CSV z imu_read_cpp (world-frame)."
    )
    parser.add_argument(
        "files",
        nargs="+",
        help="Pliki CSV do analizy (np. data/up_*.csv data/down_*.csv)",
    )
    args = parser.parse_args()

    for path in args.files:
        try:
            analyze_file(path)
        except Exception as e:
            print(f"{path}: ERROR: {e}")


if __name__ == "__main__":
    main()
