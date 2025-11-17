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
import os
from typing import Tuple, Optional

import numpy as np


# ---------- Wczytywanie danych ----------

def load_csv(path: str) -> Tuple[
    np.ndarray,  # t
    np.ndarray,  # ax
    np.ndarray,  # ay
    np.ndarray,  # az
    np.ndarray,  # qw
    np.ndarray,  # qi
    np.ndarray,  # qj
    np.ndarray,  # qk
]:
    """Wczytaj kolumny t, ax, ay, az, qw, qi, qj, qk z pliku CSV."""
    required_cols = ("t", "ax", "ay", "az", "qw", "qi", "qj", "qk")
    try:
        data = np.genfromtxt(
            path,
            delimiter=",",
            names=True,
            dtype=float,
            ndmin=1,
        )
    except Exception as e:
        raise ValueError(f"{path}: nie można odczytać pliku CSV") from e

    if data.size == 0:
        raise ValueError(f"{path}: pusty plik")

    if data.dtype.names is None:
        raise ValueError(f"{path}: brak headera z kolumnami: {required_cols}")

    for col in required_cols:
        if col not in data.dtype.names:
            raise ValueError(f"{path}: nie znaleziono kolumny '{col}' w headerze {data.dtype.names}")

    if data.shape[0] < 3:
        raise ValueError(f"{path}: za mało próbek ({data.shape[0]})")

    columns = tuple(np.asarray(data[col], dtype=float) for col in required_cols)
    return columns  # type: ignore[return-value]


# ---------- Operacje na kwaternionach ----------

def accel_world_from_sensor(
    ax: np.ndarray,
    ay: np.ndarray,
    az: np.ndarray,
    qw: np.ndarray,
    qi: np.ndarray,
    qj: np.ndarray,
    qk: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Przekonwertuj przyspieszenia z układu sensora do układu "świata"
    przy użyciu kwaternionu Game Rotation Vector.

    Uwaga: jeśli okaże się, że osie są "odwrócone" (np. UP wychodzi jako DOWN),
    wystarczy podmienić znak w tym miejscu albo użyć koniugatu q (qw, -qi, -qj, -qk).
    """
    v = np.stack((ax, ay, az), axis=1)
    q_vec = np.stack((qi, qj, qk), axis=1)
    t = 2.0 * np.cross(q_vec, v)
    rotated = v + qw[:, np.newaxis] * t + np.cross(q_vec, t)
    return rotated[:, 0], rotated[:, 1], rotated[:, 2]


# ---------- Baseline, okno gestu, integracja ----------

def estimate_baseline(
    t: np.ndarray,
    ax: np.ndarray,
    ay: np.ndarray,
    az: np.ndarray,
    window_s: float = 0.2,
) -> Tuple[float, float, float]:
    """
    Oszacuj wektor bazowy przyspieszenia (a0) z pierwszych `window_s` sekund.
    """
    t_rel = t - t[0]
    mask = t_rel <= window_s
    if not np.any(mask):
        mask = slice(None)

    return (
        float(np.mean(ax[mask])),
        float(np.mean(ay[mask])),
        float(np.mean(az[mask])),
    )


def find_gesture_window(
    t: np.ndarray,
    ax: np.ndarray,
    ay: np.ndarray,
    az: np.ndarray,
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
    if t.size < 3:
        return 0, int(t.size)

    a_dyn = np.vstack((ax - a0x, ay - a0y, az - a0z))
    magnitudes = np.linalg.norm(a_dyn, axis=0)
    i_peak = int(np.argmax(magnitudes))

    t_peak = t[i_peak]
    t_start = t_peak - half_window_s
    t_end = t_peak + half_window_s

    start_idx = int(np.searchsorted(t, t_start, side="left"))
    end_idx = int(np.searchsorted(t, t_end, side="right"))

    if end_idx - start_idx < 3:
        return 0, int(t.size)

    return start_idx, end_idx


def integrate_dynamic_velocity(
    t: np.ndarray,
    ax: np.ndarray,
    ay: np.ndarray,
    az: np.ndarray,
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
    if t.size < 2:
        return 0.0, 0.0, 0.0, 0.0

    dt = np.diff(t)
    valid_dt = dt > 0
    if not np.any(valid_dt):
        return 0.0, 0.0, 0.0, 0.0

    a_dyn = np.vstack((ax - a0x, ay - a0y, az - a0z))
    a_dyn = a_dyn[:, 1:]

    magnitude_mask = np.any(np.abs(a_dyn) >= min_dyn_threshold, axis=0)
    mask = valid_dt & magnitude_mask
    if not np.any(mask):
        return 0.0, 0.0, 0.0, float(t[-1] - t[0])

    masked_dt = dt[mask]
    masked_dyn = a_dyn[:, mask]
    dv = np.sum(masked_dyn * masked_dt, axis=1)
    duration = float(t[-1] - t[0])
    return float(dv[0]), float(dv[1]), float(dv[2]), duration


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
    dv = np.array([dvx, dvy, dvz])
    axis_idx = int(np.argmax(np.abs(dv)))
    axis = ("X", "Y", "Z")[axis_idx]
    axis_value = dv[axis_idx]
    sign = "+" if axis_value >= 0 else "-"
    return axis, sign, float(abs(axis_value))


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
