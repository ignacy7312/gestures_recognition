#!/usr/bin/env python3
"""
Offline'owy analizator kierunku ruchu dłoni na podstawie logów z imu_read_cpp.

Wejście: pliki CSV z kolumnami:
    t,ax,ay,az,gx,gy,gz,qw,qi,qj,qk

Założenia:
- Każdy plik zawiera JEDEN gest (np. jedno machnięcie w górę / dół)
  + kawałek sygnału przed i po geście (ręka nieruchomo).
- Nazwa pliku zawiera etykietę, np.: "up", "down" (np. up_01.csv, down_02.csv).

Metoda:
1. Z pierwszych ~0.2 s danych szacujemy wektor "bazowego" przyspieszenia a0.
2. Liczymy różnicę a_dyn = a - a0, czyli "dynamiczną" część ruchu.
3. Integrujemy a_dyn w czasie, dostając "przybliżone Δv" na każdej osi.
4. Oś o największej |Δv| wyznacza kierunek, a znak (±) mówi "w którą stronę".

Wynik:
- Dla każdego pliku: oś (X/Y/Z), znak (+/-), kierunek tekstowy (np. UP/DOWN)
  oraz (jeśli wykryjemy w nazwie "up"/"down") — porównanie z etykietą.
"""

import argparse
import csv
import math
import os
from typing import List, Tuple, Optional


def load_csv(path: str) -> Tuple[List[float], List[float], List[float], List[float]]:
    """Wczytaj kolumny t, ax, ay, az z pliku CSV."""
    t_list: List[float] = []
    ax_list: List[float] = []
    ay_list: List[float] = []
    az_list: List[float] = []

    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            raise ValueError(f"{path}: pusty plik")

        # Prostota: zakładamy, że header jest dokładnie taki jak z imu_read_cpp
        # t,ax,ay,az,gx,gy,gz,qw,qi,qj,qk
        try:
            idx_t = header.index("t")
            idx_ax = header.index("ax")
            idx_ay = header.index("ay")
            idx_az = header.index("az")
        except ValueError as e:
            raise ValueError(f"{path}: nie znaleziono wymaganych kolumn w headerze: {header}") from e

        for row in reader:
            if not row or len(row) <= idx_az:
                continue
            try:
                t = float(row[idx_t])
                ax = float(row[idx_ax])
                ay = float(row[idx_ay])
                az = float(row[idx_az])
            except ValueError:
                # pomijamy uszkodzone linie
                continue

            t_list.append(t)
            ax_list.append(ax)
            ay_list.append(ay)
            az_list.append(az)

    if len(t_list) < 3:
        raise ValueError(f"{path}: za mało próbek ({len(t_list)})")

    return t_list, ax_list, ay_list, az_list


def estimate_baseline(
    t: List[float],
    ax: List[float],
    ay: List[float],
    az: List[float],
    window_s: float = 0.2,
) -> Tuple[float, float, float]:
    """
    Oszacuj wektor bazowy przyspieszenia (a0) z pierwszych `window_s` sekund.

    Zakładamy, że na początku ręka jest w miarę nieruchoma, więc a ≈ g.
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

    - min_dyn_threshold: minimalna wartość |a_dyn| [m/s^2], żeby w ogóle wliczać próbkę
      (redukcja szumu, w spoczynku nie chcemy pompować Δv).
    Zwraca: (dvx, dvy, dvz, duration) – duration = czas od pierwszej do ostatniej próbki.
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

        # filtr progu dynamicznego
        if abs(dx) < min_dyn_threshold and abs(dy) < min_dyn_threshold and abs(dz) < min_dyn_threshold:
            continue

        dvx += dx * dt
        dvy += dy * dt
        dvz += dz * dt

    return dvx, dvy, dvz, duration


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

    UWAGA: To jest umowna mapa – możesz ją dostosować do tego,
    jak faktycznie trzymasz czujnik na dłoni.

    Przykład:
    - Z+ = UP,   Z- = DOWN
    - X+ = RIGHT, X- = LEFT
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
    Na razie obsługujemy tylko 'up' i 'down'.
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
    # "back" łapie zarówno "back" jak i "backward"
    if "backward" in name or "back" in name:
        return "BACKWARD"
    return None


def analyze_file(path: str) -> None:
    """
    Pełna analiza jednego pliku + wypisanie wyniku na stdout.
    """
    t, ax, ay, az = load_csv(path)
    a0x, a0y, a0z = estimate_baseline(t, ax, ay, az)

    dvx, dvy, dvz, duration = integrate_dynamic_velocity(
        t, ax, ay, az,
        a0x, a0y, a0z,
        min_dyn_threshold=0.5,  # możesz podbić / obniżyć
    )

    axis, sign, mag = classify_direction(dvx, dvy, dvz)
    pred_label = axis_sign_to_label(axis, sign)
    true_label = infer_true_label_from_filename(path)

    print(f"=== {path} ===")
    # print(f"  samples:{length(t)}")
    print(f"  baseline a0: ({a0x:.3f}, {a0y:.3f}, {a0z:.3f}) m/s^2")
    print(f"  Δv: dvx={dvx:.3f}, dvy={dvy:.3f}, dvz={dvz:.3f} (m/s)")
    print(f"  duration: {duration:.3f} s")
    print(f"  dominant axis: {axis} {sign}  |Δv|={mag:.3f}")
    print(f"  predicted: {pred_label}")
    if true_label:
        ok = (true_label == pred_label)
        print(f"  true label (from filename): {true_label}  ->  {'OK' if ok else 'MISMATCH'}")
    print()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Offline analizator kierunku gestu z plików CSV z imu_read_cpp."
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
