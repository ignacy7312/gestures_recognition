import argparse
import subprocess
import time
import os
from datetime import datetime

def main():
    parser = argparse.ArgumentParser(description="Odpal ./build/imu_read X razy z różnymi plikami wyjściowymi.")
    parser.add_argument(
        "--direction",
        required=True,
        choices=["left", "right", "up", "forward", "backward"],
        help="Kierunek / prefix pliku: left, right, up, forward, backward"
    )
    parser.add_argument(
        "--count",
        type=int,
        required=True,
        help="Ile razy powtórzyć cykl"
    )

    args = parser.parse_args()

    # Tworzymy folder data + datetime
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    base_dir = f"data/data{timestamp}"
    os.makedirs(base_dir, exist_ok=True)

    print(f"Zapis danych w: {base_dir}\n")

    for i in range(args.count):
        index = i + 1
        out_path = f"{base_dir}/{args.direction}{index}.csv"

        cmd = [
            "./build/imu_read",
            "--bus", "1",
            "--addr", "0x4A",
            "--hz", "100",
            "--timeout-ms", "50",
            "--out", out_path
        ]

        print(f"[{index}/{args.count}] Start: {' '.join(cmd)}")

        # start programu
        proc = subprocess.Popen(cmd)

        # działa 2 sekundy
        time.sleep(4)

        # zamykanie jeśli nadal działa
        if proc.poll() is None:
            print("  → Zamykanie procesu...")
            proc.terminate()
            time.sleep(0.5)
            if proc.poll() is None:
                print("  → Kill procesu...")
                proc.kill()

        print("  → Czekam 2 sekundy...\n")
        time.sleep(2)

if __name__ == "__main__":
    main()
