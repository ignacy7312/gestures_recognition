# BNO085 IMU – C++20 CLI (Raspberry Pi 3)

Minimalny projekt w C++20 dla Raspberry Pi 3, czytający BNO08x/BNO085
po I²C (SHTP/SH-2) i udostępniający:

- `imu_read_cpp` – strumień CSV z danych IMU (docelowo: accel/gyro/linear_accel + Game Rotation Vector)
- `imu_status_cpp` – strumień tekstowy/NDJSON ze statusami Activity/Steps/Stability

## Wymagania

- Raspberry Pi OS / Ubuntu na RPi3
- `cmake >= 3.20`
- `g++-12` lub `clang++-16`
- nagłówki i2c-dev:

```bash
sudo apt-get install -y cmake g++ libi2c-dev
