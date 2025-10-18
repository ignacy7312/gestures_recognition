# Developer Notes

## CLI overview
- Build with `cargo build --release`; during development prefer `cargo run -- ...`.
- Global help: `cargo run -- --help`.
- Each subcommand has its own help (`cargo run -- check --help`, `cargo run -- read --help`).
- Default I²C bus is `1`, default address `0x4A` (BNO085 with PS0/PS1 low).

## `check` command
- Quick connectivity smoke test; dumps SHTP headers/payload sizes.
- Example: `cargo run -- check --bus 1 --addr 0x4A --wait-ms 5`.
- Prints first 16 B payload preview in hex to stdout for manual inspection.
- Useful immediately after wiring or power cycling.

## `read` command
- Streams fused IMU data as CSV on stdout, logs diagnostics on stderr.
- Typical run: `cargo run -- read --bus 1 --addr 0x4A --hz 100 --timeout-ms 50`.
- Options:
  - `--bus <u8>`: I²C bus (default `1`).
  - `--addr <hex>`: sensor address (`0x4A` default, accepts `0x..`).
  - `--hz <u16>`: target sampling rate in Hz (default `100`, clamp 1..400).
  - `--timeout-ms <u64>`: per-poll timeout before reporting a drop (default `50`).
  - `--no-header`: omit the CSV header row.
  - `--log-level <error|warn|info|debug>`: baseline stderr verbosity (default `info`).
- CSV columns: `t,ax,ay,az,gx,gy,gz,qw,qi,qj,qk` (time in seconds since init, SI units).

### Logging & metrics
- Uses `env_logger`; CLI `--log-level` sets the global filter, override with `RUST_LOG` if needed.
- Every 5 s prints aggregate metrics: total frames, frames in last window, effective Hz, drop %, last error, total drops.
- Setting `--log-level debug` reveals detailed recovery paths and I²C errors.

### Recovery behaviour
- On timeout/soft errors: attempts `Imu::handle_reset()` (soft reset + re-enable reports).
- On hard I²C errors/Product ID failures: closes the bus and retries full `Imu::init()` with exponential backoff (100 ms → 2 s) until success or SIGINT.
- SIGINT (`Ctrl+C`) flips a shared flag and allows graceful shutdown after flushing CSV output.
