use anyhow::{bail, Context, Result};
use clap::{Args, Parser, Subcommand, ValueEnum};
use ctrlc;
use embedded_hal::blocking::i2c::Read as I2cRead;
use env_logger::Env;
use imu_rust::{Imu, ImuConfig, ImuError};
use linux_embedded_hal::I2cdev;
use log::{error, info, warn, LevelFilter};
use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

const MAX_FRAME: usize = 512;
const INITIAL_CHECK_DELAY_MS: u64 = 200;
const METRICS_INTERVAL: Duration = Duration::from_secs(5);
const FLUSH_INTERVAL: usize = 1;

#[derive(Debug, Parser)]
#[command(name = "imu", version, about = "BNO085 tooling")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Szybkie sprawdzenie połączenia poprzez zrzut ramek SHTP
    Check(CheckArgs),
    /// Strumień CSV z pełnej obsługi IMU (dawne imu_read)
    Read(ReadArgs),
}

#[derive(Debug, Args)]
struct CheckArgs {
    #[arg(
        long,
        default_value_t = 1u8,
        help = "Numer magistrali I2C (domyślnie 1)"
    )]
    bus: u8,

    #[arg(
        long,
        default_value = "0x4A",
        value_parser = parse_hex_u8,
        help = "Adres I2C BNO08x w postaci heksadecymalnej"
    )]
    addr: u8,

    #[arg(
        long,
        default_value_t = 5u64,
        help = "Opóźnienie (ms) pomiędzy kolejnymi odczytami ramek"
    )]
    wait_ms: u64,
}

#[derive(Debug, Args)]
struct ReadArgs {
    #[arg(long, default_value_t = 1u8, help = "I2C bus number (default 1)")]
    bus: u8,

    #[arg(
        long,
        default_value = "0x4A",
        value_parser = parse_hex_u8,
        help = "I2C address in hex (default 0x4A)"
    )]
    addr: u8,

    #[arg(
        long,
        default_value_t = 100u16,
        value_parser = clap::value_parser!(u16).range(1..=400),
        help = "Target sampling frequency in Hz"
    )]
    hz: u16,

    #[arg(
        long,
        default_value_t = 50u64,
        value_parser = clap::value_parser!(u64).range(1..=500),
        help = "Poll timeout in milliseconds"
    )]
    timeout_ms: u64,

    #[arg(long, default_value_t = false, help = "Skip printing CSV header")]
    no_header: bool,

    #[arg(
        long,
        default_value = "info",
        value_enum,
        help = "Log level for stderr output"
    )]
    log_level: LogLevel,
}

#[derive(Copy, Clone, Debug, ValueEnum)]
enum LogLevel {
    Error,
    Warn,
    Info,
    Debug,
}

impl From<LogLevel> for LevelFilter {
    fn from(level: LogLevel) -> Self {
        match level {
            LogLevel::Error => LevelFilter::Error,
            LogLevel::Warn => LevelFilter::Warn,
            LogLevel::Info => LevelFilter::Info,
            LogLevel::Debug => LevelFilter::Debug,
        }
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::Check(args) => run_check(args),
        Command::Read(args) => run_read(args),
    }
}

fn run_check(args: CheckArgs) -> Result<()> {
    let i2c_path = format!("/dev/i2c-{}", args.bus);
    let mut i2c =
        I2cdev::new(&i2c_path).with_context(|| format!("nie mogę otworzyć {}", i2c_path))?;

    println!("I2C OK: {}, urządzenie 0x{:02X}", i2c_path, args.addr);
    println!(
        "SHTP dump: czekam na pierwsze pakiety (po resecie powinien przyjść 'Advertisement')."
    );

    thread::sleep(Duration::from_millis(INITIAL_CHECK_DELAY_MS));
    let wait = Duration::from_millis(args.wait_ms);

    loop {
        let mut hdr_buf = [0u8; 4];
        i2c.read(args.addr, &mut hdr_buf)
            .context("I2C read nagłówka SHTP nieudany")?;

        let hdr = ShtpHeader::parse(hdr_buf);
        if hdr.len < 4 {
            bail!("SHTP: nieprawidłowa długość ramki: {}", hdr.len);
        }

        let payload_len = (hdr.len as usize).saturating_sub(4);
        if payload_len > (MAX_FRAME - 4) {
            bail!(
                "SHTP: payload {} B > bufor {} B — zwiększ MAX_FRAME",
                payload_len,
                MAX_FRAME - 4
            );
        }

        let mut payload = vec![0u8; payload_len];
        if payload_len > 0 {
            i2c.read(args.addr, &mut payload)
                .context("I2C read payloadu SHTP nieudany")?;
        }

        println!(
            "[SHTP] len={}, ch={}, seq={}, payload={}",
            hdr.len, hdr.channel, hdr.sequence, payload_len
        );

        if payload_len > 0 {
            let show = payload_len.min(16);
            print!("        first {}B:", show);
            for b in &payload[..show] {
                print!(" {:02X}", b);
            }
            println!();
        }

        thread::sleep(wait);
    }
}

fn run_read(args: ReadArgs) -> Result<()> {
    init_logging(args.log_level.into());

    let config = ImuConfig {
        bus: args.bus,
        address: args.addr,
        hz: args.hz,
    };

    info!(
        "starting imu_read bus={} addr=0x{:02X} hz={} timeout_ms={}",
        config.bus, config.address, config.hz, args.timeout_ms
    );

    let mut imu = Imu::init(config.clone()).context("failed to initialize IMU")?;

    let running = Arc::new(AtomicBool::new(true));
    {
        let running = Arc::clone(&running);
        ctrlc::set_handler(move || {
            if running.swap(false, Ordering::SeqCst) {
                warn!("SIGINT received, stopping ...");
            }
        })
        .context("failed to install SIGINT handler")?;
    }

    let mut stdout = io::BufWriter::new(io::stdout());
    if !args.no_header {
        writeln!(stdout, "t,ax,ay,az,gx,gy,gz,qw,qi,qj,qk")?;
        stdout.flush()?;
    }

    let interval = Duration::from_secs_f64(1.0 / f64::from(args.hz));
    let timeout = Duration::from_millis(args.timeout_ms);
    let mut next_tick = Instant::now();

    let mut frames_total: u64 = 0;
    let mut drops_total: u64 = 0;
    let mut frames_window: u64 = 0;
    let mut drops_window: u64 = 0;
    let mut last_error_msg: Option<String> = None;
    let mut last_flush = 0usize;
    let mut metrics_start = Instant::now();

    while running.load(Ordering::SeqCst) {
        if let Some(delay) = next_tick.checked_duration_since(Instant::now()) {
            if delay > Duration::from_micros(200) {
                thread::sleep(delay);
            }
        }
        next_tick = Instant::now() + interval;

        match imu.poll_frame(timeout) {
            Ok(frame) => {
                frames_total += 1;
                frames_window += 1;
                write!(
                    stdout,
                    "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}\n",
                    frame.t,
                    frame.ax,
                    frame.ay,
                    frame.az,
                    frame.gx,
                    frame.gy,
                    frame.gz,
                    frame.qw,
                    frame.qi,
                    frame.qj,
                    frame.qk
                )?;
                last_flush += 1;
                if last_flush >= FLUSH_INTERVAL {
                    stdout.flush()?;
                    last_flush = 0;
                }
            }
            Err(ImuError::Timeout) => {
                drops_total += 1;
                drops_window += 1;
                last_error_msg = Some("timeout".to_owned());
                continue;
            }
            Err(ImuError::SensorReset) => {
                drops_total += 1;
                drops_window += 1;
                last_error_msg = Some("sensor_reset".to_owned());
                warn!("sensor reported reset, reinitializing");
                if let Err(err) = imu.handle_reset() {
                    warn!("reset handling failed: {err:?}, attempting full reinit");
                    match recover_imu(&config, &running) {
                        Some(new_imu) => imu = new_imu,
                        None => break,
                    }
                }
                continue;
            }
            Err(ImuError::Comm(err)) => {
                drops_total += 1;
                drops_window += 1;
                last_error_msg = Some(format!("comm: {err:?}"));
                warn!("i2c communication error: {err:?}, attempting recovery");
                thread::sleep(Duration::from_millis(10));
                if let Err(reset_err) = imu.handle_reset() {
                    warn!("reset after comm error failed: {reset_err:?}, reinitializing");
                    match recover_imu(&config, &running) {
                        Some(new_imu) => imu = new_imu,
                        None => break,
                    }
                }
                continue;
            }
            Err(ImuError::Bus(err)) => {
                drops_total += 1;
                drops_window += 1;
                last_error_msg = Some(format!("bus: {err}"));
                error!("i2c bus error {err}, reopening");
                match recover_imu(&config, &running) {
                    Some(new_imu) => imu = new_imu,
                    None => break,
                }
                continue;
            }
            Err(ImuError::ProductId) => {
                drops_total += 1;
                drops_window += 1;
                last_error_msg = Some("product_id".to_owned());
                warn!("product id verification failed, reinitializing");
                match recover_imu(&config, &running) {
                    Some(new_imu) => imu = new_imu,
                    None => break,
                }
                continue;
            }
            Err(ImuError::Protocol(msg)) => {
                drops_total += 1;
                drops_window += 1;
                last_error_msg = Some(format!("protocol: {msg}"));
                warn!("protocol error ({msg}), attempting recovery");
                if let Err(err) = imu.handle_reset() {
                    warn!("protocol recovery failed: {err:?}, reinitializing");
                    match recover_imu(&config, &running) {
                        Some(new_imu) => imu = new_imu,
                        None => break,
                    }
                }
                continue;
            }
        }

        if metrics_start.elapsed() >= METRICS_INTERVAL {
            let elapsed = metrics_start.elapsed().as_secs_f64();
            let effective_hz = if elapsed > 0.0 {
                frames_window as f64 / elapsed
            } else {
                0.0
            };
            let total_window = frames_window + drops_window;
            let drops_pct = if total_window > 0 {
                (drops_window as f64 / total_window as f64) * 100.0
            } else {
                0.0
            };
            info!(
                "metrics frames_total={} frames_last_window={} effective_hz={:.2} drops_pct={:.2} last_error={} drops_total={}",
                frames_total,
                frames_window,
                effective_hz,
                drops_pct,
                last_error_msg.as_deref().unwrap_or("none"),
                drops_total
            );
            frames_window = 0;
            drops_window = 0;
            metrics_start = Instant::now();
        }
    }

    stdout.flush()?;
    info!(
        "imu_read stopped: frames_total={} drops_total={}",
        frames_total, drops_total
    );
    Ok(())
}

fn init_logging(level: LevelFilter) {
    let env = Env::default().default_filter_or(level.to_string());
    env_logger::Builder::from_env(env)
        .format_timestamp_millis()
        .init();
}

fn parse_hex_u8(input: &str) -> Result<u8, String> {
    let trimmed = input.trim();
    let without_prefix = trimmed
        .strip_prefix("0x")
        .or_else(|| trimmed.strip_prefix("0X"))
        .unwrap_or(trimmed);
    u8::from_str_radix(without_prefix, 16)
        .map_err(|err| format!("invalid hex byte '{input}': {err}"))
}

fn recover_imu(config: &ImuConfig, running: &Arc<AtomicBool>) -> Option<Imu> {
    let mut backoff = Duration::from_millis(100);
    loop {
        if !running.load(Ordering::SeqCst) {
            return None;
        }
        match Imu::init(config.clone()) {
            Ok(new_imu) => {
                info!("reinitialized imu");
                return Some(new_imu);
            }
            Err(err) => {
                warn!("reinit failed: {err:?}, retrying in {:?}", backoff);
                thread::sleep(backoff);
                backoff = (backoff * 2).min(Duration::from_secs(2));
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct ShtpHeader {
    len: u16,
    channel: u8,
    sequence: u8,
}

impl ShtpHeader {
    fn parse(hdr: [u8; 4]) -> Self {
        let raw_len = u16::from_le_bytes([hdr[0], hdr[1]]);
        let len = raw_len & 0x7FFF; // bit 15 is continuation flag
        let channel = hdr[2];
        let sequence = hdr[3];
        Self {
            len,
            channel,
            sequence,
        }
    }
}
