use std::time::{Duration, Instant};

use bno080::interface::{I2cInterface, SensorInterface};
use bno080::Error as BnoError;
use embedded_hal::blocking::delay::DelayMs;
use linux_embedded_hal::i2cdev::linux::LinuxI2CError;
use linux_embedded_hal::{Delay, I2cdev};
use thiserror::Error;

const PACKET_HEADER_LENGTH: usize = 4;
const PACKET_SEND_BUF_LEN: usize = 256;
const PACKET_RECV_BUF_LEN: usize = 1024;
const NUM_CHANNELS: usize = 6;

const CHANNEL_COMMAND: u8 = 0;
const CHANNEL_EXECUTABLE: u8 = 1;
const CHANNEL_HUB_CONTROL: u8 = 2;
const CHANNEL_SENSOR_REPORTS: u8 = 3;

const CMD_RESP_ADVERTISEMENT: u8 = 0x00;
const CMD_RESP_ERROR_LIST: u8 = 0x01;

const EXECUTABLE_DEVICE_CMD_RESET: u8 = 0x01;
const EXECUTABLE_DEVICE_RESP_RESET_COMPLETE: u8 = 0x01;

const SHUB_PROD_ID_REQ: u8 = 0xF9;
const SHUB_PROD_ID_RESP: u8 = 0xF8;
const SHUB_COMMAND_RESP: u8 = 0xF1;
const SHUB_REPORT_SET_FEATURE_CMD: u8 = 0xFD;

const SH2_CMD_INITIALIZE: u8 = 0x04;
const SH2_INIT_UNSOLICITED: u8 = 0x80;
const SH2_INIT_SYSTEM: u8 = 0x01;
const SH2_STARTUP_INIT_UNSOLICITED: u8 = SH2_CMD_INITIALIZE | SH2_INIT_UNSOLICITED;

const SENSOR_REPORTID_ACCELEROMETER: u8 = 0x01;
const SENSOR_REPORTID_GYRO_CALIBRATED: u8 = 0x02;
const SENSOR_REPORTID_LINEAR_ACCELERATION: u8 = 0x04;
const SENSOR_REPORTID_ROTATION_VECTOR: u8 = 0x05;
const SENSOR_REPORTID_GAME_ROTATION_VECTOR: u8 = 0x08;

const Q8_SCALE: f32 = 1.0 / (1 << 8) as f32;
const Q9_SCALE: f32 = 1.0 / (1 << 9) as f32;
const Q14_SCALE: f32 = 1.0 / (1 << 14) as f32;

#[derive(Debug, Clone, Copy)]
pub struct Frame {
    pub t: f64,
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
    pub qw: f32,
    pub qi: f32,
    pub qj: f32,
    pub qk: f32,
}

#[derive(Debug, Clone)]
pub struct ImuConfig {
    pub bus: u8,
    pub address: u8,
    pub hz: u16,
}

impl ImuConfig {
    pub fn device_path(&self) -> String {
        format!("/dev/i2c-{}", self.bus)
    }
}

#[derive(Debug, Error)]
pub enum ImuError {
    #[error("i2c device error: {0}")]
    Bus(#[from] LinuxI2CError),
    #[error("communication error: {0:?}")]
    Comm(BnoError<LinuxI2CError, ()>),
    #[error("timeout waiting for sensor data")]
    Timeout,
    #[error("sensor reported reset")]
    SensorReset,
    #[error("sensor did not acknowledge product id")]
    ProductId,
    #[error("protocol error: {0}")]
    Protocol(&'static str),
}

#[derive(Debug, Default)]
struct ValueSlot<T> {
    value: Option<T>,
    counter: u64,
}

impl<T> ValueSlot<T> {
    fn update(&mut self, value: T, counter: u64) {
        self.value = Some(value);
        self.counter = counter;
    }

    fn clear(&mut self) {
        self.value = None;
        self.counter = 0;
    }
}

#[derive(Debug, Default)]
struct SensorState {
    accel: ValueSlot<[f32; 3]>,
    linear_accel: ValueSlot<[f32; 3]>,
    gyro: ValueSlot<[f32; 3]>,
    quat: ValueSlot<[f32; 4]>,
    advert_received: bool,
    init_received: bool,
    prod_id_verified: bool,
    last_error: Option<u8>,
    pending_reset: bool,
}

#[derive(Debug, Default)]
struct FrameMarkers {
    quat: u64,
    gyro: u64,
    accel: u64,
}

pub struct Imu {
    iface: I2cInterface<I2cdev>,
    delay: Delay,
    seq_numbers: [u8; NUM_CHANNELS],
    recv_buf: [u8; PACKET_RECV_BUF_LEN],
    send_buf: [u8; PACKET_SEND_BUF_LEN],
    report_counter: u64,
    state: SensorState,
    last_frame: FrameMarkers,
    config: ImuConfig,
    start: Instant,
}

impl Imu {
    pub fn init(config: ImuConfig) -> Result<Self, ImuError> {
        let device_path = config.device_path();
        let dev = I2cdev::new(device_path)?;
        let iface = I2cInterface::new(dev, config.address);
        let mut imu = Self {
            iface,
            delay: Delay,
            seq_numbers: [0; NUM_CHANNELS],
            recv_buf: [0; PACKET_RECV_BUF_LEN],
            send_buf: [0; PACKET_SEND_BUF_LEN],
            report_counter: 0,
            state: SensorState::default(),
            last_frame: FrameMarkers::default(),
            config,
            start: Instant::now(),
        };
        imu.bootstrap()?;
        imu.enable_reports(imu.config.hz)?;
        Ok(imu)
    }

    pub fn enable_reports(&mut self, hz: u16) -> Result<(), ImuError> {
        let hz = hz.max(1);
        self.config.hz = hz;
        let interval_us = (1_000_000u32 / hz as u32).max(1_000);
        self.send_feature_command(SENSOR_REPORTID_GAME_ROTATION_VECTOR, interval_us)?;
        self.send_feature_command(SENSOR_REPORTID_LINEAR_ACCELERATION, interval_us)?;
        self.send_feature_command(SENSOR_REPORTID_ACCELEROMETER, interval_us)?;
        self.send_feature_command(SENSOR_REPORTID_GYRO_CALIBRATED, interval_us)?;
        Ok(())
    }

    pub fn poll_frame(&mut self, timeout: Duration) -> Result<Frame, ImuError> {
        let deadline = Instant::now() + timeout;
        loop {
            let now = Instant::now();
            if now >= deadline {
                return Err(ImuError::Timeout);
            }
            let remaining = deadline.saturating_duration_since(now);
            let wait_ms = remaining.as_millis().min(250) as u8;
            let received = self.receive_packet_with_timeout(wait_ms)?;
            if received > 0 {
                self.handle_received_packet(received)?;
                if self.state.pending_reset {
                    self.state.pending_reset = false;
                    return Err(ImuError::SensorReset);
                }
                if let Some(frame) = self.try_build_frame() {
                    return Ok(frame);
                }
            }
        }
    }

    pub fn handle_reset(&mut self) -> Result<(), ImuError> {
        self.bootstrap()?;
        self.enable_reports(self.config.hz)?;
        self.clear_measurements();
        Ok(())
    }

    pub fn last_error(&self) -> Option<u8> {
        self.state.last_error
    }

    fn bootstrap(&mut self) -> Result<(), ImuError> {
        self.seq_numbers = [0; NUM_CHANNELS];
        self.state = SensorState::default();
        self.last_frame = FrameMarkers::default();
        self.report_counter = 0;
        self.iface.setup(&mut self.delay)?;
        self.delay.delay_ms(1u8);
        self.soft_reset()?;
        self.delay.delay_ms(150u8);
        self.eat_all_messages(200)?;
        self.delay.delay_ms(50u8);
        self.eat_all_messages(50)?;
        self.verify_product_id()?;
        self.state.pending_reset = false;
        self.start = Instant::now();
        Ok(())
    }

    fn soft_reset(&mut self) -> Result<(), ImuError> {
        let data = [EXECUTABLE_DEVICE_CMD_RESET];
        let recv_len = self.send_and_receive_packet(CHANNEL_EXECUTABLE, &data)?;
        if recv_len > 0 {
            self.handle_received_packet(recv_len)?;
        }
        Ok(())
    }

    fn verify_product_id(&mut self) -> Result<(), ImuError> {
        let cmd = [SHUB_PROD_ID_REQ, 0];
        self.send_packet(CHANNEL_HUB_CONTROL, &cmd)?;
        let start = Instant::now();
        while !self.state.prod_id_verified {
            let handled = self.handle_one_message(150)?;
            if handled == 0 && start.elapsed() > Duration::from_millis(500) {
                return Err(ImuError::ProductId);
            }
        }
        Ok(())
    }

    fn clear_measurements(&mut self) {
        self.state.accel.clear();
        self.state.linear_accel.clear();
        self.state.gyro.clear();
        self.state.quat.clear();
        self.last_frame = FrameMarkers::default();
    }

    fn try_build_frame(&mut self) -> Option<Frame> {
        let quat_slot = &self.state.quat;
        let gyro_slot = &self.state.gyro;
        if quat_slot.value.is_none() || gyro_slot.value.is_none() {
            return None;
        }
        let (accel_value, accel_counter) = match self.select_accel_slot() {
            Some(val) => val,
            None => return None,
        };
        if quat_slot.counter <= self.last_frame.quat
            || gyro_slot.counter <= self.last_frame.gyro
            || accel_counter <= self.last_frame.accel
        {
            return None;
        }
        let quat = quat_slot.value.unwrap();
        let gyro = gyro_slot.value.unwrap();
        self.last_frame.quat = quat_slot.counter;
        self.last_frame.gyro = gyro_slot.counter;
        self.last_frame.accel = accel_counter;
        Some(Frame {
            t: self.start.elapsed().as_secs_f64(),
            ax: accel_value[0],
            ay: accel_value[1],
            az: accel_value[2],
            gx: gyro[0],
            gy: gyro[1],
            gz: gyro[2],
            qw: quat[0],
            qi: quat[1],
            qj: quat[2],
            qk: quat[3],
        })
    }

    fn select_accel_slot(&self) -> Option<([f32; 3], u64)> {
        let linear = &self.state.linear_accel;
        let accel = &self.state.accel;
        match (&linear.value, &accel.value) {
            (Some(lin), Some(acc)) => {
                if linear.counter >= accel.counter {
                    Some((*lin, linear.counter))
                } else {
                    Some((*acc, accel.counter))
                }
            }
            (Some(lin), None) => Some((*lin, linear.counter)),
            (None, Some(acc)) => Some((*acc, accel.counter)),
            (None, None) => None,
        }
    }

    fn send_feature_command(&mut self, report_id: u8, interval_us: u32) -> Result<(), ImuError> {
        let body = [
            SHUB_REPORT_SET_FEATURE_CMD,
            report_id,
            0,
            0,
            0,
            (interval_us & 0xFF) as u8,
            ((interval_us >> 8) & 0xFF) as u8,
            ((interval_us >> 16) & 0xFF) as u8,
            ((interval_us >> 24) & 0xFF) as u8,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];
        self.send_packet(CHANNEL_HUB_CONTROL, &body)?;
        Ok(())
    }

    fn eat_all_messages(&mut self, max_iterations: u16) -> Result<(), ImuError> {
        for _ in 0..max_iterations {
            let handled = self.handle_one_message(10)?;
            if handled == 0 {
                break;
            }
        }
        Ok(())
    }

    fn handle_one_message(&mut self, timeout_ms: u8) -> Result<u32, ImuError> {
        let received = self.receive_packet_with_timeout(timeout_ms)?;
        if received > 0 {
            self.handle_received_packet(received)?;
            Ok(1)
        } else {
            Ok(0)
        }
    }

    fn receive_packet_with_timeout(&mut self, timeout_ms: u8) -> Result<usize, ImuError> {
        Ok(self
            .iface
            .read_with_timeout(&mut self.recv_buf, &mut self.delay, timeout_ms)?)
    }

    fn send_packet(&mut self, channel: u8, body: &[u8]) -> Result<usize, ImuError> {
        let packet_len = self.prep_send_packet(channel, body);
        self.iface.write_packet(&self.send_buf[..packet_len])?;
        Ok(packet_len)
    }

    fn send_and_receive_packet(&mut self, channel: u8, body: &[u8]) -> Result<usize, ImuError> {
        let packet_len = self.prep_send_packet(channel, body);
        Ok(self
            .iface
            .send_and_receive_packet(&self.send_buf[..packet_len], &mut self.recv_buf)?)
    }

    fn prep_send_packet(&mut self, channel: u8, body: &[u8]) -> usize {
        let packet_len = body.len() + PACKET_HEADER_LENGTH;
        let header = [
            (packet_len & 0xFF) as u8,
            ((packet_len >> 8) & 0x7F) as u8,
            channel,
            self.seq_numbers[channel as usize],
        ];
        self.seq_numbers[channel as usize] = self.seq_numbers[channel as usize].wrapping_add(1);
        self.send_buf[..PACKET_HEADER_LENGTH].copy_from_slice(&header);
        self.send_buf[PACKET_HEADER_LENGTH..packet_len].copy_from_slice(body);
        packet_len
    }

    fn handle_received_packet(&mut self, received_len: usize) -> Result<(), ImuError> {
        if received_len < PACKET_HEADER_LENGTH {
            return Err(ImuError::Protocol("packet shorter than header"));
        }
        let channel = self.recv_buf[2];
        let report_id = if received_len > PACKET_HEADER_LENGTH {
            self.recv_buf[4]
        } else {
            0
        };
        match channel {
            CHANNEL_COMMAND => self.handle_command_channel(report_id, received_len),
            CHANNEL_EXECUTABLE => self.handle_exec_channel(report_id),
            CHANNEL_HUB_CONTROL => self.handle_hub_control(report_id, received_len),
            CHANNEL_SENSOR_REPORTS => self.handle_sensor_reports(received_len),
            _ => Ok(()),
        }
    }

    fn handle_command_channel(
        &mut self,
        report_id: u8,
        received_len: usize,
    ) -> Result<(), ImuError> {
        match report_id {
            CMD_RESP_ADVERTISEMENT => {
                self.state.advert_received = true;
                if received_len < PACKET_HEADER_LENGTH + 1 {
                    return Ok(());
                }
                Ok(())
            }
            CMD_RESP_ERROR_LIST => {
                if received_len > PACKET_HEADER_LENGTH + 1 {
                    self.state.last_error = Some(self.recv_buf[PACKET_HEADER_LENGTH + 1]);
                }
                Ok(())
            }
            _ => Ok(()),
        }
    }

    fn handle_exec_channel(&mut self, report_id: u8) -> Result<(), ImuError> {
        if report_id == EXECUTABLE_DEVICE_RESP_RESET_COMPLETE {
            self.state.pending_reset = true;
        }
        Ok(())
    }

    fn handle_hub_control(&mut self, report_id: u8, received_len: usize) -> Result<(), ImuError> {
        match report_id {
            SHUB_COMMAND_RESP => {
                if received_len > PACKET_HEADER_LENGTH + 2 {
                    let resp = self.recv_buf[PACKET_HEADER_LENGTH + 2];
                    if resp == SH2_STARTUP_INIT_UNSOLICITED || resp == SH2_INIT_SYSTEM {
                        self.state.init_received = true;
                    }
                }
                Ok(())
            }
            SHUB_PROD_ID_RESP => {
                self.state.prod_id_verified = true;
                Ok(())
            }
            _ => Ok(()),
        }
    }

    fn handle_sensor_reports(&mut self, received_len: usize) -> Result<(), ImuError> {
        if received_len <= PACKET_HEADER_LENGTH + 5 {
            return Ok(());
        }
        let mut cursor = PACKET_HEADER_LENGTH + 5;
        while cursor < received_len {
            if cursor + 4 > received_len {
                break;
            }
            let feature_report_id = self.recv_buf[cursor];
            let _seq = self.recv_buf[cursor + 1];
            let _status = self.recv_buf[cursor + 2];
            let _delay = self.recv_buf[cursor + 3];
            cursor += 4;
            let remaining = received_len.saturating_sub(cursor);
            let packet = &self.recv_buf[..received_len];
            let read_i16 = |idx: &mut usize| -> Option<i16> {
                if *idx + 2 > packet.len() {
                    None
                } else {
                    let val = i16::from_le_bytes([packet[*idx], packet[*idx + 1]]);
                    *idx += 2;
                    Some(val)
                }
            };
            let mut idx = cursor;
            match feature_report_id {
                SENSOR_REPORTID_GAME_ROTATION_VECTOR | SENSOR_REPORTID_ROTATION_VECTOR => {
                    if remaining < 8 {
                        break;
                    }
                    let qi = read_i16(&mut idx).ok_or(ImuError::Protocol("quat data"))?;
                    let qj = read_i16(&mut idx).ok_or(ImuError::Protocol("quat data"))?;
                    let qk = read_i16(&mut idx).ok_or(ImuError::Protocol("quat data"))?;
                    let qr = read_i16(&mut idx).ok_or(ImuError::Protocol("quat data"))?;
                    if idx + 2 <= received_len {
                        let _ = read_i16(&mut idx);
                    }
                    self.bump_report_counter();
                    let quat = [
                        q14_to_f32(qr),
                        q14_to_f32(qi),
                        q14_to_f32(qj),
                        q14_to_f32(qk),
                    ];
                    self.state.quat.update(quat, self.report_counter);
                }
                SENSOR_REPORTID_LINEAR_ACCELERATION => {
                    if remaining < 6 {
                        break;
                    }
                    let ax = read_i16(&mut idx).ok_or(ImuError::Protocol("lin accel"))?;
                    let ay = read_i16(&mut idx).ok_or(ImuError::Protocol("lin accel"))?;
                    let az = read_i16(&mut idx).ok_or(ImuError::Protocol("lin accel"))?;
                    self.bump_report_counter();
                    self.state.linear_accel.update(
                        [q8_to_f32(ax), q8_to_f32(ay), q8_to_f32(az)],
                        self.report_counter,
                    );
                }
                SENSOR_REPORTID_ACCELEROMETER => {
                    if remaining < 6 {
                        break;
                    }
                    let ax = read_i16(&mut idx).ok_or(ImuError::Protocol("accel"))?;
                    let ay = read_i16(&mut idx).ok_or(ImuError::Protocol("accel"))?;
                    let az = read_i16(&mut idx).ok_or(ImuError::Protocol("accel"))?;
                    self.bump_report_counter();
                    self.state.accel.update(
                        [q8_to_f32(ax), q8_to_f32(ay), q8_to_f32(az)],
                        self.report_counter,
                    );
                }
                SENSOR_REPORTID_GYRO_CALIBRATED => {
                    if remaining < 6 {
                        break;
                    }
                    let gx = read_i16(&mut idx).ok_or(ImuError::Protocol("gyro"))?;
                    let gy = read_i16(&mut idx).ok_or(ImuError::Protocol("gyro"))?;
                    let gz = read_i16(&mut idx).ok_or(ImuError::Protocol("gyro"))?;
                    self.bump_report_counter();
                    self.state.gyro.update(
                        [q9_to_f32(gx), q9_to_f32(gy), q9_to_f32(gz)],
                        self.report_counter,
                    );
                }
                _ => {}
            }
            cursor = idx;
        }
        Ok(())
    }

    fn bump_report_counter(&mut self) {
        self.report_counter = self.report_counter.wrapping_add(1);
        if self.report_counter == 0 {
            self.report_counter = 1;
        }
    }
}

fn q8_to_f32(val: i16) -> f32 {
    val as f32 * Q8_SCALE
}

fn q9_to_f32(val: i16) -> f32 {
    val as f32 * Q9_SCALE
}

fn q14_to_f32(val: i16) -> f32 {
    val as f32 * Q14_SCALE
}

impl From<BnoError<LinuxI2CError, ()>> for ImuError {
    fn from(err: BnoError<LinuxI2CError, ()>) -> Self {
        Self::Comm(err)
    }
}
