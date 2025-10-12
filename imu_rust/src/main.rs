use anyhow::{bail, Context, Result};
use linux_embedded_hal::I2cdev;
use std::thread;
use std::time::Duration;

// KLUCZOWE: import traitu z embedded_hal (a nie z linux_embedded_hal)
use embedded_hal::blocking::i2c::Read as I2cRead;

const BNO08X_I2C_ADDR: u8 = 0x4A;
const MAX_FRAME: usize = 512;

#[derive(Debug, Clone, Copy)]
struct ShtpHeader {
    len: u16,
    channel: u8,
    sequence: u8,
}

impl ShtpHeader {
    fn parse(hdr: [u8; 4]) -> Self {
        let len = u16::from_le_bytes([hdr[0], hdr[1]]);
        let channel = hdr[2];
        let sequence = hdr[3];
        Self { len, channel, sequence }
    }
}

fn main() -> Result<()> {
    // 1) Otwórz i2c-1 na RPi
    let i2c_path = "/dev/i2c-1";
    let mut i2c = I2cdev::new(i2c_path).context("nie mogę otworzyć /dev/i2c-1")?;

    println!("I2C OK: {}, urządzenie 0x{:02X}", i2c_path, BNO08X_I2C_ADDR);
    println!("SHTP dump: czekam na pierwsze pakiety (po resecie powinien przyjść 'Advertisement').");

    thread::sleep(Duration::from_millis(200));

    loop {
        // 2a) Czytaj nagłówek (4B)
        let mut hdr_buf = [0u8; 4];
        // UWAGA: używamy traitu z embedded_hal -> metoda na i2c
        i2c.read(BNO08X_I2C_ADDR, &mut hdr_buf)
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

        // 2b) Czytaj payload
        let mut payload = vec![0u8; payload_len];
        if payload_len > 0 {
            i2c.read(BNO08X_I2C_ADDR, &mut payload)
                .context("I2C read payloadu SHTP nieudany")?;
        }

        // 2c) Log
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

        thread::sleep(Duration::from_millis(5));
    }
}
