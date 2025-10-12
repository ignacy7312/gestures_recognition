use anyhow::Result;
use linux_embedded_hal::I2cdev;

// Uwaga: w wydaniu crates.io (0.1.3) interfejs I²C jest w tym module:
use bno080::interface::i2c::I2cInterface;

fn main() -> Result<()> {
    // Ścieżka do magistrali I²C na Raspberry Pi
    let i2c_path = "/dev/i2c-1";

    // Domyślny adres BNO08x na I²C (zmień na 0x4B jeśli masz przestawiony pin/zworę DI)
    let addr: u8 = 0x4A;

    // 1) Otwórz urządzenie I²C
    let i2c = I2cdev::new(i2c_path)?;

    // 2) Zbuduj interfejs do BNO08x
    let _iface = I2cInterface::new(i2c, addr);

    // 3) Diagnostyka – na tym etapie mamy „połączenie” (otwartą magistralę + interfejs)
    println!(
        "OK: otwarto {} i zbudowano interfejs I²C do BNO08x pod adresem 0x{:02X}",
        i2c_path, addr
    );
    println!("(Akwizycję/SH-2/SHTP dołożymy w kolejnym kroku.)");

    Ok(())
}
