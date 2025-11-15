#include "bno/sh2_reports.hpp"

#include <cmath>
#include <cstring>


namespace bno {

namespace {

// mały helper
inline std::int16_t le_i16(const std::uint8_t* p) {
    return static_cast<std::int16_t>(p[0] | (std::int16_t(p[1]) << 8));
}

inline Sh2Accuracy decode_accuracy(std::uint8_t status) {
    // Wg SH-2 RM: w polu „Status” dolne 2 bity kodują dokładność 0..3. :contentReference[oaicite:6]{index=6}
    std::uint8_t acc = status & 0x03u;
    switch (acc) {
        case 1: return Sh2Accuracy::Low;
        case 2: return Sh2Accuracy::Medium;
        case 3: return Sh2Accuracy::High;
        default: return Sh2Accuracy::Unreliable;
    }
}

} // namespace

std::optional<Sh2SensorEvent> parse_sh2_sensor_event(const std::uint8_t* data,
                                                     std::size_t len) {
    if (!data || len < 4) {
        return std::nullopt;
    }

    const std::uint8_t report_id = data[0];
    const std::uint8_t status    = data[2];

    Sh2SensorEvent evt{};
    evt.accuracy = decode_accuracy(status);

    switch (report_id) {
    case 0x01: { // Accelerometer (m/s^2, Q8) :contentReference[oaicite:7]{index=7}
        if (len < 10) return std::nullopt;
        std::int16_t x_raw = le_i16(&data[4]);
        std::int16_t y_raw = le_i16(&data[6]);
        std::int16_t z_raw = le_i16(&data[8]);
        constexpr float SCALE = 1.0f / 256.0f; // Q8
        Vec3f v{
            x_raw * SCALE,
            y_raw * SCALE,
            z_raw * SCALE,
        };
        evt.sensor_id = Sh2SensorId::Accelerometer;
        evt.accel = v;
        break;
    }

    case 0x04: { // Linear Acceleration (m/s^2, Q8) :contentReference[oaicite:8]{index=8}
        if (len < 10) return std::nullopt;
        std::int16_t x_raw = le_i16(&data[4]);
        std::int16_t y_raw = le_i16(&data[6]);
        std::int16_t z_raw = le_i16(&data[8]);
        constexpr float SCALE = 1.0f / 256.0f; // Q8
        Vec3f v{
            x_raw * SCALE,
            y_raw * SCALE,
            z_raw * SCALE,
        };
        evt.sensor_id = Sh2SensorId::LinearAcceleration;
        evt.accel = v;
        break;
    }

    case 0x02: { // Gyroscope Calibrated (rad/s, Q9) :contentReference[oaicite:9]{index=9}
        if (len < 10) return std::nullopt;
        std::int16_t x_raw = le_i16(&data[4]);
        std::int16_t y_raw = le_i16(&data[6]);
        std::int16_t z_raw = le_i16(&data[8]);
        constexpr float SCALE = 1.0f / 512.0f; // Q9
        Vec3f v{
            x_raw * SCALE,
            y_raw * SCALE,
            z_raw * SCALE,
        };
        evt.sensor_id = Sh2SensorId::GyroscopeCalibrated;
        evt.gyro = v;
        break;
    }

    case 0x08: { // Game Rotation Vector (kwaternion Q14) :contentReference[oaicite:10]{index=10}
        if (len < 12) return std::nullopt;
        std::int16_t i_raw = le_i16(&data[4]);
        std::int16_t j_raw = le_i16(&data[6]);
        std::int16_t k_raw = le_i16(&data[8]);
        std::int16_t r_raw = le_i16(&data[10]);
        constexpr float SCALE = 1.0f / 16384.0f; // Q14
        Quaternion q{
            r_raw * SCALE, // real (w)
            i_raw * SCALE,
            j_raw * SCALE,
            k_raw * SCALE,
        };
        evt.sensor_id = Sh2SensorId::GameRotationVector;
        evt.game_quat = q;
        break;
    }

    default:
        // Inny raport – na razie nie obsługujemy.
        return std::nullopt;
    }

    return evt;
}

bool build_enable_report_command(Sh2SensorId sensor,
                                 std::uint32_t interval_us,
                                 std::uint8_t* out_buf,
                                 std::size_t& out_len,
                                 std::size_t max_len) {
    // Set Feature Command (0xFD) + Common Dynamic Feature Report (17 bajtów). :contentReference[oaicite:11]{index=11}
    if (!out_buf || max_len < 17) {
        return false;
    }

    std::memset(out_buf, 0, max_len);

    const std::uint8_t feature_report_id = static_cast<std::uint8_t>(sensor);

    out_buf[0] = 0xFD;                // Report ID = Set Feature Command
    out_buf[1] = feature_report_id;   // Feature Report ID
    out_buf[2] = 0x00;                // Feature flags (0 = non-wakeup)
    out_buf[3] = 0x00;                // Change sensitivity LSB
    out_buf[4] = 0x00;                // Change sensitivity MSB

    // Report Interval (4 bajty LE, w mikrosekundach)
    out_buf[5] = static_cast<std::uint8_t>(interval_us & 0xFF);
    out_buf[6] = static_cast<std::uint8_t>((interval_us >> 8) & 0xFF);
    out_buf[7] = static_cast<std::uint8_t>((interval_us >> 16) & 0xFF);
    out_buf[8] = static_cast<std::uint8_t>((interval_us >> 24) & 0xFF);

    // Batch Interval = 0 (bierzemy dane "na żywo")
    out_buf[9]  = 0;
    out_buf[10] = 0;
    out_buf[11] = 0;
    out_buf[12] = 0;

    // Sensor-specific config word = 0
    out_buf[13] = 0;
    out_buf[14] = 0;
    out_buf[15] = 0;
    out_buf[16] = 0;

    out_len = 17;
    return true;
}

} // namespace bno
