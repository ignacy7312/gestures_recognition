#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace bno {

/// Report IDs SH-2 (zgodne z tabelą 6.5.x SH-2 RM)
///  - 0x01: Accelerometer
///  - 0x02: Gyroscope Calibrated
///  - 0x04: Linear Acceleration
///  - 0x08: Game Rotation Vector
/// itd. :contentReference[oaicite:1]{index=1}
enum class Sh2SensorId : std::uint8_t {
    Accelerometer          = 0x01,
    GyroscopeCalibrated    = 0x02,
    LinearAcceleration     = 0x04,
    Gravity                = 0x06,
    GameRotationVector     = 0x08,

    // opcjonalne statusowe (na później):
    StepDetector           = 0x18,
    StepCounter            = 0x11,
    StabilityClassifier    = 0x13,
    ActivityClassifier     = 0x1E,
};

/// Wspólny wektor 3D (np. accel, gyro).
struct Vec3f {
    float x{};
    float y{};
    float z{};
};

/// Kwaternion jednostkowy (Game Rotation Vector).
struct Quaternion {
    float real{}; // w
    float i{};
    float j{};
    float k{};
};

/// Minimalny stan kalibracji / jakości (status SH-2).
/// W SH-2 „status” ma 2 bity dokładności: 0..3. :contentReference[oaicite:2]{index=2}
enum class Sh2Accuracy : std::uint8_t {
    Unreliable = 0,
    Low        = 1,
    Medium     = 2,
    High       = 3,
};

/// Jeden event sensora zinterpretowany przez parser SH-2.
struct Sh2SensorEvent {
    Sh2SensorId sensor_id{};
    std::uint32_t timestamp_us{0};  ///< dla większości raportów przetworzonych = 0
    Sh2Accuracy accuracy{Sh2Accuracy::Unreliable};

    // Dane numeryczne:
    std::optional<Vec3f> accel;            ///< accel lub linear accel (m/s^2)
    std::optional<Vec3f> gyro;             ///< gyro (rad/s)
    std::optional<Quaternion> game_quat;   ///< game rotation vector (unit)

    // Opcjonalne statusy – do rozwinięcia w kolejnych iteracjach.
    std::optional<std::string> activity_label;
    std::optional<int> activity_confidence; // 0..100
    std::optional<std::uint32_t> steps_total;
    std::optional<bool> step_event;
    std::optional<std::string> stability_state;
};

/// Dekoder SH-2 z payloadu SHTP → Sh2SensorEvent.
/// Obsługiwane raporty:
///   0x01 – Accelerometer (Q8, m/s^2)
///   0x04 – Linear Acceleration (Q8, m/s^2)
///   0x02 – Gyroscope Calibrated (Q9, rad/s)
///   0x08 – Game Rotation Vector (kwaternion Q14) :contentReference[oaicite:3]{index=3}
std::optional<Sh2SensorEvent> parse_sh2_sensor_event(const std::uint8_t* data,
                                                     std::size_t len);

/// Zbuduj komendę "Set Feature" (0xFD) dla danego raportu.
/// Wg SH-2: Set Feature Command = 0xFD + Common Dynamic Feature Report. :contentReference[oaicite:4]{index=4}
///   - featureReportId   = report ID (np. 0x04 dla Linear Accel)
///   - featureFlags      = 0 (non-wakeup)
///   - changeSensitivity = 0
///   - reportInterval    = interval_us (uint32 LE)
///   - batchInterval     = 0
///   - sensorConfigWord  = 0
bool build_enable_report_command(Sh2SensorId sensor,
                                 std::uint32_t interval_us,
                                 std::uint8_t* out_buf,
                                 std::size_t& out_len,
                                 std::size_t max_len);

} // namespace bno
