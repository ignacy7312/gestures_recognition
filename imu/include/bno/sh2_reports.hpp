#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace bno {

/// SH-2 sensor IDs – TODO: potwierdź wartości w SH-2 Reference Manual.
enum class Sh2SensorId : std::uint8_t {
    Accelerometer      = 0x01, // TODO verify
    Gyroscope          = 0x02, // TODO verify
    LinearAcceleration = 0x04, // TODO verify
    GameRotationVector = 0x08, // Game Rotation Vector wg SH-2 manual :contentReference[oaicite:2]{index=2}
    // opcjonalne:
    StepDetector       = 0x10, // TODO verify
    StepCounter        = 0x11, // TODO verify
    StabilityClassifier= 0x1A, // TODO verify
    ActivityClassifier = 0x1E, // TODO verify
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
enum class Sh2Accuracy : std::uint8_t {
    Unreliable = 0,
    Low        = 1,
    Medium     = 2,
    High       = 3,
};

/// Jeden event sensora zinterpreowany przez parser SH-2.
struct Sh2SensorEvent {
    Sh2SensorId sensor_id{};
    std::uint32_t timestamp_us{0};  ///< timestamp z SH-2 (host interpretacja)
    Sh2Accuracy accuracy{Sh2Accuracy::Unreliable};

    std::optional<Vec3f> accel;            ///< accel / linear accel
    std::optional<Vec3f> gyro;             ///< gyro
    std::optional<Quaternion> game_quat;   ///< game rotation vector

    // Opcjonalne statusy – do rozwinięcia w kolejnych iteracjach.
    std::optional<std::string> activity_label;
    std::optional<int> activity_confidence; // 0..100
    std::optional<std::uint32_t> steps_total;
    std::optional<bool> step_event;
    std::optional<std::string> stability_state;
};

/// Dekoder SH-2 z payloadu SHTP → Sh2SensorEvent.
/// Szczegółowy layout raportu trzeba wypełnić na podstawie SH-2 Reference Manual.
/// Na razie interfejs, implementacja w `sh2_parser.cpp`.
std::optional<Sh2SensorEvent> parse_sh2_sensor_event(const std::uint8_t* data,
                                                     std::size_t len);

/// Zbuduj komendę "enable report" dla danego sensora.
/// Zgodnie z SH-2: payload zawiera m.in. Report ID, Sensor ID, interval_us.
/// Implementacja w `sh2_parser.cpp`.
bool build_enable_report_command(Sh2SensorId sensor,
                                 std::uint32_t interval_us,
                                 std::uint8_t* out_buf,
                                 std::size_t& out_len,
                                 std::size_t max_len);

} // namespace bno
