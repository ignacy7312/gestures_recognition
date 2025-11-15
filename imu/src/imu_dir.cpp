#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>

#include "bno/shtp.hpp"
#include "bno/sh2_reports.hpp"
#include "bno/gesture_dir.hpp"
#include "bno/sh2_enable.hpp"


// Zakładam, że masz już funkcje typu:
// - enable_report(sensor_id, hz, transport, err) w imu_read_cpp
// - parse_sh2_sensor_event(...) w sh2_parser.cpp
// Tutaj je zadeklaruję i użyję; implementacja taka sama jak w imu_read_cpp.

namespace bno {

// Minimalne deklaracje – dopasuj do swoich nazw:
// struct AccelSample {
//     double x, y, z; // m/s^2
// };

// struct GameQuatSample {
//     double real;
//     double i, j, k;
// };

// struct Sh2SensorEvent {
//     std::optional<AccelSample>    accel;
//     std::optional<GameQuatSample> game_quat;
// };

// parser SH-2 – istnieje już w Twoim projekcie
// std::optional<Sh2SensorEvent> parse_sh2_sensor_event(const std::uint8_t* data, std::size_t len);

// enable feature – istnieje już w imu_read_cpp (tu tylko deklaracja)
// bool enable_report_accel(ShtpI2cTransport& transport, int hz, ShtpError& err);
// bool enable_report_game_rv(ShtpI2cTransport& transport, int hz, ShtpError& err);

} // namespace bno

struct CliConfig {
    int bus = 1;
    std::uint8_t addr = 0x4A;
    int hz = 100;
    int timeout_ms = 50;
};

static void print_usage(const char* argv0)
{
    std::cerr
        << "Usage: " << argv0 << " [options]\n"
        << "Options:\n"
        << "  --bus <int>        I2C bus (default 1)\n"
        << "  --addr <hex>       I2C address (default 0x4A)\n"
        << "  --hz <int>         Sampling rate (50..100, default 100)\n"
        << "  --timeout-ms <int> I2C read timeout (default 50)\n"
        << "  -h, --help         Show this help\n";
}

static bool parse_args(int argc, char** argv, CliConfig& cfg)
{
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--bus" && i + 1 < argc) {
            cfg.bus = std::atoi(argv[++i]);
        } else if (arg == "--addr" && i + 1 < argc) {
            cfg.addr = static_cast<std::uint8_t>(std::strtol(argv[++i], nullptr, 0));
        } else if (arg == "--hz" && i + 1 < argc) {
            cfg.hz = std::atoi(argv[++i]);
        } else if (arg == "--timeout-ms" && i + 1 < argc) {
            cfg.timeout_ms = std::atoi(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return false;
        } else {
            std::cerr << "Unknown arg: " << arg << "\n";
            print_usage(argv[0]);
            return false;
        }
    }
    if (cfg.hz < 50) cfg.hz = 50;
    if (cfg.hz > 100) cfg.hz = 100;
    return true;
}

int main(int argc, char** argv)
{
    CliConfig cfg;
    if (!parse_args(argc, argv, cfg)) {
        return 1;
    }

    bno::ShtpI2cTransport transport;
    bno::ShtpError err;
    transport.set_max_frame_size(512);

    if (!transport.open(cfg.bus, cfg.addr, err)) {
        std::cerr << "Failed to open I2C: " << err.message
                  << " (errno=" << err.sys_errno << ")\n";
        return 1;
    }

    // Enable potrzebnych raportów: Accel + Game Rotation Vector
    if (!bno::enable_report_accel(transport, cfg.hz, err)) {
        std::cerr << "Failed to enable Accel: " << err.message << "\n";
        return 1;
    }
    if (!bno::enable_report_game_rv(transport, cfg.hz, err)) {
        std::cerr << "Failed to enable GameRV: " << err.message << "\n";
        return 1;
    }

    // Detektor gestów – można potem dostroić parametry
    bno::GestureDirectionDetector::Config det_cfg;
    det_cfg.baseline_window_s    = 0.2;
    det_cfg.half_window_s        = 0.3;
    det_cfg.min_dyn_threshold    = 0.5;
    det_cfg.min_peak_magnitude   = 1.5;
    det_cfg.min_gesture_interval = 0.8;

    bno::GestureDirectionDetector detector(det_cfg);

    struct LastState {
        bool have_accel = false;
        bool have_quat  = false;
        bno::Vec3 last_accel{};
        bno::Quat last_quat{};
    };


    using clock = std::chrono::steady_clock;
    const auto t_start = clock::now();

    std::cerr << "imu_dir_cpp: running on bus " << cfg.bus
              << ", addr 0x" << std::hex << int(cfg.addr) << std::dec
              << ", hz=" << cfg.hz << "\n";


    LastState state;
    // Pętla główna: czytamy ramki SHTP z kanałów sensorowych
    while (true) {
        auto frame_opt = transport.read_frame(err, cfg.timeout_ms);
        if (!frame_opt) {
            // timeout – pomijamy, to normalne przy braku danych
            continue;
        }
        const auto& frame = *frame_opt;

        const auto ch = frame.header.channel;
        // Zakładamy, że raporty sensorowe lecą na kanałach 3 i/lub 5 (jak w imu_read_cpp)
        if (ch < 2 || ch > 5) {
            continue;
        }

        const std::uint8_t* p   = frame.payload.data();
        std::size_t         len = frame.payload.size();

        // Pomiń 0xFB Base Timestamp + 5 bajtów jak w imu_read_cpp
        if (len >= 5 && p[0] == 0xFB) {
            p   += 5;
            len -= 5;
        }
        if (len == 0) {
            continue;
        }

                auto evt_opt = bno::parse_sh2_sensor_event(p, len);
        if (!evt_opt) {
            continue;
        }
        const auto& evt = *evt_opt;

        // Uaktualnij ostatnie znane accel / quat
        if (evt.accel.has_value()) {
            state.have_accel = true;
            state.last_accel = bno::Vec3{
                evt.accel->x,
                evt.accel->y,
                evt.accel->z,
            };
        }

        if (evt.game_quat.has_value()) {
            state.have_quat = true;
            state.last_quat = bno::Quat{
                evt.game_quat->real,
                evt.game_quat->i,
                evt.game_quat->j,
                evt.game_quat->k,
            };
        }

        // Jeśli jeszcze nie mamy kompletnego zestawu (accel + quat), czekamy dalej
        if (!state.have_accel || !state.have_quat) {
            continue;
        }

        // Zbuduj próbkę z „ostatniego accel” i „ostatniego quat”
        const auto now   = clock::now();
        const double t_s = std::chrono::duration<double>(now - t_start).count();

        detector.add_sample(t_s, state.last_accel, state.last_quat);


        if (auto res_opt = detector.poll_result()) {
            const auto& res = *res_opt;

            // Prosty tekstowy output 1 linia = 1 gest
            std::cout
                << "t=" << res.t_center
                << " dir=" << res.label
                << " axis=" << res.axis << res.sign
                << " dv=(" << res.delta_v_world.x
                << "," << res.delta_v_world.y
                << "," << res.delta_v_world.z << ")"
                << " dur=" << res.duration
                << "\n";
            std::cout.flush();
        }
    }

    return 0;
}
