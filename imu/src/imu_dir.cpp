#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>

#include "bno/shtp.hpp"
#include "bno/sh2_reports.hpp"
#include "bno/gesture_dir.hpp"   // nasz detektor gestów

using namespace std::chrono_literals;

namespace {

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
    if (cfg.hz < 50 || cfg.hz > 100) {
        std::cerr << "hz must be in [50,100]\n";
        return false;
    }
    return true;
}

/// Dokładnie taki sam helper jak w imu_read.cpp:
/// wysyła Set Feature Command dla podanego raportu.
bool enable_report(bno::ShtpTransport& transport,
                   bno::Sh2SensorId sensor,
                   int hz,
                   bno::ShtpError& err)
{
    const std::uint32_t interval_us =
        static_cast<std::uint32_t>(1'000'000 / hz);

    std::uint8_t buf[32];
    std::size_t len = 0;
    if (!bno::build_enable_report_command(sensor, interval_us,
                                          buf, len, sizeof(buf))) {
        std::cerr << "build_enable_report_command failed\n";
        return false;
    }

    // Set Feature Command idzie na kanał kontrolny SH-2.
    if (!transport.write_frame(bno::ShtpChannel::Control, buf, len, err)) {
        std::cerr << "write_frame(SetFeature) failed: " << err.message
                  << " (errno=" << err.sys_errno << ")\n";
        return false;
    }

    return true;
}

} // namespace

int main(int argc, char** argv)
{
    CliConfig cfg;
    if (!parse_args(argc, argv, cfg)) {
        return 1;
    }

    bno::ShtpI2cTransport transport;
    bno::ShtpError err;

    if (!transport.open(cfg.bus, cfg.addr, err)) {
        std::cerr << "Failed to open I2C bus=" << cfg.bus
                  << " addr=0x" << std::hex << int(cfg.addr) << std::dec
                  << " : " << err.message << " (errno=" << err.sys_errno << ")\n";
        return 1;
    }

    // Tak jak w imu_read.cpp – dopiero po otwarciu:
    transport.set_max_frame_size(bno::SHTP_MAX_FRAME);

    // Uwaga: tutaj używamy TEJ SAMEJ funkcji enable_report, co w imu_read.cpp.
    // Włączamy tylko to, czego potrzebuje detektor:
    //  - Linear Acceleration (m/s^2)
    //  - Game Rotation Vector (kwaternion orientacji)
    if (!enable_report(transport, bno::Sh2SensorId::LinearAcceleration, cfg.hz, err)) {
        std::cerr << "Failed to enable Linear Accel\n";
    }
    if (!enable_report(transport, bno::Sh2SensorId::GameRotationVector, cfg.hz, err)) {
        std::cerr << "Failed to enable Game Rotation Vector\n";
    }

    // Konfiguracja detektora gestów – trochę poluzowane progi na start
    bno::GestureDirectionDetector::Config det_cfg;
    det_cfg.baseline_window_s    = 0.2;
    det_cfg.half_window_s        = 0.3;
    det_cfg.min_dyn_threshold    = 0.3; // było 0.5
    det_cfg.min_peak_magnitude   = 1.0; // było 1.5
    det_cfg.min_gesture_interval = 0.5; // było 0.8

    bno::GestureDirectionDetector detector(det_cfg);

    struct LastState {
        bool have_accel = false;
        bool have_quat  = false;
        bno::Vec3 last_accel{};
        bno::Quat last_quat{};
    } state;

    using clock = std::chrono::steady_clock;
    const auto t_start = clock::now();

    std::cerr << "imu_dir_cpp: running on bus " << cfg.bus
              << ", addr 0x" << std::hex << int(cfg.addr) << std::dec
              << ", hz=" << cfg.hz << "\n";

    // Statystyki debugowe
    std::uint64_t frames       = 0;
    std::uint64_t events       = 0;
    std::uint64_t accel_events = 0;
    std::uint64_t quat_events  = 0;
    std::uint64_t samples      = 0;
    std::uint64_t gestures     = 0;
    std::uint64_t timeouts     = 0;

    auto last_stats_print = clock::now();

    // Pętla główna
    while (true) {
        auto frame_opt = transport.read_frame(err, cfg.timeout_ms);
        if (!frame_opt) {
            ++timeouts;
        } else {
            const auto& frame = *frame_opt;
            ++frames;

            const auto ch = frame.header.channel;

            // Kanały z raportami SH-2 – jak w imu_read.cpp (2..5)
            if (ch >= 2 && ch <= 5) {
                const std::uint8_t* p   = frame.payload.data();
                std::size_t         len = frame.payload.size();

                // Obsługa 0xFB Base Timestamp Reference (jak w imu_read.cpp)
                if (len >= 5 && p[0] == 0xFB) {
                    p   += 5;
                    len -= 5;
                }
                if (len > 0) {
                    auto evt_opt = bno::parse_sh2_sensor_event(p, len);
                    if (evt_opt) {
                        ++events;
                        const auto& evt = *evt_opt;

                        if (evt.accel.has_value()) {
                            ++accel_events;
                            state.have_accel = true;
                            state.last_accel = bno::Vec3{
                                evt.accel->x,
                                evt.accel->y,
                                evt.accel->z,
                            };
                        }

                        if (evt.game_quat.has_value()) {
                            ++quat_events;
                            state.have_quat = true;
                            state.last_quat = bno::Quat{
                                evt.game_quat->real,
                                evt.game_quat->i,
                                evt.game_quat->j,
                                evt.game_quat->k,
                            };
                        }
                    }
                }
            }
        }

        // Jeśli mamy komplet (accel + quat), karmimy detektor
        if (state.have_accel && state.have_quat) {
            const auto now   = clock::now();
            const double t_s = std::chrono::duration<double>(now - t_start).count();

            detector.add_sample(t_s, state.last_accel, state.last_quat);
            ++samples;

            if (auto res_opt = detector.poll_result()) {
                ++gestures;
                const auto& res = *res_opt;

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

        // Co około 1 s wypisz statystyki na stderr
        const auto now_stats = clock::now();
        const double dt_stats =
            std::chrono::duration<double>(now_stats - last_stats_print).count();
        if (dt_stats > 1.0) {
            last_stats_print = now_stats;
            std::cerr
                << "[stats] frames="       << frames
                << " events="             << events
                << " accel_events="       << accel_events
                << " quat_events="        << quat_events
                << " samples="            << samples
                << " gestures="           << gestures
                << " timeouts="           << timeouts
                << "\n";
        }
    }

    return 0;
}
