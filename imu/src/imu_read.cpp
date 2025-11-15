#include "bno/shtp.hpp"
#include "bno/sh2_reports.hpp"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <fstream>

using namespace std::chrono_literals;

namespace {

struct CliConfig {
    int bus = 1;
    std::uint8_t addr = 0x4A;
    int hz = 100;
    int timeout_ms = 50;
    bool header = true;
    std::string out_path = "dupa.csv";
};

volatile std::sig_atomic_t g_stop = 0;

void signal_handler(int) {
    g_stop = 1;
}

void print_usage(const char* argv0) {
    std::cout << "Usage: " << argv0 << " [options]\n"
              << "  --bus <int>           I2C bus (default 1)\n"
              << "  --addr <hex>          I2C address (default 0x4A)\n"
              << "  --hz <50..100>        Output rate (default 100)\n"
              << "  --timeout-ms <int>    I2C read timeout (default 50)\n"
              << "  --no-header           Do not print CSV header\n"
              << "  --out <path>          Write CSV data to file instead of stdout\n";
}

bool parse_args(int argc, char** argv, CliConfig& cfg) {
    for (int i = 1; i < argc; ++i) {
        std::string_view arg{argv[i]};
        if (arg == "--bus" && i + 1 < argc) {
            cfg.bus = std::atoi(argv[++i]);
        } else if (arg == "--addr" && i + 1 < argc) {
            cfg.addr = static_cast<std::uint8_t>(std::strtol(argv[++i], nullptr, 0));
        } else if (arg == "--hz" && i + 1 < argc) {
            cfg.hz = std::atoi(argv[++i]);
        } else if (arg == "--timeout-ms" && i + 1 < argc) {
            cfg.timeout_ms = std::atoi(argv[++i]);
        } else if (arg == "--no-header") {
            cfg.header = false;
        } else if (arg == "--out" && i + 1 < argc) {
            cfg.out_path = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return false;
        } else {
            std::cout << "Unknown arg: " << arg << "\n";
            print_usage(argv[0]);
            return false;
        }
    }
    if (cfg.hz < 50 || cfg.hz > 100) {
        std::cout << "hz must be in [50,100]\n";
        return false;
    }
    return true;
}

/// Helper: wyślij Set Feature Command dla danego raportu.
bool enable_report(bno::ShtpTransport& transport,
                   bno::Sh2SensorId sensor,
                   int hz,
                   bno::ShtpError& err) {
    const std::uint32_t interval_us = static_cast<std::uint32_t>(1'000'000 / hz);

    std::uint8_t buf[32];
    std::size_t len = 0;
    if (!bno::build_enable_report_command(sensor, interval_us, buf, len, sizeof(buf))) {
        std::cout << "build_enable_report_command failed\n";
        return false;
    }

    // Set Feature Command idzie na kanał kontrolny SH-2. :contentReference[oaicite:12]{index=12}
    if (!transport.write_frame(bno::ShtpChannel::Control, buf, len, err)) {
        std::cout << "write_frame(SetFeature) failed: " << err.message
                  << " (errno=" << err.sys_errno << ")\n";
        return false;
    }

    return true;
}

} // namespace

int main(int argc, char** argv) {
    CliConfig cfg;
    if (!parse_args(argc, argv, cfg)) {
        return 1;
    }

    std::signal(SIGINT, signal_handler);

    bno::ShtpI2cTransport transport;
    bno::ShtpError err;

    // Strumień wyjściowy dla danych: stdout lub plik
    std::ofstream file_out;
    std::ostream* data_out = &std::cout;

    if (!cfg.out_path.empty()) {
        file_out.open(cfg.out_path, std::ios::out | std::ios::trunc);
        if (!file_out) {
            std::cerr << "Failed to open output file: " << cfg.out_path << "\n";
            return 1;
        }
        data_out = &file_out;
    }

    if (!transport.open(cfg.bus, cfg.addr, err)) {
        std::cout << "Failed to open I2C bus=" << cfg.bus
                  << " addr=0x" << std::hex << int(cfg.addr) << std::dec
                  << " : " << err.message << " (errno=" << err.sys_errno << ")\n";
        return 1;
    }

    transport.set_max_frame_size(bno::SHTP_MAX_FRAME);

    // Włączamy raporty, których potrzebujemy:
    //  - Linear Accel (preferowane do ax/ay/az)
    //  - Accelerometer (fallback)
    //  - Gyro Calibrated
    //  - Game Rotation Vector
    if (!enable_report(transport, bno::Sh2SensorId::LinearAcceleration, cfg.hz, err)) {
        std::cout << "Failed to enable Linear Accel\n";
    }
    if (!enable_report(transport, bno::Sh2SensorId::Accelerometer, cfg.hz, err)) {
        std::cout << "Failed to enable Accelerometer\n";
    }
    if (!enable_report(transport, bno::Sh2SensorId::GyroscopeCalibrated, cfg.hz, err)) {
        std::cout << "Failed to enable Gyro Calibrated\n";
    }
    if (!enable_report(transport, bno::Sh2SensorId::GameRotationVector, cfg.hz, err)) {
        std::cout << "Failed to enable Game Rotation Vector\n";
    }

    if (cfg.header) {
        *data_out << "t,ax,ay,az,gx,gy,gz,qw,qi,qj,qk\n";
        data_out->flush();
    }


    const auto period = std::chrono::duration<double>(1.0 / cfg.hz);
    auto t0 = std::chrono::steady_clock::now();

    std::size_t frames_total = 0;

    // Aktualny stan (ostatnie wartości z poszczególnych raportów)
    double ax = 0.0, ay = 0.0, az = 0.0;
    double gx = 0.0, gy = 0.0, gz = 0.0;
    double qw = 1.0, qi = 0.0, qj = 0.0, qk = 0.0;

    while (!g_stop) {
        auto frame_opt = transport.read_frame(err, cfg.timeout_ms);
        if (!frame_opt) {
            // timeout / error – w docelowej wersji tu wejdzie logika reinit/reset.
            continue;
        }

        const auto& frame = *frame_opt;

       const auto ch = frame.header.channel;

        // Interesują nas tylko kanały z raportami SH-2 (normal + gyroRV).
        if (ch < 2 || ch > 5) {
            continue;
        }

        // --- NOWE: obsługa 0xFB Base Timestamp Reference ---
        // BNO08x na kanale 3 potrafi wysłać:
        //   [0xFB, baseDelta(4 bajty), 0x01/0x02/0x04/0x08, ...]
        // Nasz parser oczekuje, że data[0] to już report ID sensora,
        // więc jeśli widzimy 0xFB, pomijamy pierwszych 5 bajtów.

        const std::uint8_t* p   = frame.payload.data();
        std::size_t         len = frame.payload.size();

        if (len >= 5 && p[0] == 0xFB) {
            // 0xFB = Base Timestamp Reference (5 bajtów)
            p   += 5;
            len -= 5;
        }

        // Jeśli po bazie czasu nie ma już nic – nie ma co parsować.
        if (len == 0) {
            continue;
        }

        auto evt_opt = bno::parse_sh2_sensor_event(p, len);

        if (!evt_opt) {
            // DEBUG: teraz logujemy krótszy payload (już *po* ewentualnym 0xFB)
            std::cerr << "[imu_read_cpp] unknown sensor report on ch=" << int(ch)
                    << " len=" << len << " :";
            for (std::size_t i = 0; i < len && i < 16; ++i) {
                std::cerr << " " << std::hex << int(p[i]) << std::dec;
            }
            std::cerr << "\n";
            continue;
        }

        const auto& evt = *evt_opt;

        if (evt.accel.has_value()) {
            ax = evt.accel->x;
            ay = evt.accel->y;
            az = evt.accel->z;
        }

        if (evt.gyro.has_value()) {
            gx = evt.gyro->x;
            gy = evt.gyro->y;
            gz = evt.gyro->z;
        }

        if (evt.game_quat.has_value()) {
            qw = evt.game_quat->real;
            qi = evt.game_quat->i;
            qj = evt.game_quat->j;
            qk = evt.game_quat->k;
        }


        ++frames_total;
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - t0).count();

        *data_out << t << ','
          << ax << ',' << ay << ',' << az << ','
          << gx << ',' << gy << ',' << gz << ','
          << qw << ',' << qi << ',' << qj << ',' << qk << '\n';


        // Throttling do ~cfg.hz (dane mogą realnie napływać z taką samą lub większą częstotliwością)
        std::this_thread::sleep_for(period);
    }

    std::cout << "Stopped, frames_total=" << frames_total << "\n";
    return 0;
}

