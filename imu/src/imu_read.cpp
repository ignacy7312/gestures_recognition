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

using namespace std::chrono_literals;

namespace {

struct CliConfig {
    int bus = 1;
    std::uint8_t addr = 0x4A;
    int hz = 100;
    int timeout_ms = 50;
    bool header = true;
};

volatile std::sig_atomic_t g_stop = 0;

void signal_handler(int) {
    g_stop = 1;
}

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0 << " [options]\n"
              << "  --bus <int>           I2C bus (default 1)\n"
              << "  --addr <hex>          I2C address (default 0x4A)\n"
              << "  --hz <50..100>        Output rate (default 100)\n"
              << "  --timeout-ms <int>    I2C read timeout (default 50)\n"
              << "  --no-header           Do not print CSV header\n";
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

} // namespace

int main(int argc, char** argv) {
    CliConfig cfg;
    if (!parse_args(argc, argv, cfg)) {
        return 1;
    }

    std::signal(SIGINT, signal_handler);

    bno::ShtpI2cTransport transport;
    bno::ShtpError err;

    if (!transport.open(cfg.bus, cfg.addr, err)) {
        std::cerr << "Failed to open I2C bus=" << cfg.bus
                  << " addr=0x" << std::hex << int(cfg.addr) << std::dec
                  << " : " << err.message << " (errno=" << err.sys_errno << ")\n";
        return 1;
    }

    transport.set_max_frame_size(bno::SHTP_MAX_FRAME);

    if (cfg.header) {
        std::cout << "t,ax,ay,az,gx,gy,gz,qw,qi,qj,qk\n";
        std::cout.flush();
    }

    const auto period = std::chrono::duration<double>(1.0 / cfg.hz);
    auto t0 = std::chrono::steady_clock::now();

    std::size_t frames_total = 0;

    while (!g_stop) {
        auto frame_opt = transport.read_frame(err, cfg.timeout_ms);
        if (!frame_opt) {
            // timeout / error – w docelowej wersji tu wejdzie logika reinit/reset.
            continue;
        }

        ++frames_total;
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - t0).count();

        // TODO: zdekoduj payload SH-2 → accel/gyro/quat
        // Na razie wypisujemy same zera, żeby kontrakt CSV był zachowany.
        double ax = 0.0, ay = 0.0, az = 0.0;
        double gx = 0.0, gy = 0.0, gz = 0.0;
        double qw = 1.0, qi = 0.0, qj = 0.0, qk = 0.0;

        std::cout << t << ','
                  << ax << ',' << ay << ',' << az << ','
                  << gx << ',' << gy << ',' << gz << ','
                  << qw << ',' << qi << ',' << qj << ',' << qk << '\n';

        // Throttling do ~cfg.hz – docelowo zależeć powinno raczej od ustawień raportów.
        std::this_thread::sleep_for(period);
    }

    std::cerr << "Stopped, frames_total=" << frames_total << "\n";
    return 0;
}
