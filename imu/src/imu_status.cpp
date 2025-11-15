#include "bno/shtp.hpp"
#include "bno/sh2_reports.hpp"

#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

namespace {

struct CliConfig {
    int bus = 1;
    std::uint8_t addr = 0x4A;
    int hz = 50;
    int duration_s = 0; // 0 = nieskończenie
    bool json = false;
};

volatile std::sig_atomic_t g_stop = 0;

void signal_handler(int) {
    g_stop = 1;
}

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0 << " [options]\n"
              << "  --bus <int>         I2C bus (default 1)\n"
              << "  --addr <hex>        I2C address (default 0x4A)\n"
              << "  --hz <int>          Poll rate (default 50)\n"
              << "  --duration <sec>    Duration seconds (0 = infinite)\n"
              << "  --json              Output NDJSON\n";
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
        } else if (arg == "--duration" && i + 1 < argc) {
            cfg.duration_s = std::atoi(argv[++i]);
        } else if (arg == "--json") {
            cfg.json = true;
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return false;
        } else {
            std::cerr << "Unknown arg: " << arg << "\n";
            print_usage(argv[0]);
            return false;
        }
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
        std::cerr << "Failed to open I2C: " << err.message
                  << " (errno=" << err.sys_errno << ")\n";
        return 1;
    }

    // TODO: enable reports dla Activity/Steps/Stability.

    std::size_t count = 0;
    while (!g_stop) {
        auto frame_opt = transport.read_frame(err, 50);
        if (!frame_opt) {
            continue;
        }

        // TODO: parsowanie raportów statusowych (Activity/Steps/Stability).
        // Na razie placeholder żeby pokazać strukturę NDJSON.

        if (cfg.json) {
            std::cout
                << "{\"t\":" << count
                << ",\"activity_label\":null"
                << ",\"activity_conf\":null"
                << ",\"steps_total\":null"
                << ",\"step_event\":null"
                << ",\"stability_state\":null"
                << ",\"calib_state\":null"
                << ",\"notes\":\"placeholder\""
                << "}\n";
        } else {
            std::cout << "[t=" << count << "] activity=?, steps=?, stability=?, calib=?\n";
        }

        ++count;
        if (cfg.duration_s > 0 && static_cast<int>(count / cfg.hz) >= cfg.duration_s) {
            break;
        }
    }

    return 0;
}
