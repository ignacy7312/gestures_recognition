#include "bno/shtp.hpp"

#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>

namespace bno {

namespace {

void fill_io_error(ShtpError& err, ShtpError::Code code, const char* msg) {
    err.code = code;
    err.sys_errno = errno;
    err.message = msg ? msg : "";
}

} // namespace

ShtpI2cTransport::~ShtpI2cTransport() {
    close();
}

bool ShtpI2cTransport::open(int bus, std::uint8_t addr, ShtpError& err) {
    close();

    std::string dev = "/dev/i2c-" + std::to_string(bus);
    fd_ = ::open(dev.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) {
        fill_io_error(err, ShtpError::Code::IoError, "open /dev/i2c-X failed");
        return false;
    }

    if (ioctl(fd_, I2C_SLAVE, addr) < 0) {
        fill_io_error(err, ShtpError::Code::IoError, "ioctl(I2C_SLAVE) failed");
        close();
        return false;
    }

    addr_ = addr;
    err = ShtpError{};
    return true;
}

void ShtpI2cTransport::close() noexcept {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool ShtpI2cTransport::read_exact(std::uint8_t* buf,
                                  std::size_t len,
                                  int timeout_ms,
                                  ShtpError& err) {
    if (fd_ < 0) {
        fill_io_error(err, ShtpError::Code::NotOpen, "transport not open");
        return false;
    }

    using clock = std::chrono::steady_clock;
    auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);

    std::size_t offset = 0;
    while (offset < len) {
        auto now = clock::now();
        if (timeout_ms > 0 && now >= deadline) {
            fill_io_error(err, ShtpError::Code::Timeout, "read timeout");
            return false;
        }

        ssize_t n = ::read(fd_, buf + offset, len - offset);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                continue;
            }
            fill_io_error(err, ShtpError::Code::IoError, "read failed");
            return false;
        }
        if (n == 0) {
            fill_io_error(err, ShtpError::Code::IoError, "read EOF");
            return false;
        }
        offset += static_cast<std::size_t>(n);
    }

    err = ShtpError{};
    return true;
}

bool ShtpI2cTransport::write_exact(const std::uint8_t* buf,
                                   std::size_t len,
                                   ShtpError& err) {
    if (fd_ < 0) {
        fill_io_error(err, ShtpError::Code::NotOpen, "transport not open");
        return false;
    }

    std::size_t offset = 0;
    while (offset < len) {
        ssize_t n = ::write(fd_, buf + offset, len - offset);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                continue;
            }
            fill_io_error(err, ShtpError::Code::IoError, "write failed");
            return false;
        }
        if (n == 0) {
            fill_io_error(err, ShtpError::Code::IoError, "write zero bytes");
            return false;
        }
        offset += static_cast<std::size_t>(n);
    }

    err = ShtpError{};
    return true;
}

std::optional<ShtpFrame> ShtpI2cTransport::read_frame(ShtpError& err, int timeout_ms) {
    if (fd_ < 0) {
        fill_io_error(err, ShtpError::Code::NotOpen, "transport not open");
        return std::nullopt;
    }

    // 1. Najpierw 4 bajty nagłówka
    if (!read_exact(rx_buf_.data(), 4, timeout_ms, err)) {
        return std::nullopt;
    }

    ShtpHeader header{};
    header.length_le = static_cast<std::uint16_t>(rx_buf_[0] | (rx_buf_[1] << 8));
    header.channel   = rx_buf_[2];
    header.sequence  = rx_buf_[3];

    if (header.length_le < 4 || header.length_le > max_frame_size_) {
        fill_io_error(err, ShtpError::Code::OversizeFrame, "invalid SHTP length");
        return std::nullopt;
    }

    const std::size_t payload_len = header.length_le - 4;
    if (!read_exact(rx_buf_.data() + 4, payload_len, timeout_ms, err)) {
        return std::nullopt;
    }

    ShtpFrame frame;
    frame.header = header;
    frame.payload.assign(rx_buf_.data() + 4, rx_buf_.data() + 4 + payload_len);
    err = ShtpError{};
    return frame;
}

bool ShtpI2cTransport::write_frame(ShtpChannel channel,
                                   const std::uint8_t* data,
                                   std::size_t len,
                                   ShtpError& err) {
    if (len + 4 > max_frame_size_) {
        fill_io_error(err, ShtpError::Code::OversizeFrame, "frame too large");
        return false;
    }
    if (fd_ < 0) {
        fill_io_error(err, ShtpError::Code::NotOpen, "transport not open");
        return false;
    }

    // SHTP header
    const std::uint16_t total_len = static_cast<std::uint16_t>(len + 4);
    tx_buf_[0] = static_cast<std::uint8_t>(total_len & 0xFF);
    tx_buf_[1] = static_cast<std::uint8_t>((total_len >> 8) & 0xFF);
    const auto ch = static_cast<std::uint8_t>(channel);
    tx_buf_[2] = ch;

    std::uint8_t& seq = sequence_per_channel_[ch];
    tx_buf_[3] = seq++;
    if (seq == 0) seq = 0; // naturalny overflow uint8_t

    if (len > 0 && data != nullptr) {
        std::memcpy(tx_buf_.data() + 4, data, len);
    }

    return write_exact(tx_buf_.data(), total_len, err);
}

} // namespace bno
