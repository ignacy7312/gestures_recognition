#include "bno/shtp.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace bno {

namespace {

std::string make_i2c_dev_path(int bus) {
    return "/dev/i2c-" + std::to_string(bus);
}

} // namespace

ShtpI2cTransport::~ShtpI2cTransport() {
    close();
}

bool ShtpI2cTransport::open(int bus, std::uint8_t addr, ShtpError& err) {
    close();

    std::string path = make_i2c_dev_path(bus);
    int fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        err.code      = ShtpError::Code::IoError;
        err.sys_errno = errno;
        err.message   = "open(" + path + ") failed";
        return false;
    }

    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        err.code      = ShtpError::Code::IoError;
        err.sys_errno = errno;
        err.message   = "ioctl(I2C_SLAVE) failed";
        ::close(fd);
        return false;
    }

    fd_   = fd;
    addr_ = addr;
    err   = ShtpError{};
    return true;
}

void ShtpI2cTransport::close() noexcept {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

///
/// Czytanie ramki SHTP po I²C.
/// Schemat:
///   1. poll() z timeoutem,
///   2. read(4) → nagłówek (Length[2], Channel, Sequence),
///   3. wyliczamy length = Length & 0x7FFF,
///   4. read(length) → cała ramka (nagłówek + payload),
///   5. wypełniamy ShtpFrame.
///
std::optional<ShtpFrame> ShtpI2cTransport::read_frame(ShtpError& err,
                                                      int timeout_ms) {
    if (fd_ < 0) {
        err.code      = ShtpError::Code::NotOpen;
        err.sys_errno = EBADF;
        err.message   = "I2C not open";
        return std::nullopt;
    }

    // 1. poll() na fd z timeoutem
    struct pollfd pfd;
    pfd.fd     = fd_;
    pfd.events = POLLIN;

    int rv = ::poll(&pfd, 1, timeout_ms);
    if (rv == 0) {
        // timeout – brak ramki to nie błąd krytyczny
        err = ShtpError{};
        return std::nullopt;
    }
    if (rv < 0) {
        err.code      = ShtpError::Code::IoError;
        err.sys_errno = errno;
        err.message   = "poll() failed";
        return std::nullopt;
    }

    std::array<std::uint8_t, SHTP_MAX_FRAME> buf{};

    // 2. pierwszy odczyt – 4 bajty nagłówka
    std::uint8_t header_raw[4];
    ssize_t n = ::read(fd_, header_raw, 4);
    if (n < 0) {
        err.code      = ShtpError::Code::IoError;
        err.sys_errno = errno;
        err.message   = "read(header) failed";
        return std::nullopt;
    }
    if (n == 0) {
        // nic na szynie – traktujemy jak brak ramki
        err = ShtpError{};
        return std::nullopt;
    }
    if (n != 4) {
        err.code      = ShtpError::Code::IoError;
        err.sys_errno = EIO;
        err.message   = "short read(header)";
        return std::nullopt;
    }

    std::uint16_t length = std::uint16_t(header_raw[0] |
                                         (std::uint16_t(header_raw[1]) << 8));
    length &= 0x7FFF; // bez bitu kontynuacji

    if (length < 4 || length > max_frame_size_) {
        err.code      = ShtpError::Code::OversizeFrame;
        err.sys_errno = EPROTO;
        err.message   = "invalid SHTP length=" + std::to_string(length);
        return std::nullopt;
    }

    // 3. drugi odczyt – *cała* ramka length bajtów
    n = ::read(fd_, buf.data(), length);
    if (n < 0) {
        err.code      = ShtpError::Code::IoError;
        err.sys_errno = errno;
        err.message   = "read(frame) failed";
        return std::nullopt;
    }
    if (static_cast<std::size_t>(n) != length) {
        err.code      = ShtpError::Code::IoError;
        err.sys_errno = EIO;
        err.message   = "short read(frame)";
        return std::nullopt;
    }

    // 4. parsujemy nagłówek z buf
    ShtpHeader hdr{};
    std::uint16_t length2 = std::uint16_t(buf[0] |
                              (std::uint16_t(buf[1]) << 8));
    length2 &= 0x7FFF;

    if (length2 != length) {
        // coś dziwnego – log i odrzucamy tę ramkę
        std::cerr << "[shtp] length mismatch header=" << length2
                  << " second_read=" << length << "\n";
        err.code      = ShtpError::Code::InvalidHeader;
        err.sys_errno = EPROTO;
        err.message   = "length mismatch";
        return std::nullopt;
    }

    hdr.length_le = length2;
    hdr.channel   = buf[2];
    hdr.sequence  = buf[3];

    ShtpFrame frame;
    frame.header = hdr;

    const std::size_t payload_len = length2 - 4;
    frame.payload.resize(payload_len);
    if (payload_len > 0) {
        std::memcpy(frame.payload.data(), buf.data() + 4, payload_len);
    }

    err = ShtpError{};
    return frame;
}

///
/// Zapisywanie ramki:
///  - wypełniamy lokalny bufor: [len_lo, len_hi, channel, sequence, payload…]
///  - jeden write() na I²C.
///
bool ShtpI2cTransport::write_frame(ShtpChannel ch,
                                   const std::uint8_t* payload,
                                   std::size_t payload_len,
                                   ShtpError& err) {
    if (fd_ < 0) {
        err.code      = ShtpError::Code::NotOpen;
        err.sys_errno = EBADF;
        err.message   = "I2C not open";
        return false;
    }

    const std::size_t total_len = 4 + payload_len;
    if (total_len > max_frame_size_) {
        err.code      = ShtpError::Code::OversizeFrame;
        err.sys_errno = EMSGSIZE;
        err.message   = "payload too large";
        return false;
    }

    std::array<std::uint8_t, SHTP_MAX_FRAME> buf{};

    std::uint16_t length = static_cast<std::uint16_t>(total_len);
    buf[0] = static_cast<std::uint8_t>(length & 0xFF);
    buf[1] = static_cast<std::uint8_t>((length >> 8) & 0x7F); // bez bitu kontynuacji
    std::uint8_t ch_byte = static_cast<std::uint8_t>(ch);
    buf[2] = ch_byte;

    // sequence per channel – korzystamy z istniejącej tablicy sequence_per_channel_
    std::uint8_t& seq = sequence_per_channel_[ch_byte];
    buf[3] = seq++;
    // overflow uint8_t jest OK

    if (payload_len > 0 && payload != nullptr) {
        std::memcpy(buf.data() + 4, payload, payload_len);
    }

    ssize_t n = ::write(fd_, buf.data(), total_len);
    if (n < 0) {
        err.code      = ShtpError::Code::IoError;
        err.sys_errno = errno;
        err.message   = "write() failed";
        return false;
    }
    if (static_cast<std::size_t>(n) != total_len) {
        err.code      = ShtpError::Code::IoError;
        err.sys_errno = EIO;
        err.message   = "short write()";
        return false;
    }

    err = ShtpError{};
    return true;
}

} // namespace bno
