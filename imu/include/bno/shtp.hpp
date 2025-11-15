#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace bno {

constexpr std::size_t SHTP_MAX_FRAME = 512;

/// SHTP frame header: length (LSB/MSB), channel, sequence.
/// length = header + payload (czyli >= 4).
struct ShtpHeader {
    std::uint16_t length_le{0};  ///< całkowita długość ramki (łącznie z 4B nagłówka)
    std::uint8_t  channel{0};
    std::uint8_t  sequence{0};
};

/// Kanały SHTP używane przez SH-2.
/// Konkretny mapping zweryfikuj w SH-2 Reference Manual.
enum class ShtpChannel : std::uint8_t {
    Command      = 0,  ///< komendy SH-2, feature requests itd.
    Control      = 1,  ///< sterowanie, sleep/wake, itp.
    SensorReport = 2,  ///< podstawowy kanał raportów sensorowych (SH-2)
    // Inne kanały na razie pomijamy (nie są potrzebne do MVP).
};

/// Prosty błąd transportu / protokołu.
struct ShtpError {
    enum class Code {
        None = 0,
        IoError,
        Timeout,
        OversizeFrame,
        InvalidHeader,
        DeviceReset,
        NotOpen,
        Unknown,
    };

    Code code{Code::None};
    int  sys_errno{0};
    std::string message;

    explicit operator bool() const noexcept { return code != Code::None; }
};

/// Pojedyncza ramka SHTP (payload bez 4-bajtowego nagłówka).
struct ShtpFrame {
    ShtpHeader header{};
    std::vector<std::uint8_t> payload;
};

/// Abstrakcyjny interfejs transportu – na MVP użyjemy implementacji Linux I2C.
class ShtpTransport {
public:
    virtual ~ShtpTransport() = default;

    /// Odczytaj jedną ramkę SHTP z urządzenia.
    virtual std::optional<ShtpFrame> read_frame(ShtpError& err, int timeout_ms) = 0;

    /// Wyślij jedną ramkę SHTP (payload + kanał; nagłówek dodawany wewnątrz).
    virtual bool write_frame(ShtpChannel channel,
                             const std::uint8_t* data,
                             std::size_t len,
                             ShtpError& err) = 0;

    /// Czy transport jest aktualnie otwarty.
    virtual bool is_open() const noexcept = 0;
};

/// Implementacja SHTP przez Linux i2c-dev (`/dev/i2c-N`).
/// Zaprojektowana pod Raspberry Pi 3, zgodnie z notami Adafruit
/// rekomendującymi 400 kHz I2C dla BNO08x. :contentReference[oaicite:0]{index=0}
class ShtpI2cTransport final : public ShtpTransport {
public:
    ShtpI2cTransport() = default;
    ~ShtpI2cTransport() override;

    /// Otwórz `/dev/i2c-<bus>` i ustaw adres.
    bool open(int bus, std::uint8_t addr, ShtpError& err);

    /// Zamknij, jeśli otwarte.
    void close() noexcept;

    bool is_open() const noexcept override { return fd_ >= 0; }

    std::optional<ShtpFrame> read_frame(ShtpError& err, int timeout_ms) override;
    bool write_frame(ShtpChannel channel,
                     const std::uint8_t* data,
                     std::size_t len,
                     ShtpError& err) override;

    /// Ustaw maksymalny rozmiar ramki (łącznie z nagłówkiem).
    void set_max_frame_size(std::size_t bytes) { max_frame_size_ = bytes; }

private:
    int fd_{-1};
    std::uint8_t addr_{0};
    std::array<std::uint8_t, SHTP_MAX_FRAME> rx_buf_{};
    std::array<std::uint8_t, SHTP_MAX_FRAME> tx_buf_{};
    std::size_t max_frame_size_{SHTP_MAX_FRAME};

    std::array<std::uint8_t, 8> sequence_per_channel_{}; // sequence++ per channel

    bool read_exact(std::uint8_t* buf, std::size_t len, int timeout_ms, ShtpError& err);
    bool write_exact(const std::uint8_t* buf, std::size_t len, ShtpError& err);
};

} // namespace bno
