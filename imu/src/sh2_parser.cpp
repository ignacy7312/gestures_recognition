#include "bno/sh2_reports.hpp"

#include <cstring>

namespace bno {

std::optional<Sh2SensorEvent> parse_sh2_sensor_event(const std::uint8_t* data,
                                                     std::size_t len) {
    if (!data || len < 4) {
        return std::nullopt;
    }

    // Uwaga:
    // Layout raportów SH-2 jest szczegółowo opisany w "SH-2 Reference Manual".
    // Tutaj NIE zgadujemy offsetów – tylko szkicujemy strukturę kodu.
    //
    // Typowo:
    //  byte 0 : reportId
    //  byte 1 : status / sequence
    //  byte 2-3.. : timestamp (LSB-first)
    //  kolejne bajty : payload zależny od reportId (kwaternion, wektory itp.)
    //
    // TODO: Wypełnij dokładne pola wg tabel z SH-2 Reference Manual.

    Sh2SensorEvent evt{};

    const std::uint8_t report_id = data[0];
    (void)report_id;

    // Tutaj można mapować reportId -> Sh2SensorId oraz dekodować payload
    // w zależności od typu raportu. Na razie zostawiamy "event bez danych",
    // żeby kod budował się poprawnie.

    return evt;
}

bool build_enable_report_command(Sh2SensorId sensor,
                                 std::uint32_t interval_us,
                                 std::uint8_t* out_buf,
                                 std::size_t& out_len,
                                 std::size_t max_len) {
    if (!out_buf || max_len < 8) { // minimalny rozmiar (placeholder)
        return false;
    }

    // TODO:
    // SH-2 "Set Feature Command" – sekcja 6.x w SH-2 Reference Manual.
    // Ten payload zawiera m.in.:
    //  - Report ID komendy
    //  - Sensor ID
    //  - Interval_us (uint32)
    //  - Flags / Batch Interval / Sensor-Specific Config
    //
    // Tutaj tylko zostawiamy prosty placeholder,
    // żeby można było zawołać funkcję bez crasha.

    std::memset(out_buf, 0, max_len);
    out_buf[0] = 0; // placeholder: reportId komendy
    out_buf[1] = static_cast<std::uint8_t>(sensor);
    // tutaj można by zapisać interval_us itd.

    out_len = 8; // placeholder
    return true;
}

} // namespace bno
