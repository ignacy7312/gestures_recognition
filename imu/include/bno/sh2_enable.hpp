#pragma once

#include "bno/shtp.hpp"      // transport
#include "bno/sh2_reports.hpp"

namespace bno {

// Set Feature Command ID
constexpr uint8_t SHTP_REPORT_SET_FEATURE_CMD = 0xFD;

// Funkcja pomocnicza: send SET_FEATURE command
inline bool sh2_set_feature(
        ShtpI2cTransport& transport,
        Sh2SensorId sensorId,
        uint16_t interval_us,    // period = 1e6 / hz
        ShtpError& err)
{
    uint8_t payload[17] = {};

    payload[0]  = SHTP_REPORT_SET_FEATURE_CMD;  // raport 0xFD
    payload[1]  = static_cast<uint8_t>(sensorId);                     // sensor ID
    payload[2]  = 0;                             // feature flags LSB
    payload[3]  = 0;                             // feature flags MSB

    // report interval w mikrosekundach
    payload[4]  = static_cast<uint8_t>(interval_us) & 0xFF;
    payload[5]  = (static_cast<uint8_t>(interval_us) >> 8) & 0xFF;
    payload[6]  = 0;
    payload[7]  = 0;

    // batch interval = 0
    payload[8]  = 0;
    payload[9]  = 0;
    payload[10] = 0;
    payload[11] = 0;

    // sensor-specific config (unused)
    payload[12] = payload[13] = payload[14] = payload[15] = payload[16] = 0;

    bool ok = transport.write_frame(
        ShtpChannel::Control,
        payload,
        sizeof(payload),
        err
    );

    return ok;
}


// Enable Accelerometer (raw accel)
inline bool enable_report_accel(
    ShtpI2cTransport& transport,
    int hz,
    ShtpError& err)
{
    const int period_us = 1000000 / hz;

    return sh2_set_feature(
        transport,
        Sh2SensorId::LinearAcceleration,       // from sh2_reports.hpp
        static_cast<uint16_t>(period_us),
        err
    );
}


// Enable Game Rotation Vector (quaternion)
inline bool enable_report_game_rv(
    ShtpI2cTransport& transport,
    int hz,
    ShtpError& err)
{
    const int period_us = 1000000 / hz;

    return sh2_set_feature(
        transport,
        Sh2SensorId::GameRotationVector,  // from sh2_reports.hpp
        static_cast<uint16_t>(period_us),
        err
    );
}

} // namespace bno
