/**
 * @file E07400MM.cpp
 * @brief E07400MM (CC1101 clone) driver implementation tested with E07-400MM10S (https://www.cdebyte.com/products/E07-400MM10S)
 *
 * @author 0x42en (0x42en@gmail.com)
 * @date 2025-01-08
 */

#include "E07400MM.h"

E07_400MM::E07_400MM(Module *module) : CC1101(module)
{
    // Constructor - just call parent
}

float E07_400MM::convertRSSI(uint8_t rssi_raw)
{
    // CC1101 RSSI conversion formula (from datasheet)
    if (rssi_raw >= 128)
    {
        return ((rssi_raw - 256) / 2.0f) - 74.0f;
    }
    else
    {
        return (rssi_raw / 2.0f) - 74.0f;
    }
}

float E07_400MM::getRSSI()
{
    // CRITICAL: E07-400MM requires BURST mode for status registers (0x30-0x3D)
    // RadioLib's getRSSI() uses SPIreadRegister() which doesn't work
    // RSSI register is 0x34, BURST bit is 0x40 (read) or 0xC0 (read + burst)

    uint8_t rssi_raw;
    SPIreadRegisterBurst(0x34 | 0xC0, 1, &rssi_raw);

    return convertRSSI(rssi_raw);
}

size_t E07_400MM::getPacketLength(bool update)
{
    // Return cached value if update not requested
    if (!update && this->packetLengthQueried)
    {
        return this->packetLength;
    }

    // CRITICAL: E07-400MM requires BURST mode for FIFO register (0x3F)
    // In variable length mode, first byte in FIFO = packet length
    // RadioLib's implementation uses SPIreadRegister() which fails

    uint8_t packet_length;
    SPIreadRegisterBurst(0x3F | 0x40, 1, &packet_length);

    // Cache the result
    this->packetLength = packet_length;
    this->packetLengthQueried = true;

    return packet_length;
}

int16_t E07_400MM::readData(uint8_t *data, size_t len)
{
    // Reset query flag
    this->packetLengthQueried = false;

    // Read packet length from FIFO (BURST mode)
    uint8_t packet_length = getPacketLength();

    // Sanity check
    if (packet_length == 0 || packet_length > RADIOLIB_CC1101_MAX_PACKET_LENGTH)
    {
        // Flush RX FIFO and restart
        SPIsendCommand(RADIOLIB_CC1101_CMD_FLUSH_RX);
        startReceive();

        return RADIOLIB_ERR_INVALID_PAYLOAD;
    }

    // Limit read length to buffer size
    size_t read_length = (packet_length < len) ? packet_length : len;

    // Read packet data from FIFO (BURST mode required!)
    SPIreadRegisterBurst(0x3F | 0x40, read_length, data);

    // Read RSSI and LQI status bytes (2 bytes appended by CC1101 in variable length mode)
    uint8_t rssi_raw, lqi_crc;
    SPIreadRegisterBurst(0x3F | 0x40, 1, &rssi_raw);
    SPIreadRegisterBurst(0x3F | 0x40, 1, &lqi_crc);

    // Store for later retrieval via getRSSI() and getLQI()
    this->rawRSSI = rssi_raw;
    this->rawLQI = lqi_crc & 0x7F;

    // Check CRC (bit 7 of LQI byte)
    bool crc_ok = (lqi_crc & 0x80) != 0;

    if (!crc_ok && this->crcOn)
    {
        startReceive(); // Restart listening
        return RADIOLIB_ERR_CRC_MISMATCH;
    }

    // Restart listening for next packet
    startReceive();

    return RADIOLIB_ERR_NONE;
}

int16_t E07_400MM::scanRSSI(float *rssi_values, size_t num_points,
                            float center_freq, float step_khz,
                            uint16_t dwell_time_us)
{
    // Validate parameters
    if (rssi_values == nullptr || num_points == 0)
    {
        return RADIOLIB_ERR_INVALID_RSSI_OFFSET;
    }

    // Validate dwell time (must be reasonable)
    if (dwell_time_us < 500)
    {
        dwell_time_us = 500;
    }
    else if (dwell_time_us > 50000)
    {
        dwell_time_us = 50000;
    }

    // Calculate frequency range: num_points centered on center_freq
    // Start = center - (num_points/2 × step), End = center + (num_points/2 × step)
    float start_freq = center_freq - ((num_points / 2.0f) * (step_khz / 1000.0f));
    float step_mhz = step_khz / 1000.0f;

    // Enter RX mode once (optimization - avoid mode switching overhead)
    SPIsendCommand(RADIOLIB_CC1101_CMD_RX);

    // Scan all frequency points
    float current_freq = start_freq;
    for (size_t i = 0; i < num_points; i++)
    {
        // Set frequency using RadioLib method (handles all register calculations)
        int16_t state = setFrequency(current_freq);

        if (state != RADIOLIB_ERR_NONE)
        {
            rssi_values[i] = -999.0f; // Mark as invalid
            current_freq += step_mhz;
            continue;
        }

// Wait for AGC settling time
// CRITICAL: Must use esp_rom_delay_us() because FreeRTOS tick is 10ms
// CC1101 AGC settling time is typically 2-3ms according to datasheet
#if defined(ARDUINO)
        delayMicroseconds(dwell_time_us);
#elif defined(ESP_PLATFORM)
        esp_rom_delay_us(dwell_time_us);
#else
        // Fallback: busy-wait loop (not accurate, but prevents build errors)
        volatile uint32_t count = dwell_time_us * 10;
        while (count--)
        {
            __asm__ __volatile__("nop");
        }
#endif

        // Read RSSI directly (no cache) using our override
        rssi_values[i] = getRSSI();

        // Next frequency
        current_freq += step_mhz;
    }

    // Return to standby mode
    standby();

    return RADIOLIB_ERR_NONE;
}
