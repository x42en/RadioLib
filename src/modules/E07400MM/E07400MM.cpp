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
    uint8_t rssi_raw;
    // CRITICAL FIX: E07-400MM clones require BURST mode for reading status registers
    SPIreadRegisterBurst(RADIOLIB_CC1101_REG_RSSI | RADIOLIB_CC1101_CMD_ACCESS_STATUS_REG, 1, &rssi_raw);

    return convertRSSI(rssi_raw);
}

size_t E07_400MM::getPacketLength(bool update)
{
    // Return cached value if update not requested
    if (!update && this->packetLengthQueried)
    {
        return this->packetLength;
    }

    uint8_t packet_length;
    // Read first byte from FIFO (packet length in variable length mode)
    SPIreadRegisterBurst(RADIOLIB_CC1101_REG_FIFO, 1, &packet_length);

    // Sanity check
    if (packet_length == 0 || packet_length > RADIOLIB_CC1101_MAX_PACKET_LENGTH)
    {
        SPIsendCommand(RADIOLIB_CC1101_CMD_FLUSH_RX);
        startReceive();
        return 0;
    }

    // Cache the result
    this->packetLength = packet_length;
    this->packetLengthQueried = true;

    return packet_length;
}

int16_t E07_400MM::readData(uint8_t *data, size_t len)
{
    // Reset query flag
    this->packetLengthQueried = false;

    // Read packet length from FIFO
    uint8_t packet_length = getPacketLength();

    // // Sanity check
    // if (packet_length == 0 || packet_length > RADIOLIB_CC1101_MAX_PACKET_LENGTH)
    // {
    //     SPIsendCommand(RADIOLIB_CC1101_CMD_FLUSH_RX);
    //     startReceive();
    //     return RADIOLIB_ERR_INVALID_PAYLOAD;
    // }

    // Limit read length to buffer size
    size_t read_length = (packet_length < len) ? packet_length : len;

    // Read packet data from FIFO (SPIreadRegisterBurst adds BURST bit automatically)
    SPIreadRegisterBurst(RADIOLIB_CC1101_REG_FIFO, read_length, data);

    // Read RSSI and LQI status bytes (2 bytes appended by CC1101)
    uint8_t rssi_raw, lqi_crc;
    SPIreadRegisterBurst(RADIOLIB_CC1101_REG_FIFO, 1, &rssi_raw);
    SPIreadRegisterBurst(RADIOLIB_CC1101_REG_FIFO, 1, &lqi_crc);

    // Store for later retrieval
    this->rawRSSI = rssi_raw;
    this->rawLQI = lqi_crc & 0x7F;

    // Check CRC (bit 7 of LQI byte)
    bool crc_ok = (lqi_crc & 0x80) != 0;

    if (!crc_ok && this->crcOn)
    {
        startReceive();
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

        // CRITICAL: Enter RX mode AFTER frequency change!
        // setFrequency() puts CC1101 in IDLE, we must re-enter RX for AGC to work
        SPIsendCommand(RADIOLIB_CC1101_CMD_RX);

// Wait for AGC settling time
// CRITICAL: Must use esp_rom_delay_us() because FreeRTOS tick is 10ms
// CC1101 AGC settling time is typically 2-3ms according to datasheet
#if defined(ARDUINO)
        delayMicroseconds(dwell_time_us);
#elif defined(ESP_PLATFORM)
        esp_rom_delay_us(dwell_time_us);
#else
        // Fallback: standard delay (not accurate, but prevents build errors)
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

int16_t E07_400MM::packetMode()
{
    // CRITICAL FIX: RadioLib's CC1101::packetMode() writes PKTCTRL0 in 2 separate SPI calls:
    // SPIsetRegValue(PKTCTRL0, ..., 6, 4);  // Bits 6-4
    // SPIsetRegValue(PKTCTRL0, ..., 2, 0);  // Bits 2-0 (SEPARATE!)
    //
    // E07-400MM clones don't handle this correctly. We need atomic writes.

    int16_t state = RADIOLIB_ERR_NONE;

    // PKTCTRL1: PQT=0, CRC autoflush OFF, append status ON, no address check
    // Bits 7-5: 000 (PQT threshold)
    // Bit 3: 0 (CRC autoflush disabled)
    // Bit 2: 1 (append status bytes - CRITICAL for RSSI/LQI)
    // Bits 1-0: 00 (no address check)
    uint8_t pktctrl1 = 0x0C; // 0b00001100
    state = SPIsetRegValue(RADIOLIB_CC1101_REG_PKTCTRL1, pktctrl1);
    RADIOLIB_ASSERT(state);

    // Read what RadioLib configured for PKTCTRL0 (via begin() or setOOK/setFSK)
    uint8_t pktctrl0 = SPIgetRegValue(RADIOLIB_CC1101_REG_PKTCTRL0);

    // Force variable length mode (bits 1-0 = 01) while preserving other bits
    // Bits 7-6: preserve (data whitening)
    // Bits 5-4: preserve (packet format/CRC)
    // Bits 3-2: preserve (modulation format)
    // Bits 1-0: 01 (variable length packets)
    pktctrl0 = (pktctrl0 & 0xFC) | 0x01;

    // Write PKTCTRL0 atomically (single SPI transaction)
    state = SPIsetRegValue(RADIOLIB_CC1101_REG_PKTCTRL0, pktctrl0);
    RADIOLIB_ASSERT(state);

// E07-400MM clone needs time to process packet config changes
// Without this delay, FIFO operations may fail or produce garbage
#if defined(RADIOLIB_BUILD_ARDUINO)
    delay(5);
#elif defined(ESP_PLATFORM)
    // vTaskDelay requires FreeRTOS headers - use ROM delay instead
    esp_rom_delay_us(5000); // 5ms
#else
    // Fallback: busy wait
    for (volatile uint32_t i = 0; i < 1000000; i++)
        ;
#endif

    return state;
}

int16_t E07_400MM::restoreSyncMode()
{
    // Read current MDMCFG2 value
    uint8_t mdmcfg2 = SPIgetRegValue(RADIOLIB_CC1101_REG_MDMCFG2);

    // Force sync word detection: 16/16 sync bits detected (recommended mode)
    // Bits 7: preserve (Manchester encoding)
    // Bits 6-4: preserve (modulation format - set by setOOK/setFSK)
    // Bits 3: preserve (FEC)
    // Bits 2-0: 010 = 16/16 sync word bits detected
    mdmcfg2 = (mdmcfg2 & 0xF8) | 0x02;

    return SPIsetRegValue(RADIOLIB_CC1101_REG_MDMCFG2, mdmcfg2);
}

int16_t E07_400MM::setOOK(bool enableOOK)
{
    // Call parent class implementation (configures MDMCFG2 bits 6-4, FREND0, PA_TABLE)
    int16_t state = CC1101::setOOK(enableOOK);
    RADIOLIB_ASSERT(state);

    // CRITICAL FIX: RadioLib's setOOK() doesn't touch MDMCFG2[2:0] (sync mode)
    // If scanner ran before (which sets MDMCFG2[2:0] = 000), sync detection stays OFF
    // We must restore it here
    state = restoreSyncMode();
    RADIOLIB_ASSERT(state);

    return state;
}
