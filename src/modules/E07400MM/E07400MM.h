/**
 * @file E07400MM.h
 * @brief E07-400MM (CC1101 clone) driver for RadioLib
 *
 * This class extends RadioLib's CC1101 class to handle quirks specific to
 * the Chinese E07-400MM clone module used in the Vandal project.
 *
 * @author 0x42en (0x42en@gmail.com)
 * @date 2025-01-08
 *
 * CRITICAL E07-400MM QUIRKS HANDLED BY THIS CLASS:
 *
 * 1. STATUS REGISTERS (0x30-0x3D) REQUIRE BURST MODE
 *    - Must use SPIreadRegisterBurst(reg | 0x40, ...) not SPIreadRegister()
 *    - Affects: RSSI, LQI, and other status registers
 *
 * 2. FIFO REGISTER (0x3F) REQUIRES BURST MODE
 *    - RadioLib's getPacketLength() uses SPIreadRegister() which fails
 *    - Must read FIFO directly with BURST mode
 *    - First byte in variable length mode = packet length
 *
 * 3. RSSI CACHING ISSUE
 *    - RadioLib getRSSI() caches values
 *    - Scanning requires direct register read for real-time RSSI
 *
 * 4. TIMING REQUIREMENTS
 *    - AGC settling time: 2-3ms per frequency point
 *    - FreeRTOS tick is 10ms - must use esp_rom_delay_us() for <10ms delays
 *
 * METHODS OVERRIDDEN:
 * - getRSSI()         : Direct register read with BURST mode (no cache)
 * - getPacketLength() : Direct FIFO access with BURST mode
 * - readData()        : Complete FIFO read with BURST mode (data + RSSI + LQI)
 *
 * METHODS SHADOWED (not virtual in CC1101, can't use 'override'):
 * - packetMode()      : Atomic PKTCTRL0 write (not split like RadioLib)
 * - setOOK()          : Force MDMCFG2 sync detection after modulation change
 *
 * NEW METHODS:
 * - scanRSSI()        : Optimized spectrum scanning with microsecond precision
 */

#ifndef E07_400MM_H
#define E07_400MM_H

#include "../CC1101/CC1101.h"
#include "../../TypeDef.h"

#if defined(ESP_PLATFORM)
#include "esp_rom_sys.h"
#elif defined(RADIOLIB_BUILD_ARDUINO)
#include <Arduino.h>
#endif

/**
 * @brief E07-400MM class extending RadioLib CC1101
 *
 * This class provides compatibility with the E07-400MM CC1101 clone by
 * overriding methods that require BURST mode for register access.
 */
class E07_400MM : public CC1101
{
public:
    /**
     * @brief Constructor
     * @param module Pointer to Module instance (SPI wrapper)
     */
    E07_400MM(Module *module);

    /**
     * @brief Get RSSI of last received packet or current RSSI level
     *
     * Overrides RadioLib's getRSSI() to use direct register access with BURST mode.
     * This is critical for scanning as RadioLib caches RSSI values.
     *
     * @return RSSI in dBm
     */
    float getRSSI() override;

    /**
     * @brief Get packet length from FIFO
     *
     * Overrides RadioLib's getPacketLength() to use direct FIFO access with BURST mode.
     * RadioLib's implementation uses SPIreadRegister() which doesn't work with FIFO.
     *
     * @param update Update packet length from FIFO (default: true)
     * @return Packet length in bytes
     */
    size_t getPacketLength(bool update = true) override;

    /**
     * @brief Read received packet data
     *
     * Overrides RadioLib's readData() to use BURST mode for all FIFO operations.
     * Reads packet data, RSSI, and LQI in correct sequence.
     *
     * @param data Buffer to store received data
     * @param len Maximum length to read
     * @return Status code (RADIOLIB_ERR_NONE on success)
     */
    int16_t readData(uint8_t *data, size_t len) override;

    /**
     * @brief Perform optimized RSSI spectrum scan
     *
     * New method specific to E07-400MM for high-performance spectrum scanning.
     * Uses direct register access with microsecond-precision delays.
     *
     * @param rssi_values Array to store RSSI values (must be pre-allocated)
     * @param num_points Number of frequency points to scan
     * @param center_freq Center frequency in MHz
     * @param step_khz Frequency step in kHz
     * @param dwell_time_us Dwell time per frequency in microseconds (default: 3000µs = 3ms)
     * @return Status code (RADIOLIB_ERR_NONE on success)
     */
    int16_t scanRSSI(float *rssi_values, size_t num_points,
                     float center_freq, float step_khz,
                     uint16_t dwell_time_us = 3000);

    /**
     * @brief Configure packet mode with atomic PKTCTRL0 write
     *
     * Overrides RadioLib's packetMode() which writes PKTCTRL0 in 2 separate SPI calls.
     * E07-400MM clones require atomic register writes.
     *
     * NOTE: Not marked 'override' because CC1101::packetMode() is not virtual.
     * This method SHADOWS (not overrides) the parent method.
     *
     * @return Status code (RADIOLIB_ERR_NONE on success)
     */
    int16_t packetMode();

    /**
     * @brief Set OOK/ASK modulation and restore sync word detection
     *
     * Overrides RadioLib's setOOK() to force MDMCFG2 sync mode restoration.
     * This fixes the scanner→sniffer bug where sync detection gets disabled.
     *
     * NOTE: Not marked 'override' because CC1101::setOOK() is not virtual.
     * This method SHADOWS (not overrides) the parent method.
     *
     * @param enableOOK Enable OOK (true) or disable (false)
     * @return Status code (RADIOLIB_ERR_NONE on success)
     */
    int16_t setOOK(bool enableOOK);

private:
    /**
     * @brief Convert raw RSSI register value to dBm
     *
     * Uses CC1101 formula: RSSI_dBm = (RSSI_dec/2) - 74 (if RSSI_dec < 128)
     *                                 or ((RSSI_dec-256)/2) - 74 (if RSSI_dec >= 128)
     *
     * @param rssi_raw Raw RSSI register value (0-255)
     * @return RSSI in dBm
     */
    float convertRSSI(uint8_t rssi_raw);

    /**
     * @brief Restore sync word detection in MDMCFG2
     *
     * After scanner mode (which disables sync), or after setOOK()/setFSK()
     * (which don't touch MDMCFG2[2:0]), we must restore sync word detection.
     *
     * Sets MDMCFG2[2:0] = 0b010 (16/16 sync bits detected, recommended mode)
     *
     * @return Status code (RADIOLIB_ERR_NONE on success)
     */
    int16_t restoreSyncMode();
};

#endif // E07_400MM_H
