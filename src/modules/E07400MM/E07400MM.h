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
 *    - Sub-millisecond delays are issued through the RadioLib HAL
 *      (hal->delayMicroseconds); the active HAL is responsible for
 *      providing busy-wait accuracy below the platform scheduler tick
 *
 * METHODS OVERRIDDEN:
 * - getRSSIRaw()      : Live RSSI register read with BURST mode (used by
 *                       CC1101::getRSSILive() and CC1101::scanRSSI())
 * - getPacketLength() : Direct FIFO access with BURST mode
 * - readData()        : Complete FIFO read with BURST mode (data + RSSI + LQI)
 *
 * METHODS SHADOWED (not virtual in CC1101, can't use 'override'):
 * - packetMode()      : Atomic PKTCTRL0 write (not split like RadioLib)
 * - setOOK()          : Force MDMCFG2 sync detection after modulation change
 *
 * The generic spectrum sweep (scanRSSI) and the live RSSI accessor
 * (getRSSILive) now live in the CC1101 base class; this clone only needs to
 * override the low-level getRSSIRaw() read to satisfy the BURST quirk.
 */

#if !defined(_RADIOLIB_E07400MM_H) && !RADIOLIB_EXCLUDE_CC1101
#define _RADIOLIB_E07400MM_H

#include "../../TypeDef.h"
#include "../../Module.h"
#include "../CC1101/CC1101.h"

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

protected:
    /**
     * @brief Read the raw RSSI status register using BURST mode
     *
     * Overrides CC1101::getRSSIRaw() to satisfy the E07-400MM quirk that
     * status registers (0x30-0x3D) can only be read in BURST mode. Used by
     * the inherited getRSSILive() and scanRSSI() implementations.
     *
     * @return Raw RSSI register value (0-255)
     */
    uint8_t getRSSIRaw() override;

private:
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

#endif
