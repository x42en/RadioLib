#include "E07400MM.h"
#if !RADIOLIB_EXCLUDE_CC1101

E07_400MM::E07_400MM(Module *module) : CC1101(module)
{
    // Constructor - just call parent
}

uint8_t E07_400MM::getRSSIRaw()
{
    uint8_t rssi_raw;
    // E07-400MM clones require BURST mode for reading status registers
    SPIreadRegisterBurst(RADIOLIB_CC1101_REG_RSSI | RADIOLIB_CC1101_CMD_ACCESS_STATUS_REG, 1, &rssi_raw);

    return rssi_raw;
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

    // Sanity check (packet_length is uint8_t, so it can never exceed
    // RADIOLIB_CC1101_MAX_PACKET_LENGTH == 255; only zero is invalid here)
    if (packet_length == 0)
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
    // Get packet length (E07 reads the length byte via BURST)
    size_t length = getPacketLength();

    // Clamp to the caller's buffer when smaller than the received packet
    if ((len != 0) && (len < length))
    {
        length = len;
    }

    // Discard the address byte when address filtering is enabled
    uint8_t filter = SPIgetRegValue(RADIOLIB_CC1101_REG_PKTCTRL1, 1, 0);
    if (filter != RADIOLIB_CC1101_ADR_CHK_NONE)
    {
        uint8_t address;
        SPIreadRegisterBurst(RADIOLIB_CC1101_REG_FIFO, 1, &address);
    }

    // Read packet payload (BURST is mandatory on E07 clones)
    SPIreadRegisterBurst(RADIOLIB_CC1101_REG_FIFO, length, data);

    // Read appended status bytes only when enabled (mirrors CC1101::readData)
    bool isAppendStatus = SPIgetRegValue(RADIOLIB_CC1101_REG_PKTCTRL1, 2, 2) == RADIOLIB_CC1101_APPEND_STATUS_ON;

    int16_t state = RADIOLIB_ERR_NONE;
    if (isAppendStatus)
    {
        // RSSI byte (BURST read for the E07 quirk)
        uint8_t rssi_raw;
        SPIreadRegisterBurst(RADIOLIB_CC1101_REG_FIFO, 1, &rssi_raw);
        this->rawRSSI = rssi_raw;

        // LQI + CRC byte (bit 7 = CRC OK)
        uint8_t lqi_crc;
        SPIreadRegisterBurst(RADIOLIB_CC1101_REG_FIFO, 1, &lqi_crc);
        this->rawLQI = lqi_crc & 0x7F;

        if (this->crcOn && (lqi_crc & RADIOLIB_CC1101_CRC_OK) == RADIOLIB_CC1101_CRC_ERROR)
        {
            this->packetLengthQueried = false;
            state = RADIOLIB_ERR_CRC_MISMATCH;
        }
    }

    // Clear cached length so getPacketLength re-reads on the next packet
    this->packetLengthQueried = false;

    // Flush + standby according to RXOFF_MODE (classic CC1101 behavior);
    // the application is responsible for calling startReceive() again
    if (SPIgetRegValue(RADIOLIB_CC1101_REG_MCSM1, 3, 2) == RADIOLIB_CC1101_RXOFF_IDLE)
    {
        (void)finishReceive();
    }

    return state;
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

    // E07-400MM clone needs time to process packet config changes.
    // Without this delay, FIFO operations may fail or produce garbage.
    this->getMod()->hal->delayMicroseconds(5000); // 5ms

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

#endif
