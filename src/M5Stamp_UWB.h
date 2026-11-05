/**
 * @file M5Stamp_UWB.h
 * @brief Arduino wrapper for the M5Stack Stamp UWB QM33120W module.
 *
 * This public header exposes only Arduino-facing types and APIs. The imported
 * Qorvo dwt driver is kept internal under src/qm33120w_sdk/.
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include "M5Stamp_UWB_Types.h"

/**
 * @brief Arduino interface for probing, initializing, and transmitting with Stamp UWB.
 */
class M5Stamp_UWB {
public:
    M5Stamp_UWB();
    ~M5Stamp_UWB();
    M5Stamp_UWB(const M5Stamp_UWB&)            = delete;
    M5Stamp_UWB& operator=(const M5Stamp_UWB&) = delete;

    /**
     * @brief Initialize GPIO/SPI, probe the UWB chip, and apply the PHY configuration.
     */
    bool begin(const M5Stamp_UWBConfig& config = M5Stamp_UWBConfig(),
               const M5Stamp_UWBPHYConfig& phy = M5Stamp_UWBPHYConfig());

    /**
     * @brief Stop UWB activity and release the active driver instance.
     */
    void end();

    /**
     * @brief Reconfigure the UWB PHY after begin().
     */
    bool init(const M5Stamp_UWBPHYConfig& phy = M5Stamp_UWBPHYConfig());

    /**
     * @brief Toggle the hardware RESET pin if one is configured.
     */
    void hardReset(uint32_t reset_low_ms = 5, uint32_t startup_ms = 100);

    /**
     * @brief Return the probed Device ID. Returns 0 if the device is not connected.
     */
    uint32_t deviceId() const;

    /**
     * @brief Read DEV_ID directly through SPI without requiring a successful probe.
     */
    uint32_t readRawDeviceId();

    /**
     * @brief Return a short chip name based on the probed Device ID.
     */
    const char* chipName() const;

    /**
     * @brief Send a short-address IEEE 802.15.4 frame with a user payload.
     */
    M5Stamp_UWBTxResult sendFrame(const uint8_t* payload, size_t length,
                                  const M5Stamp_UWBFrameConfig& frame = M5Stamp_UWBFrameConfig(),
                                  uint32_t timeoutMs                  = 100);

    /**
     * @brief Convenience overload for sending a C string payload.
     */
    M5Stamp_UWBTxResult sendFrame(const char* payload, const M5Stamp_UWBFrameConfig& frame = M5Stamp_UWBFrameConfig(),
                                  uint32_t timeoutMs = 100);

    /**
     * @brief Wait for one short-address IEEE 802.15.4 frame and copy its payload.
     */
    M5Stamp_UWBRxResult receiveFrame(uint8_t* payload, size_t payloadSize, uint32_t timeoutMs = 100);

    /**
     * @brief Run one SS-TWR initiator exchange and return the measured range.
     */
    M5Stamp_UWBRangeResult requestRange(const M5Stamp_UWBRangeConfig& range = M5Stamp_UWBRangeConfig());

    /**
     * @brief Wait for one SS-TWR poll and send a timestamped response.
     */
    M5Stamp_UWBResponderResult respondRange(const M5Stamp_UWBRangeConfig& range = M5Stamp_UWBRangeConfig());

    /**
     * @brief Run one DS-TWR initiator exchange and return the measured range.
     */
    M5Stamp_UWBDSRangeResult requestDSRange(const M5Stamp_UWBDSRangeConfig& range = M5Stamp_UWBDSRangeConfig());

    /**
     * @brief Wait for one DS-TWR poll, complete the final exchange, and return the measured range.
     */
    M5Stamp_UWBDSResponderResult respondDSRange(const M5Stamp_UWBDSRangeConfig& range = M5Stamp_UWBDSRangeConfig());

    bool isConnected() const;
    bool isInitialized() const;

    /**
     * @brief Return true when DW_IRQ is attached and its interrupt mask is armed.
     */
    bool isIrqActive() const;

    /** @brief Backwards-compatible alias for isIrqActive(). */
    bool isInterruptDriven() const;
    M5Stamp_UWBError lastError() const;
    const char* lastErrorName() const;
    const M5Stamp_UWBConfig& config() const;

private:
    struct Impl;

    bool probe();
    void setSpiRate(uint32_t hz);
    void wakeupDeviceWithIoImpl();
    bool validCs() const;
    void setError(M5Stamp_UWBError error);

    static int32_t readFromSpi(uint16_t header_length, uint8_t* header_buffer, uint16_t read_length,
                               uint8_t* read_buffer);
    static int32_t writeToSpi(uint16_t header_length, const uint8_t* header_buffer, uint16_t body_length,
                              const uint8_t* body_buffer);
    static int32_t writeToSpiWithCrc(uint16_t header_length, const uint8_t* header_buffer, uint16_t body_length,
                                     const uint8_t* body_buffer, uint8_t crc8);
    static void setSlowRate();
    static void setFastRate();
    static void wakeupDeviceWithIo();

    int32_t readFromSpiImpl(uint16_t header_length, uint8_t* header_buffer, uint16_t read_length, uint8_t* read_buffer);
    int32_t writeToSpiImpl(uint16_t header_length, const uint8_t* header_buffer, uint16_t body_length,
                           const uint8_t* body_buffer, const uint8_t* crc8);

    Impl* _impl;
    static M5Stamp_UWB* _active;
};
