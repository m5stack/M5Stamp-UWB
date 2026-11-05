/**
 * @file M5Stamp_UWB_Types.h
 * @brief Public Arduino-facing types for the M5Stamp_UWB library.
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <Arduino.h>
#include <SPI.h>

#ifndef M5STAMP_UWB_PIN_UNUSED
#define M5STAMP_UWB_PIN_UNUSED (-1)
#endif

#define M5STAMP_UWB_OK 0

static constexpr uint32_t M5STAMP_UWB_QM33120_DEVICE_ID = 0xDECA0314;

/**
 * @brief Board-level pin and SPI configuration for a Stamp UWB connection.
 *
 * All pin numbers are Arduino GPIO numbers. Defaults match the Stamp C5 host wiring.
 * Set optional pins to M5STAMP_UWB_PIN_UNUSED if the carrier board does not connect them.
 */
struct M5Stamp_UWBConfig {
    SPIClass* spi                 = &SPI;      //!< SPI bus used by the UWB chip.
    int8_t pin_cs                 = 11;        //!< SPI chip-select pin.
    int8_t pin_rst                = 25;        //!< Optional hardware reset pin.
    int8_t pin_irq                = 0;         //!< Optional interrupt input pin.
    int8_t pin_wakeup             = 24;        //!< Optional WAKEUP pin.
    int8_t pin_gp7                = 23;        //!< Optional DW_GP7 input pin.
    int8_t pin_sck                = 12;        //!< Optional custom SPI SCK pin.
    int8_t pin_miso               = 26;        //!< Optional custom SPI MISO pin.
    int8_t pin_mosi               = 27;        //!< Optional custom SPI MOSI pin.
    uint32_t spi_slow_hz          = 2000000;   //!< Initial SPI speed used before driver initialization.
    uint32_t spi_fast_hz          = 16000000;  //!< SPI speed used after the driver switches to fast mode.
    uint8_t probe_retry_count     = 5;         //!< Number of Device ID/probe retries during begin().
    uint16_t probe_retry_delay_ms = 20;        //!< Delay between Device ID/probe retries.
    bool begin_spi                = true;      //!< Call SPI.begin() inside begin().
    bool hard_reset_on_begin      = true;      //!< Toggle RESET before probing the UWB device.
};

/**
 * @brief High-level errors returned by the Arduino wrapper.
 */
enum class M5Stamp_UWBError : int8_t {
    Ok = 0,
    InvalidConfig,
    SpiNotReady,
    ProbeFailed,
    DeviceIdMismatch,
    InitFailed,
    ConfigFailed,
    TxDataFailed,
    TxStartFailed,
    TxTimeout,
    RxStartFailed,
    RxTimeout,
    RxError,
    RxBufferTooSmall,
    FrameParseFailed,
    RangeTimestampInvalid,
    RangeFrameMismatch,
    InvalidArgument,
    Busy,
};

enum class M5Stamp_UWBDataRate : uint8_t {
    Rate850K,
    Rate6M8,
};

enum class M5Stamp_UWBChannel : uint8_t {
    Channel5 = 5,  //!< Not supported: rejected by begin()/init() for Japan.
    Channel9 = 9,  //!< The only supported profile.
};

enum class M5Stamp_UWBPreambleLength : uint16_t {
    Len64   = 64,
    Len128  = 128,
    Len256  = 256,
    Len512  = 512,
    Len1024 = 1024,
};

enum class M5Stamp_UWBPacSize : uint8_t {
    Pac4,
    Pac8,
    Pac16,
    Pac32,
};

enum class M5Stamp_UWBSfdType : uint8_t {
    IEEE4A,
    DW8,
    DW16,
    IEEE4Z,
};

enum class M5Stamp_UWBStsMode : uint8_t {
    Off,
    Mode1,
    Mode2,
    Sdc,
};

enum class M5Stamp_UWBPdoaMode : uint8_t {
    Off,
    Mode1,
    Mode3,
};

/**
 * @brief UWB PHY configuration used by M5Stamp_UWB::init().
 *
 * If only channel is changed, init() applies the built-in recommended profile
 * for that channel. If any other field is changed, the whole user-provided
 * configuration is used.
 */
struct M5Stamp_UWBPHYConfig {
    M5Stamp_UWBChannel channel               = M5Stamp_UWBChannel::Channel9;
    M5Stamp_UWBPreambleLength preambleLength = M5Stamp_UWBPreambleLength::Len128;
    M5Stamp_UWBPacSize pacSize               = M5Stamp_UWBPacSize::Pac8;
    uint8_t txPreambleCode                   = 9;
    uint8_t rxPreambleCode                   = 9;
    M5Stamp_UWBSfdType sfdType               = M5Stamp_UWBSfdType::DW8;
    M5Stamp_UWBDataRate dataRate             = M5Stamp_UWBDataRate::Rate6M8;
    M5Stamp_UWBStsMode stsMode               = M5Stamp_UWBStsMode::Off;
    uint16_t stsLength                       = 64;
    M5Stamp_UWBPdoaMode pdoaMode             = M5Stamp_UWBPdoaMode::Off;
    uint16_t sfdTimeout                      = 129;
    uint8_t phrMode                          = 0;
    uint8_t phrRate                          = 0;
    uint8_t pgDelay                          = 0x34;
    uint32_t txPower                         = 0xfefefefe;
    uint16_t txAntennaDelay                  = 16385;
    uint16_t rxAntennaDelay                  = 16385;
    bool enableLnaPa                         = true;
};

/**
 * @brief IEEE 802.15.4 short-address frame parameters for TX helper APIs.
 */
struct M5Stamp_UWBFrameConfig {
    uint16_t panId   = 0xDECA;
    uint16_t src     = 0x0001;
    uint16_t dst     = 0xFFFF;
    uint8_t sequence = 0;
    bool useSequence = false;
    bool ranging     = false;
};

/**
 * @brief Result returned by M5Stamp_UWB::sendFrame().
 */
struct M5Stamp_UWBTxResult {
    bool success           = false;
    uint8_t sequence       = 0;
    uint32_t elapsedMs     = 0;
    M5Stamp_UWBError error = M5Stamp_UWBError::Ok;
};

/**
 * @brief Result returned by M5Stamp_UWB::receiveFrame().
 */
struct M5Stamp_UWBRxResult {
    bool success           = false;
    uint8_t sequence       = 0;
    uint16_t panId         = 0;
    uint16_t src           = 0;
    uint16_t dst           = 0;
    size_t payloadLength   = 0;
    uint16_t frameLength   = 0;
    uint32_t elapsedMs     = 0;
    bool ranging           = false;
    M5Stamp_UWBError error = M5Stamp_UWBError::Ok;
};

/**
 * @brief Single-sided two-way ranging timing and address configuration.
 */
struct M5Stamp_UWBRangeConfig {
    uint16_t panId                     = 0xDECA;
    uint16_t initiatorAddress          = 0x0001;
    uint16_t responderAddress          = 0x0002;
    uint32_t responseRxAfterTxDelayUus = 500;
    uint32_t responseTxDelayUus        = 3000;
    uint32_t rxTimeoutUus              = 4500;
    uint32_t hostTimeoutMs             = 100;
};

/**
 * @brief Double-sided two-way ranging timing and address configuration.
 */
struct M5Stamp_UWBDSRangeConfig {
    uint16_t panId                          = 0xDECA;
    uint16_t initiatorAddress               = 0x0001;
    uint16_t responderAddress               = 0x0002;
    uint32_t responseRxAfterTxDelayUus      = 1500;
    uint32_t responseTxDelayUus             = 3000;
    uint32_t finalTxDelayUus                = 1800;
    uint32_t finalRxAfterResponseTxDelayUus = 500;
    uint32_t resultRxAfterFinalTxDelayUus   = 500;
    uint32_t rxTimeoutUus                   = 3000;
    uint32_t hostTimeoutMs                  = 100;
    uint8_t resultRepeatCount               = 3;
    uint32_t resultRepeatGapMs              = 3;
};

/**
 * @brief Result returned by M5Stamp_UWB::requestRange().
 */
struct M5Stamp_UWBRangeResult {
    bool success           = false;
    uint8_t sequence       = 0;
    int32_t distanceMm     = 0;
    float distanceM        = 0.0f;
    uint32_t elapsedMs     = 0;
    M5Stamp_UWBError error = M5Stamp_UWBError::Ok;
};

/**
 * @brief Result returned by M5Stamp_UWB::requestDSRange().
 */
struct M5Stamp_UWBDSRangeResult {
    bool success           = false;
    uint8_t sequence       = 0;
    int32_t distanceMm     = 0;
    float distanceM        = 0.0f;
    uint32_t elapsedMs     = 0;
    M5Stamp_UWBError error = M5Stamp_UWBError::Ok;
};

/**
 * @brief Result returned by M5Stamp_UWB::respondRange().
 */
struct M5Stamp_UWBResponderResult {
    bool success           = false;
    uint8_t sequence       = 0;
    uint16_t requester     = 0;
    uint32_t elapsedMs     = 0;
    M5Stamp_UWBError error = M5Stamp_UWBError::Ok;
};

/**
 * @brief Result returned by M5Stamp_UWB::respondDSRange().
 */
struct M5Stamp_UWBDSResponderResult {
    bool success           = false;
    uint8_t sequence       = 0;
    uint16_t requester     = 0;
    int32_t distanceMm     = 0;
    float distanceM        = 0.0f;
    uint32_t elapsedMs     = 0;
    M5Stamp_UWBError error = M5Stamp_UWBError::Ok;
};
