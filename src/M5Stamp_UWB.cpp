/**
 * @file M5Stamp_UWB.cpp
 * @brief Arduino port layer and high-level wrapper for the Qorvo dwt driver.
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "M5Stamp_UWB.h"

#include <cstring>

#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

extern "C" {
#include "qm33120w_sdk/deca_device_api.h"
#include "qm33120w_sdk/deca_interface.h"
extern const struct dwt_driver_s dw3720_driver;

static portMUX_TYPE g_deca_mutex = portMUX_INITIALIZER_UNLOCKED;

/* Platform delay hooks required by the imported Qorvo dwt driver. */
void deca_sleep(unsigned int time_ms)
{
    delay(time_ms);
}

void deca_usleep(unsigned long time_us)
{
    delayMicroseconds(time_us);
}

/* Protect SDK read-modify-write sections without globally disabling all CPU interrupts. */
decaIrqStatus_t decamutexon(void)
{
    portENTER_CRITICAL(&g_deca_mutex);
    return 0;
}

void decamutexoff(decaIrqStatus_t)
{
    portEXIT_CRITICAL(&g_deca_mutex);
}
}

struct M5Stamp_UWB::Impl {
    M5Stamp_UWBConfig config;
    SPISettings spi_settings    = SPISettings(2000000, MSBFIRST, SPI_MODE0);
    dwt_spi_s dwt_spi           = {M5Stamp_UWB::readFromSpi, M5Stamp_UWB::writeToSpi, M5Stamp_UWB::writeToSpiWithCrc,
                                   M5Stamp_UWB::setSlowRate, M5Stamp_UWB::setFastRate};
    bool connected              = false;
    bool initialized            = false;
    uint32_t device_id          = 0;
    uint8_t tx_sequence         = 0;
    uint16_t tx_antenna_delay   = 16385;
    M5Stamp_UWBError last_error = M5Stamp_UWBError::Ok;
};

M5Stamp_UWB* M5Stamp_UWB::_active = nullptr;

/* --------------------------------------------------------------------------
 * DW_IRQ driven wait.
 *
 * DW_IRQ is a level-sensitive, active-high line: it stays high while any
 * enabled SYS_STATUS bit is pending. The ISR only wakes the task; every SPI
 * access stays in task context. The wait itself blocks until the line rises or
 * the caller's timeout expires, instead of polling once per millisecond.
 *
 * The semaphore is drained at the start of every public operation, and a
 * pending bit outside the current wait mask is cleared so it cannot hold the
 * line high and swallow the edge for the event we are actually waiting for.
 * -------------------------------------------------------------------------- */
static SemaphoreHandle_t s_irq_sem = nullptr;
static bool s_irq_attached         = false;
static bool s_irq_enabled          = false;
static int8_t s_irq_pin            = M5STAMP_UWB_PIN_UNUSED;

static constexpr uint32_t kIrqRxMask =
    static_cast<uint32_t>(DWT_INT_RXFCG_BIT_MASK) | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR;
static constexpr uint32_t kIrqMask = kIrqRxMask | static_cast<uint32_t>(DWT_INT_TXFRS_BIT_MASK);

static void IRAM_ATTR uwbIrqIsr()
{
    if (s_irq_sem == nullptr) {
        return;
    }
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_irq_sem, &higher_priority_task_woken);
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void irqDrain()
{
    if (s_irq_sem == nullptr) {
        return;
    }
    while (xSemaphoreTake(s_irq_sem, 0) == pdTRUE) {
    }
}

static void irqAttach(int8_t pin)
{
    if (s_irq_attached || (pin == M5STAMP_UWB_PIN_UNUSED)) {
        return;
    }
    const int irq = digitalPinToInterrupt(pin);
    if (irq < 0) {
        return;
    }
    if (s_irq_sem == nullptr) {
        s_irq_sem = xSemaphoreCreateBinary();
        if (s_irq_sem == nullptr) {
            return;
        }
    }
    pinMode(pin, INPUT);
    attachInterrupt(irq, uwbIrqIsr, RISING);
    s_irq_pin      = pin;
    s_irq_attached = true;
}

static void irqDetach()
{
    if (!s_irq_attached) {
        return;
    }
    const int irq = digitalPinToInterrupt(s_irq_pin);
    if (irq >= 0) {
        detachInterrupt(irq);
    }
    s_irq_attached = false;
    s_irq_enabled  = false;
    s_irq_pin      = M5STAMP_UWB_PIN_UNUSED;
    irqDrain();
}

static void irqEnable()
{
    if (!s_irq_attached) {
        s_irq_enabled = false;
        return;
    }
    /* dwt_configure() clears SYS_ENABLE_LO, so this must follow every configure. */
    dwt_setinterrupt(kIrqMask, 0, DWT_ENABLE_INT_ONLY);
    dwt_writesysstatuslo(kIrqMask);
    irqDrain();
    s_irq_enabled = true;
}

static void irqDisable()
{
    if (!s_irq_enabled) {
        return;
    }
    dwt_setinterrupt(0, 0, DWT_ENABLE_INT_ONLY);
    s_irq_enabled = false;
}

/* Block until DW_IRQ or the remaining timeout. `status` is the value the caller
 * just read; `mask` is the event it is waiting for. */
static void irqWait(uint32_t status, uint32_t mask, uint32_t start_ms, uint32_t timeout_ms)
{
    if (!s_irq_attached || !s_irq_enabled) {
        delay(1);
        return;
    }

    const uint32_t elapsed = millis() - start_ms;
    if (elapsed >= timeout_ms) {
        return;
    }

    const uint32_t stale = status & kIrqMask & ~mask;
    if (stale != 0) {
        dwt_writesysstatuslo(stale);
        return;
    }

    TickType_t ticks = pdMS_TO_TICKS(timeout_ms - elapsed);
    if (ticks == 0) {
        ticks = 1;
    }
    (void)xSemaphoreTake(s_irq_sem, ticks);
}

static uint16_t pacSymbols(M5Stamp_UWBPacSize pacSize)
{
    switch (pacSize) {
        case M5Stamp_UWBPacSize::Pac4:
            return 4;
        case M5Stamp_UWBPacSize::Pac16:
            return 16;
        case M5Stamp_UWBPacSize::Pac32:
            return 32;
        case M5Stamp_UWBPacSize::Pac8:
        default:
            return 8;
    }
}

static uint16_t sfdSymbols(M5Stamp_UWBSfdType sfdType)
{
    switch (sfdType) {
        case M5Stamp_UWBSfdType::DW16:
            return 16;
        case M5Stamp_UWBSfdType::IEEE4A:
        case M5Stamp_UWBSfdType::IEEE4Z:
        case M5Stamp_UWBSfdType::DW8:
        default:
            return 8;
    }
}

static uint16_t makeSfdTimeout(const M5Stamp_UWBPHYConfig& phy)
{
    if (phy.sfdTimeout != 0) {
        return phy.sfdTimeout;
    }

    const uint16_t preambleSymbols = static_cast<uint16_t>(phy.preambleLength);
    const uint16_t sfdLength       = sfdSymbols(phy.sfdType);
    const uint16_t pacLength       = pacSymbols(phy.pacSize);

    // Qorvo recommended formula: SFD timeout = preamble length + 1 + SFD length - PAC length.
    return static_cast<uint16_t>(preambleSymbols + 1 + sfdLength - pacLength);
}

static dwt_uwb_bit_rate_e toDwtDataRate(M5Stamp_UWBDataRate dataRate)
{
    return dataRate == M5Stamp_UWBDataRate::Rate850K ? DWT_BR_850K : DWT_BR_6M8;
}

static dwt_pac_size_e toDwtPacSize(M5Stamp_UWBPacSize pacSize)
{
    switch (pacSize) {
        case M5Stamp_UWBPacSize::Pac4:
            return DWT_PAC4;
        case M5Stamp_UWBPacSize::Pac16:
            return DWT_PAC16;
        case M5Stamp_UWBPacSize::Pac32:
            return DWT_PAC32;
        case M5Stamp_UWBPacSize::Pac8:
        default:
            return DWT_PAC8;
    }
}

static dwt_sfd_type_e toDwtSfdType(M5Stamp_UWBSfdType sfdType)
{
    switch (sfdType) {
        case M5Stamp_UWBSfdType::IEEE4A:
            return DWT_SFD_IEEE_4A;
        case M5Stamp_UWBSfdType::DW16:
            return DWT_SFD_DW_16;
        case M5Stamp_UWBSfdType::IEEE4Z:
            return DWT_SFD_IEEE_4Z;
        case M5Stamp_UWBSfdType::DW8:
        default:
            return DWT_SFD_DW_8;
    }
}

static dwt_sts_mode_e toDwtStsMode(M5Stamp_UWBStsMode stsMode)
{
    switch (stsMode) {
        case M5Stamp_UWBStsMode::Mode1:
            return DWT_STS_MODE_1;
        case M5Stamp_UWBStsMode::Mode2:
            return DWT_STS_MODE_2;
        case M5Stamp_UWBStsMode::Sdc:
            return DWT_STS_MODE_SDC;
        case M5Stamp_UWBStsMode::Off:
        default:
            return DWT_STS_MODE_OFF;
    }
}

static dwt_sts_lengths_e toDwtStsLength(uint16_t stsLength)
{
    switch (stsLength) {
        case 16:
            return DWT_STS_LEN_16;
        case 32:
            return DWT_STS_LEN_32;
        case 128:
            return DWT_STS_LEN_128;
        case 256:
            return DWT_STS_LEN_256;
        case 512:
            return DWT_STS_LEN_512;
        case 1024:
            return DWT_STS_LEN_1024;
        case 2048:
            return DWT_STS_LEN_2048;
        case 64:
        default:
            return DWT_STS_LEN_64;
    }
}

static dwt_pdoa_mode_e toDwtPdoaMode(M5Stamp_UWBPdoaMode pdoaMode)
{
    switch (pdoaMode) {
        case M5Stamp_UWBPdoaMode::Mode1:
            return DWT_PDOA_M1;
        case M5Stamp_UWBPdoaMode::Mode3:
            return DWT_PDOA_M3;
        case M5Stamp_UWBPdoaMode::Off:
        default:
            return DWT_PDOA_M0;
    }
}

static uint8_t channelNumber(M5Stamp_UWBChannel channel)
{
    return static_cast<uint8_t>(channel);
}

static bool isValidChannel(M5Stamp_UWBChannel channel)
{
    /* CH5 is intentionally unsupported: Japanese radio regulations only allow
     * the CH9 profile for this product. Keep the enum value for source
     * compatibility, but reject it before dwt_configure(). */
    return channel == M5Stamp_UWBChannel::Channel9;
}

static M5Stamp_UWBPHYConfig recommendedPHYProfile(M5Stamp_UWBChannel channel)
{
    (void)channel;
    M5Stamp_UWBPHYConfig phy;
    /* The library is locked to the CH9 profile. */
    phy.channel = M5Stamp_UWBChannel::Channel9;
    return phy;
}

static bool usesDefaultNonChannelFields(const M5Stamp_UWBPHYConfig& phy)
{
    const M5Stamp_UWBPHYConfig defaults;
    return (phy.preambleLength == defaults.preambleLength) && (phy.pacSize == defaults.pacSize) &&
           (phy.txPreambleCode == defaults.txPreambleCode) && (phy.rxPreambleCode == defaults.rxPreambleCode) &&
           (phy.sfdType == defaults.sfdType) && (phy.dataRate == defaults.dataRate) &&
           (phy.stsMode == defaults.stsMode) && (phy.stsLength == defaults.stsLength) &&
           (phy.pdoaMode == defaults.pdoaMode) && (phy.sfdTimeout == defaults.sfdTimeout) &&
           (phy.phrMode == defaults.phrMode) && (phy.phrRate == defaults.phrRate) &&
           (phy.pgDelay == defaults.pgDelay) && (phy.txPower == defaults.txPower) &&
           (phy.txAntennaDelay == defaults.txAntennaDelay) && (phy.rxAntennaDelay == defaults.rxAntennaDelay) &&
           (phy.enableLnaPa == defaults.enableLnaPa);
}

static M5Stamp_UWBPHYConfig resolvePHYConfig(const M5Stamp_UWBPHYConfig& phy)
{
    if (usesDefaultNonChannelFields(phy)) {
        return recommendedPHYProfile(phy.channel);
    }
    return phy;
}

static dwt_config_t toDwtConfig(const M5Stamp_UWBPHYConfig& phy)
{
    dwt_config_t config   = {};
    config.chan           = channelNumber(phy.channel);
    config.txPreambLength = static_cast<uint16_t>(phy.preambleLength);
    config.rxPAC          = toDwtPacSize(phy.pacSize);
    config.txCode         = phy.txPreambleCode;
    config.rxCode         = phy.rxPreambleCode;
    config.sfdType        = toDwtSfdType(phy.sfdType);
    config.dataRate       = toDwtDataRate(phy.dataRate);
    config.phrMode        = static_cast<dwt_phr_mode_e>(phy.phrMode);
    config.phrRate        = static_cast<dwt_phr_rate_e>(phy.phrRate);
    config.sfdTO          = makeSfdTimeout(phy);
    config.stsMode        = toDwtStsMode(phy.stsMode);
    config.stsLength      = toDwtStsLength(phy.stsLength);
    config.pdoaMode       = toDwtPdoaMode(phy.pdoaMode);
    return config;
}

static dwt_txconfig_t toDwtTxConfig(const M5Stamp_UWBPHYConfig& phy)
{
    dwt_txconfig_t config = {};
    config.PGdly          = phy.pgDelay;
    config.power          = phy.txPower;
    config.PGcount        = 0;
    return config;
}

static void set16le(uint8_t* dst, uint16_t value)
{
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

static void set32le(uint8_t* dst, uint32_t value)
{
    dst[0] = static_cast<uint8_t>(value);
    dst[1] = static_cast<uint8_t>(value >> 8);
    dst[2] = static_cast<uint8_t>(value >> 16);
    dst[3] = static_cast<uint8_t>(value >> 24);
}

static uint16_t get16le(const uint8_t* src)
{
    return static_cast<uint16_t>(src[0]) | (static_cast<uint16_t>(src[1]) << 8);
}

static uint32_t get32le(const uint8_t* src)
{
    return static_cast<uint32_t>(src[0]) | (static_cast<uint32_t>(src[1]) << 8) |
           (static_cast<uint32_t>(src[2]) << 16) | (static_cast<uint32_t>(src[3]) << 24);
}

static uint64_t get40le(const uint8_t* src)
{
    uint64_t value = 0;
    for (int i = 4; i >= 0; --i) {
        value = (value << 8) | src[i];
    }
    return value;
}

static uint64_t readTxTimestamp64()
{
    uint8_t ts[5] = {0};
    dwt_readtxtimestamp(ts);
    return get40le(ts);
}

static uint64_t readRxTimestamp64()
{
    uint8_t ts[5] = {0};
    dwt_readrxtimestamp(ts, static_cast<dwt_ip_sts_segment_e>(0));
    return get40le(ts);
}

static constexpr uint16_t SHORT_ADDRESS_HEADER_LEN = 9;

static uint16_t shortAddressFrameLength(size_t payloadLength)
{
    return static_cast<uint16_t>(SHORT_ADDRESS_HEADER_LEN + payloadLength + FCS_LEN);
}

static void buildShortAddressFrame(uint8_t* frame, uint8_t sequence, uint16_t panId, uint16_t src, uint16_t dst,
                                   const uint8_t* payload, size_t payloadLength)
{
    frame[0] = 0x41;
    frame[1] = 0x88;
    frame[2] = sequence;
    set16le(&frame[3], panId);
    set16le(&frame[5], dst);
    set16le(&frame[7], src);
    memcpy(&frame[9], payload, payloadLength);
}

static bool parseShortAddressFrame(const uint8_t* frame, uint16_t frameLen, M5Stamp_UWBRxResult& result)
{
    if ((frame == nullptr) || (frameLen < (SHORT_ADDRESS_HEADER_LEN + FCS_LEN))) {
        return false;
    }
    if ((frame[0] != 0x41) || (frame[1] != 0x88)) {
        return false;
    }

    result.sequence      = frame[2];
    result.panId         = get16le(&frame[3]);
    result.dst           = get16le(&frame[5]);
    result.src           = get16le(&frame[7]);
    result.payloadLength = frameLen - SHORT_ADDRESS_HEADER_LEN - FCS_LEN;
    result.frameLength   = frameLen;
    result.ranging       = false;
    return true;
}

static bool payloadMatches(const uint8_t* frame, uint16_t frameLen, const char* payload, size_t prefixLength,
                           size_t expectedPayloadLength)
{
    return (frame != nullptr) && (payload != nullptr) && (prefixLength <= expectedPayloadLength) &&
           (frameLen == shortAddressFrameLength(expectedPayloadLength)) &&
           (memcmp(&frame[SHORT_ADDRESS_HEADER_LEN], payload, prefixLength) == 0);
}

static M5Stamp_UWBError rxStatusToError(uint32_t status)
{
    if ((status & ((uint32_t)DWT_INT_RXFTO_BIT_MASK | (uint32_t)DWT_INT_RXPTO_BIT_MASK)) != 0) {
        return M5Stamp_UWBError::RxTimeout;
    }
    return M5Stamp_UWBError::RxError;
}

static void stopRadioAndClearStatus(uint32_t statusMask)
{
    dwt_forcetrxoff();
    dwt_writesysstatuslo(statusMask);
}

static void stopRadioAndClearRxStatus()
{
    stopRadioAndClearStatus(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_GOOD);
}

static void stopRadioAndClearTxStatus()
{
    stopRadioAndClearStatus(DWT_INT_TXFRS_BIT_MASK);
}

static void stopRadioAndClearIoStatus()
{
    stopRadioAndClearStatus(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_GOOD |
                            DWT_INT_TXFRS_BIT_MASK);
}

M5Stamp_UWB::M5Stamp_UWB() : _impl(new Impl())
{
}

M5Stamp_UWB::~M5Stamp_UWB()
{
    end();
    delete _impl;
}

bool M5Stamp_UWB::begin(const M5Stamp_UWBConfig& config, const M5Stamp_UWBPHYConfig& phy)
{
    if ((_active != nullptr) && (_active != this)) {
        setError(M5Stamp_UWBError::Busy);
        return false;
    }

    if ((config.spi == nullptr) || (config.pin_cs == M5STAMP_UWB_PIN_UNUSED)) {
        setError(M5Stamp_UWBError::InvalidConfig);
        return false;
    }

    _impl->config      = config;
    _impl->connected   = false;
    _impl->initialized = false;
    _impl->device_id   = 0;
    _active            = this;

    auto cleanupBeginFailure = [this]() {
        const M5Stamp_UWBError error = _impl->last_error;
        irqDisable();
        irqDetach();
        if (_active == this) {
            _active = nullptr;
        }
        if (_impl->config.pin_cs != M5STAMP_UWB_PIN_UNUSED) {
            digitalWrite(_impl->config.pin_cs, HIGH);
        }
        if (_impl->config.pin_wakeup != M5STAMP_UWB_PIN_UNUSED) {
            digitalWrite(_impl->config.pin_wakeup, LOW);
        }
        _impl->connected   = false;
        _impl->initialized = false;
        _impl->device_id   = 0;
        setError(error);
        return false;
    };

    pinMode(_impl->config.pin_cs, OUTPUT);
    digitalWrite(_impl->config.pin_cs, HIGH);

    if (_impl->config.pin_rst != M5STAMP_UWB_PIN_UNUSED) {
        pinMode(_impl->config.pin_rst, INPUT);
    }
    if (_impl->config.pin_wakeup != M5STAMP_UWB_PIN_UNUSED) {
        pinMode(_impl->config.pin_wakeup, OUTPUT);
        digitalWrite(_impl->config.pin_wakeup, LOW);
    }
    if (_impl->config.pin_irq != M5STAMP_UWB_PIN_UNUSED) {
        pinMode(_impl->config.pin_irq, INPUT);
        irqAttach(_impl->config.pin_irq);
    }
    if (_impl->config.pin_gp7 != M5STAMP_UWB_PIN_UNUSED) {
        pinMode(_impl->config.pin_gp7, INPUT);
    }

    if (_impl->config.begin_spi) {
        if ((_impl->config.pin_sck != M5STAMP_UWB_PIN_UNUSED) && (_impl->config.pin_miso != M5STAMP_UWB_PIN_UNUSED) &&
            (_impl->config.pin_mosi != M5STAMP_UWB_PIN_UNUSED)) {
            _impl->config.spi->begin(_impl->config.pin_sck, _impl->config.pin_miso, _impl->config.pin_mosi,
                                     _impl->config.pin_cs);
        } else {
            _impl->config.spi->begin();
        }
    }

    setSpiRate(_impl->config.spi_slow_hz);

    if (_impl->config.hard_reset_on_begin) {
        hardReset();
    }

    wakeupDeviceWithIoImpl();

    for (uint8_t i = 0; i < _impl->config.probe_retry_count; ++i) {
        const uint32_t dev_id = readRawDeviceId();
        if ((dev_id != 0x00000000UL) && (dev_id != 0xFFFFFFFFUL)) {
            break;
        }
        delay(_impl->config.probe_retry_delay_ms);
    }

    if (!probe()) {
        return cleanupBeginFailure();
    }

    if (!init(phy)) {
        return cleanupBeginFailure();
    }

    return true;
}

void M5Stamp_UWB::end()
{
    if (_impl == nullptr) {
        return;
    }

    const bool wasActive = (_active == this);
    const bool hadDriver = wasActive || _impl->connected || _impl->initialized;

    if (wasActive && _impl->initialized) {
        irqDisable();
        dwt_forcetrxoff();
    }

    irqDetach();

    if (wasActive) {
        _active = nullptr;
    }

    if (hadDriver && (_impl->config.pin_cs != M5STAMP_UWB_PIN_UNUSED)) {
        digitalWrite(_impl->config.pin_cs, HIGH);
    }
    if (hadDriver && (_impl->config.pin_wakeup != M5STAMP_UWB_PIN_UNUSED)) {
        digitalWrite(_impl->config.pin_wakeup, LOW);
    }

    _impl->connected   = false;
    _impl->initialized = false;
    _impl->device_id   = 0;
    setError(M5Stamp_UWBError::Ok);
}

bool M5Stamp_UWB::init(const M5Stamp_UWBPHYConfig& phy)
{
    if (!_impl->connected) {
        setError(M5Stamp_UWBError::ProbeFailed);
        return false;
    }
    if (_impl->device_id != M5STAMP_UWB_QM33120_DEVICE_ID) {
        setError(M5Stamp_UWBError::DeviceIdMismatch);
        return false;
    }
    if (!isValidChannel(phy.channel)) {
        setError(M5Stamp_UWBError::InvalidArgument);
        return false;
    }

    const M5Stamp_UWBPHYConfig resolvedPHY = resolvePHYConfig(phy);

    if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
        _impl->initialized = false;
        setError(M5Stamp_UWBError::InitFailed);
        return false;
    }

    dwt_config_t dwtConfig = toDwtConfig(resolvedPHY);
    if (dwt_configure(&dwtConfig) != DWT_SUCCESS) {
        _impl->initialized = false;
        setError(M5Stamp_UWBError::ConfigFailed);
        return false;
    }

    dwt_txconfig_t txConfig = toDwtTxConfig(resolvedPHY);
    dwt_configuretxrf(&txConfig);
    dwt_setrxantennadelay(resolvedPHY.rxAntennaDelay);
    dwt_settxantennadelay(resolvedPHY.txAntennaDelay);
    _impl->tx_antenna_delay = resolvedPHY.txAntennaDelay;
    if (resolvedPHY.enableLnaPa) {
        dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    }

    /* dwt_configure() cleared SYS_ENABLE_LO: re-arm the IRQ mask after it. */
    irqEnable();

    _impl->initialized = true;
    setError(M5Stamp_UWBError::Ok);
    return true;
}

bool M5Stamp_UWB::probe()
{
    struct dwt_driver_s* drivers[]        = {const_cast<struct dwt_driver_s*>(&dw3720_driver)};
    struct dwt_probe_s probe_interface    = {};
    probe_interface.dw                    = nullptr;
    probe_interface.spi                   = &_impl->dwt_spi;
    probe_interface.wakeup_device_with_io = wakeupDeviceWithIo;
    probe_interface.driver_list           = drivers;
    probe_interface.dw_driver_num         = sizeof(drivers) / sizeof(drivers[0]);

    for (uint8_t i = 0; i < _impl->config.probe_retry_count; ++i) {
        if (dwt_probe(&probe_interface) == DWT_SUCCESS) {
            _impl->connected = true;
            _impl->device_id = dwt_readdevid();
            setError(M5Stamp_UWBError::Ok);
            return true;
        }
        delay(_impl->config.probe_retry_delay_ms);
    }

    _impl->connected = false;
    _impl->device_id = readRawDeviceId();
    setError(M5Stamp_UWBError::ProbeFailed);
    return false;
}

void M5Stamp_UWB::hardReset(uint32_t reset_low_ms, uint32_t startup_ms)
{
    /* The chip loses its interrupt enable bits across reset. */
    irqDisable();
    if (_impl->config.pin_rst == M5STAMP_UWB_PIN_UNUSED) {
        delay(startup_ms);
        return;
    }

    pinMode(_impl->config.pin_rst, OUTPUT);
    digitalWrite(_impl->config.pin_rst, LOW);
    delay(reset_low_ms);
    pinMode(_impl->config.pin_rst, INPUT);
    delay(startup_ms);
}

uint32_t M5Stamp_UWB::deviceId() const
{
    return _impl->device_id;
}

uint32_t M5Stamp_UWB::readRawDeviceId()
{
    uint8_t header    = 0x00;
    uint8_t buffer[4] = {0};

    if (readFromSpiImpl(sizeof(header), &header, sizeof(buffer), buffer) != DWT_SUCCESS) {
        return 0;
    }

    return (static_cast<uint32_t>(buffer[3]) << 24UL) | (static_cast<uint32_t>(buffer[2]) << 16UL) |
           (static_cast<uint32_t>(buffer[1]) << 8UL) | static_cast<uint32_t>(buffer[0]);
}

const char* M5Stamp_UWB::chipName() const
{
    if (_impl->device_id == M5STAMP_UWB_QM33120_DEVICE_ID) {
        return "QM33120/DW3720";
    }
    return "Unknown";
}

M5Stamp_UWBTxResult M5Stamp_UWB::sendFrame(const char* payload, const M5Stamp_UWBFrameConfig& frame, uint32_t timeoutMs)
{
    if (payload == nullptr) {
        M5Stamp_UWBTxResult result;
        result.error = M5Stamp_UWBError::InvalidArgument;
        setError(result.error);
        return result;
    }
    return sendFrame(reinterpret_cast<const uint8_t*>(payload), strlen(payload), frame, timeoutMs);
}

M5Stamp_UWBTxResult M5Stamp_UWB::sendFrame(const uint8_t* payload, size_t length, const M5Stamp_UWBFrameConfig& frame,
                                           uint32_t timeoutMs)
{
    M5Stamp_UWBTxResult result;
    irqDrain();

    if (!_impl->initialized) {
        result.error = M5Stamp_UWBError::ConfigFailed;
        setError(result.error);
        return result;
    }
    if ((payload == nullptr) || (length == 0) || (length > 116)) {
        result.error = M5Stamp_UWBError::InvalidArgument;
        setError(result.error);
        return result;
    }

    uint8_t txFrame[128]   = {0};
    const size_t frameLen  = 9 + length;
    const uint8_t sequence = frame.useSequence ? frame.sequence : ++_impl->tx_sequence;
    txFrame[0]             = 0x41;
    txFrame[1]             = 0x88;
    txFrame[2]             = sequence;
    set16le(&txFrame[3], frame.panId);
    set16le(&txFrame[5], frame.dst);
    set16le(&txFrame[7], frame.src);
    memcpy(&txFrame[9], payload, length);

    stopRadioAndClearIoStatus();

    if (dwt_writetxdata(static_cast<uint16_t>(frameLen), txFrame, 0) != DWT_SUCCESS) {
        stopRadioAndClearTxStatus();
        result.error = M5Stamp_UWBError::TxDataFailed;
        setError(result.error);
        return result;
    }
    dwt_writetxfctrl(static_cast<uint16_t>(frameLen + FCS_LEN), 0, frame.ranging ? 1 : 0);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
        stopRadioAndClearTxStatus();
        result.error = M5Stamp_UWBError::TxStartFailed;
        setError(result.error);
        return result;
    }

    const uint32_t startMs = millis();
    while ((millis() - startMs) < timeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_TXFRS_BIT_MASK) != 0) {
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            result.success   = true;
            result.sequence  = txFrame[2];
            result.elapsedMs = millis() - startMs;
            result.error     = M5Stamp_UWBError::Ok;
            setError(M5Stamp_UWBError::Ok);
            return result;
        }
        irqWait(status, DWT_INT_TXFRS_BIT_MASK, startMs, timeoutMs);
    }

    stopRadioAndClearTxStatus();
    result.error = M5Stamp_UWBError::TxTimeout;
    setError(result.error);
    return result;
}

M5Stamp_UWBRxResult M5Stamp_UWB::receiveFrame(uint8_t* payload, size_t payloadSize, uint32_t timeoutMs)
{
    M5Stamp_UWBRxResult result;
    irqDrain();

    if (!_impl->initialized) {
        result.error = M5Stamp_UWBError::ConfigFailed;
        setError(result.error);
        return result;
    }
    if ((payload == nullptr) || (payloadSize == 0)) {
        result.error = M5Stamp_UWBError::InvalidArgument;
        setError(result.error);
        return result;
    }

    stopRadioAndClearIoStatus();

    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        stopRadioAndClearRxStatus();
        result.error = M5Stamp_UWBError::RxStartFailed;
        setError(result.error);
        return result;
    }

    const uint32_t startMs = millis();
    while ((millis() - startMs) < timeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rawFrame[128]   = {0};
            uint8_t rangingBit      = 0;
            const uint16_t frameLen = dwt_getframelength(&rangingBit);

            if (frameLen > sizeof(rawFrame)) {
                dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);
                result.error = M5Stamp_UWBError::RxBufferTooSmall;
                setError(result.error);
                return result;
            }

            dwt_readrxdata(rawFrame, frameLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);

            if (!parseShortAddressFrame(rawFrame, frameLen, result)) {
                result.elapsedMs = millis() - startMs;
                result.error     = M5Stamp_UWBError::FrameParseFailed;
                setError(result.error);
                return result;
            }
            result.ranging = (rangingBit != 0);
            if (result.payloadLength > payloadSize) {
                result.elapsedMs = millis() - startMs;
                result.error     = M5Stamp_UWBError::RxBufferTooSmall;
                setError(result.error);
                return result;
            }

            memcpy(payload, &rawFrame[9], result.payloadLength);
            result.success   = true;
            result.elapsedMs = millis() - startMs;
            result.error     = M5Stamp_UWBError::Ok;
            setError(M5Stamp_UWBError::Ok);
            return result;
        }

        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            stopRadioAndClearRxStatus();
            result.elapsedMs = millis() - startMs;
            result.error     = rxStatusToError(status);
            setError(result.error);
            return result;
        }
        irqWait(status, kIrqRxMask, startMs, timeoutMs);
    }

    stopRadioAndClearRxStatus();
    result.elapsedMs = millis() - startMs;
    result.error     = M5Stamp_UWBError::RxTimeout;
    setError(result.error);
    return result;
}

M5Stamp_UWBRangeResult M5Stamp_UWB::requestRange(const M5Stamp_UWBRangeConfig& range)
{
    static constexpr double speedOfLight   = 299702547.0;
    static constexpr uint64_t uusToDwtTime = 65536ULL;

    M5Stamp_UWBRangeResult result;
    irqDrain();
    if (!_impl->initialized) {
        result.error = M5Stamp_UWBError::ConfigFailed;
        setError(result.error);
        return result;
    }

    uint8_t pollFrame[12]       = {0};
    const uint8_t pollPayload[] = {'T', 'W', 'P'};
    const uint8_t pollSeq       = ++_impl->tx_sequence;
    buildShortAddressFrame(pollFrame, pollSeq, range.panId, range.initiatorAddress, range.responderAddress, pollPayload,
                           sizeof(pollPayload));

    stopRadioAndClearIoStatus();
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(range.responseRxAfterTxDelayUus);
    dwt_setrxtimeout(range.rxTimeoutUus);

    if (dwt_writetxdata(sizeof(pollFrame), pollFrame, 0) != DWT_SUCCESS) {
        stopRadioAndClearIoStatus();
        result.error = M5Stamp_UWBError::TxDataFailed;
        setError(result.error);
        return result;
    }
    dwt_writetxfctrl(sizeof(pollFrame) + FCS_LEN, 0, 1);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        stopRadioAndClearIoStatus();
        result.error = M5Stamp_UWBError::TxStartFailed;
        setError(result.error);
        return result;
    }

    const uint32_t startMs = millis();
    while ((millis() - startMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rawFrame[32] = {0};
            uint8_t rangingBit   = 0;
            uint16_t frameLen    = dwt_getframelength(&rangingBit);
            if (frameLen > sizeof(rawFrame)) {
                frameLen = sizeof(rawFrame);
            }
            dwt_readrxdata(rawFrame, frameLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | DWT_INT_TXFRS_BIT_MASK);

            M5Stamp_UWBRxResult parsed;
            if (!parseShortAddressFrame(rawFrame, frameLen, parsed) ||
                !payloadMatches(rawFrame, frameLen, "TWR", 3, 11) || (parsed.sequence != pollSeq) ||
                (parsed.panId != range.panId) || (parsed.src != range.responderAddress) ||
                (parsed.dst != range.initiatorAddress)) {
                stopRadioAndClearRxStatus();
                result.sequence  = parsed.sequence;
                result.elapsedMs = millis() - startMs;
                result.error     = M5Stamp_UWBError::RangeFrameMismatch;
                setError(result.error);
                return result;
            }

            const uint32_t pollTxTs = dwt_readtxtimestamplo32();
            const uint32_t respRxTs = dwt_readrxtimestamplo32(static_cast<dwt_ip_sts_segment_e>(0));
            const uint32_t pollRxTs = get32le(&rawFrame[12]);
            const uint32_t respTxTs = get32le(&rawFrame[16]);
            const int32_t rtdInit   = static_cast<int32_t>(respRxTs - pollTxTs);
            const int32_t rtdResp   = static_cast<int32_t>(respTxTs - pollRxTs);

            if ((rtdInit <= 0) || (rtdResp <= 0)) {
                result.elapsedMs = millis() - startMs;
                result.error     = M5Stamp_UWBError::RangeTimestampInvalid;
                setError(result.error);
                return result;
            }

            const double tof = ((static_cast<double>(rtdInit) - static_cast<double>(rtdResp)) / 2.0) * DWT_TIME_UNITS;
            result.distanceM = static_cast<float>(tof * speedOfLight);
            result.distanceMm =
                static_cast<int32_t>((result.distanceM * 1000.0f) + (result.distanceM >= 0 ? 0.5f : -0.5f));
            result.sequence  = pollSeq;
            result.elapsedMs = millis() - startMs;
            result.success   = true;
            result.error     = M5Stamp_UWBError::Ok;
            setError(M5Stamp_UWBError::Ok);
            return result;
        }

        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            stopRadioAndClearRxStatus();
            result.elapsedMs = millis() - startMs;
            result.error     = rxStatusToError(status);
            setError(result.error);
            return result;
        }
        irqWait(status, kIrqRxMask, startMs, range.hostTimeoutMs);
    }

    stopRadioAndClearRxStatus();
    result.elapsedMs = millis() - startMs;
    result.error     = M5Stamp_UWBError::RxTimeout;
    setError(result.error);
    return result;
}

M5Stamp_UWBResponderResult M5Stamp_UWB::respondRange(const M5Stamp_UWBRangeConfig& range)
{
    static constexpr uint64_t uusToDwtTime = 65536ULL;

    M5Stamp_UWBResponderResult result;
    irqDrain();
    if (!_impl->initialized) {
        result.error = M5Stamp_UWBError::ConfigFailed;
        setError(result.error);
        return result;
    }

    stopRadioAndClearIoStatus();
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);

    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        stopRadioAndClearRxStatus();
        result.error = M5Stamp_UWBError::RxStartFailed;
        setError(result.error);
        return result;
    }

    const uint32_t startMs = millis();
    while ((millis() - startMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t pollFrame[32] = {0};
            uint8_t rangingBit    = 0;
            uint16_t frameLen     = dwt_getframelength(&rangingBit);
            if (frameLen > sizeof(pollFrame)) {
                frameLen = sizeof(pollFrame);
            }
            dwt_readrxdata(pollFrame, frameLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);

            M5Stamp_UWBRxResult parsed;
            if (!parseShortAddressFrame(pollFrame, frameLen, parsed) ||
                !payloadMatches(pollFrame, frameLen, "TWP", 3, 3) || (parsed.panId != range.panId) ||
                (parsed.dst != range.responderAddress)) {
                stopRadioAndClearRxStatus();
                result.sequence  = parsed.sequence;
                result.requester = parsed.src;
                result.elapsedMs = millis() - startMs;
                result.error     = M5Stamp_UWBError::RangeFrameMismatch;
                setError(result.error);
                return result;
            }

            uint8_t ts[5] = {0};
            dwt_readrxtimestamp(ts, static_cast<dwt_ip_sts_segment_e>(0));
            const uint64_t pollRxTs = get40le(ts);
            const uint32_t respTxTime =
                static_cast<uint32_t>((pollRxTs + (range.responseTxDelayUus * uusToDwtTime)) >> 8);
            const uint64_t respTxTs =
                ((static_cast<uint64_t>(respTxTime & 0xFFFFFFFEUL)) << 8) + _impl->tx_antenna_delay;

            uint8_t respPayload[11] = {'T', 'W', 'R'};
            set32le(&respPayload[3], static_cast<uint32_t>(pollRxTs));
            set32le(&respPayload[7], static_cast<uint32_t>(respTxTs));

            uint8_t respFrame[20] = {0};
            buildShortAddressFrame(respFrame, parsed.sequence, range.panId, range.responderAddress, parsed.src,
                                   respPayload, sizeof(respPayload));

            dwt_setdelayedtrxtime(respTxTime);
            if (dwt_writetxdata(sizeof(respFrame), respFrame, 0) != DWT_SUCCESS) {
                stopRadioAndClearTxStatus();
                result.error = M5Stamp_UWBError::TxDataFailed;
                setError(result.error);
                return result;
            }
            dwt_writetxfctrl(sizeof(respFrame) + FCS_LEN, 0, 1);
            if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
                stopRadioAndClearTxStatus();
                result.error = M5Stamp_UWBError::TxStartFailed;
                setError(result.error);
                return result;
            }

            const uint32_t txStartMs = millis();
            while ((millis() - txStartMs) < 20) {
                const uint32_t txStatus = dwt_readsysstatuslo();
                if ((txStatus & DWT_INT_TXFRS_BIT_MASK) != 0) {
                    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
                    result.success   = true;
                    result.sequence  = parsed.sequence;
                    result.requester = parsed.src;
                    result.elapsedMs = millis() - startMs;
                    result.error     = M5Stamp_UWBError::Ok;
                    setError(M5Stamp_UWBError::Ok);
                    return result;
                }
                irqWait(txStatus, DWT_INT_TXFRS_BIT_MASK, txStartMs, 20);
            }

            stopRadioAndClearTxStatus();
            result.error = M5Stamp_UWBError::TxTimeout;
            setError(result.error);
            return result;
        }

        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            stopRadioAndClearRxStatus();
            result.elapsedMs = millis() - startMs;
            result.error     = rxStatusToError(status);
            setError(result.error);
            return result;
        }
        irqWait(status, kIrqRxMask, startMs, range.hostTimeoutMs);
    }

    stopRadioAndClearRxStatus();
    result.elapsedMs = millis() - startMs;
    result.error     = M5Stamp_UWBError::RxTimeout;
    setError(result.error);
    return result;
}

M5Stamp_UWBDSRangeResult M5Stamp_UWB::requestDSRange(const M5Stamp_UWBDSRangeConfig& range)
{
    static constexpr double speedOfLight   = 299702547.0;
    static constexpr uint64_t uusToDwtTime = 65536ULL;

    M5Stamp_UWBDSRangeResult result;
    irqDrain();
    if (!_impl->initialized) {
        result.error = M5Stamp_UWBError::ConfigFailed;
        setError(result.error);
        return result;
    }

    uint8_t pollFrame[12]       = {0};
    const uint8_t pollPayload[] = {'D', 'W', 'P'};
    const uint8_t pollSeq       = ++_impl->tx_sequence;
    buildShortAddressFrame(pollFrame, pollSeq, range.panId, range.initiatorAddress, range.responderAddress, pollPayload,
                           sizeof(pollPayload));

    stopRadioAndClearIoStatus();
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(range.responseRxAfterTxDelayUus);
    dwt_setrxtimeout(range.rxTimeoutUus);

    if (dwt_writetxdata(sizeof(pollFrame), pollFrame, 0) != DWT_SUCCESS) {
        stopRadioAndClearIoStatus();
        result.error = M5Stamp_UWBError::TxDataFailed;
        setError(result.error);
        return result;
    }
    dwt_writetxfctrl(sizeof(pollFrame) + FCS_LEN, 0, 1);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        stopRadioAndClearIoStatus();
        result.error = M5Stamp_UWBError::TxStartFailed;
        setError(result.error);
        return result;
    }

    uint8_t respFrame[32]  = {0};
    uint16_t respLen       = 0;
    const uint32_t startMs = millis();
    while ((millis() - startMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rangingBit = 0;
            respLen            = dwt_getframelength(&rangingBit);
            if (respLen > sizeof(respFrame)) {
                respLen = sizeof(respFrame);
            }
            dwt_readrxdata(respFrame, respLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | DWT_INT_TXFRS_BIT_MASK);
            break;
        }
        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            stopRadioAndClearRxStatus();
            result.elapsedMs = millis() - startMs;
            result.error     = rxStatusToError(status);
            setError(result.error);
            return result;
        }
        irqWait(status, kIrqRxMask, startMs, range.hostTimeoutMs);
    }

    M5Stamp_UWBRxResult parsed;
    if (!parseShortAddressFrame(respFrame, respLen, parsed) || !payloadMatches(respFrame, respLen, "DWR", 3, 11) ||
        (parsed.sequence != pollSeq) || (parsed.panId != range.panId) || (parsed.src != range.responderAddress) ||
        (parsed.dst != range.initiatorAddress)) {
        stopRadioAndClearRxStatus();
        result.sequence  = parsed.sequence;
        result.elapsedMs = millis() - startMs;
        result.error     = respLen == 0 ? M5Stamp_UWBError::RxTimeout : M5Stamp_UWBError::RangeFrameMismatch;
        setError(result.error);
        return result;
    }

    const uint64_t pollTxTs    = readTxTimestamp64();
    const uint64_t respRxTs    = readRxTimestamp64();
    const uint32_t finalTxTime = static_cast<uint32_t>((respRxTs + (range.finalTxDelayUus * uusToDwtTime)) >> 8);
    const uint64_t finalTxTs   = ((static_cast<uint64_t>(finalTxTime & 0xFFFFFFFEUL)) << 8) + _impl->tx_antenna_delay;

    uint8_t finalPayload[15] = {'D', 'W', 'F'};
    set32le(&finalPayload[3], static_cast<uint32_t>(pollTxTs));
    set32le(&finalPayload[7], static_cast<uint32_t>(respRxTs));
    set32le(&finalPayload[11], static_cast<uint32_t>(finalTxTs));

    uint8_t finalFrame[24] = {0};
    buildShortAddressFrame(finalFrame, pollSeq, range.panId, range.initiatorAddress, range.responderAddress,
                           finalPayload, sizeof(finalPayload));

    dwt_setdelayedtrxtime(finalTxTime);
    dwt_setrxaftertxdelay(range.resultRxAfterFinalTxDelayUus);
    dwt_setrxtimeout(range.rxTimeoutUus);
    if (dwt_writetxdata(sizeof(finalFrame), finalFrame, 0) != DWT_SUCCESS) {
        stopRadioAndClearIoStatus();
        result.error = M5Stamp_UWBError::TxDataFailed;
        setError(result.error);
        return result;
    }
    dwt_writetxfctrl(sizeof(finalFrame) + FCS_LEN, 0, 1);
    if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        stopRadioAndClearIoStatus();
        result.error = M5Stamp_UWBError::TxStartFailed;
        setError(result.error);
        return result;
    }

    uint8_t distFrame[32]        = {0};
    uint16_t distLen             = 0;
    const uint32_t resultStartMs = millis();
    while ((millis() - resultStartMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rangingBit = 0;
            distLen            = dwt_getframelength(&rangingBit);
            if (distLen > sizeof(distFrame)) {
                distLen = sizeof(distFrame);
            }
            dwt_readrxdata(distFrame, distLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | DWT_INT_TXFRS_BIT_MASK);
            break;
        }
        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            stopRadioAndClearRxStatus();
            result.sequence  = pollSeq;
            result.elapsedMs = millis() - startMs;
            result.error     = rxStatusToError(status);
            setError(result.error);
            return result;
        }
        irqWait(status, kIrqRxMask, resultStartMs, range.hostTimeoutMs);
    }

    if (!parseShortAddressFrame(distFrame, distLen, parsed) || !payloadMatches(distFrame, distLen, "DWD", 3, 7) ||
        (parsed.sequence != pollSeq) || (parsed.panId != range.panId) || (parsed.src != range.responderAddress) ||
        (parsed.dst != range.initiatorAddress)) {
        stopRadioAndClearRxStatus();
        result.sequence  = parsed.sequence;
        result.elapsedMs = millis() - startMs;
        result.error     = distLen == 0 ? M5Stamp_UWBError::RxTimeout : M5Stamp_UWBError::RangeFrameMismatch;
        setError(result.error);
        return result;
    }

    result.distanceMm = static_cast<int32_t>(get32le(&distFrame[12]));
    result.distanceM  = static_cast<float>(result.distanceMm) / 1000.0f;
    result.sequence   = pollSeq;
    result.elapsedMs  = millis() - startMs;
    result.success    = true;
    result.error      = M5Stamp_UWBError::Ok;
    setError(M5Stamp_UWBError::Ok);
    (void)speedOfLight;
    return result;
}

M5Stamp_UWBDSResponderResult M5Stamp_UWB::respondDSRange(const M5Stamp_UWBDSRangeConfig& range)
{
    static constexpr double speedOfLight   = 299702547.0;
    static constexpr uint64_t uusToDwtTime = 65536ULL;

    M5Stamp_UWBDSResponderResult result;
    irqDrain();
    if (!_impl->initialized) {
        result.error = M5Stamp_UWBError::ConfigFailed;
        setError(result.error);
        return result;
    }

    stopRadioAndClearIoStatus();
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);

    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        stopRadioAndClearRxStatus();
        result.error = M5Stamp_UWBError::RxStartFailed;
        setError(result.error);
        return result;
    }

    uint8_t pollFrame[32] = {0};
    uint16_t pollLen      = 0;
    M5Stamp_UWBRxResult parsed;
    const uint32_t startMs = millis();
    while ((millis() - startMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rangingBit = 0;
            pollLen            = dwt_getframelength(&rangingBit);
            if (pollLen > sizeof(pollFrame)) {
                pollLen = sizeof(pollFrame);
            }
            dwt_readrxdata(pollFrame, pollLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);
            break;
        }
        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            stopRadioAndClearRxStatus();
            result.elapsedMs = millis() - startMs;
            result.error     = rxStatusToError(status);
            setError(result.error);
            return result;
        }
        irqWait(status, kIrqRxMask, startMs, range.hostTimeoutMs);
    }

    if (!parseShortAddressFrame(pollFrame, pollLen, parsed) || !payloadMatches(pollFrame, pollLen, "DWP", 3, 3) ||
        (parsed.panId != range.panId) || (parsed.dst != range.responderAddress)) {
        stopRadioAndClearRxStatus();
        result.sequence  = parsed.sequence;
        result.requester = parsed.src;
        result.elapsedMs = millis() - startMs;
        result.error     = pollLen == 0 ? M5Stamp_UWBError::RxTimeout : M5Stamp_UWBError::RangeFrameMismatch;
        setError(result.error);
        return result;
    }

    const uint64_t pollRxTs     = readRxTimestamp64();
    const uint32_t respTxTime   = static_cast<uint32_t>((pollRxTs + (range.responseTxDelayUus * uusToDwtTime)) >> 8);
    const uint64_t respTxTsPlan = ((static_cast<uint64_t>(respTxTime & 0xFFFFFFFEUL)) << 8) + _impl->tx_antenna_delay;

    uint8_t respPayload[11] = {'D', 'W', 'R'};
    set32le(&respPayload[3], static_cast<uint32_t>(pollRxTs));
    set32le(&respPayload[7], static_cast<uint32_t>(respTxTsPlan));

    uint8_t respFrame[20] = {0};
    buildShortAddressFrame(respFrame, parsed.sequence, range.panId, range.responderAddress, parsed.src, respPayload,
                           sizeof(respPayload));

    dwt_setdelayedtrxtime(respTxTime);
    dwt_setrxaftertxdelay(range.finalRxAfterResponseTxDelayUus);
    dwt_setrxtimeout(range.rxTimeoutUus);
    if (dwt_writetxdata(sizeof(respFrame), respFrame, 0) != DWT_SUCCESS) {
        stopRadioAndClearIoStatus();
        result.error = M5Stamp_UWBError::TxDataFailed;
        setError(result.error);
        return result;
    }
    dwt_writetxfctrl(sizeof(respFrame) + FCS_LEN, 0, 1);
    if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        stopRadioAndClearIoStatus();
        result.error = M5Stamp_UWBError::TxStartFailed;
        setError(result.error);
        return result;
    }

    uint8_t finalFrame[32]      = {0};
    uint16_t finalLen           = 0;
    const uint32_t finalStartMs = millis();
    while ((millis() - finalStartMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rangingBit = 0;
            finalLen           = dwt_getframelength(&rangingBit);
            if (finalLen > sizeof(finalFrame)) {
                finalLen = sizeof(finalFrame);
            }
            dwt_readrxdata(finalFrame, finalLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | DWT_INT_TXFRS_BIT_MASK);
            break;
        }
        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            stopRadioAndClearRxStatus();
            result.sequence  = parsed.sequence;
            result.requester = parsed.src;
            result.elapsedMs = millis() - startMs;
            result.error     = rxStatusToError(status);
            setError(result.error);
            return result;
        }
        irqWait(status, kIrqRxMask, finalStartMs, range.hostTimeoutMs);
    }

    M5Stamp_UWBRxResult finalParsed;
    if (!parseShortAddressFrame(finalFrame, finalLen, finalParsed) ||
        !payloadMatches(finalFrame, finalLen, "DWF", 3, 15) || (finalParsed.sequence != parsed.sequence) ||
        (finalParsed.panId != range.panId) || (finalParsed.src != parsed.src) ||
        (finalParsed.dst != range.responderAddress)) {
        stopRadioAndClearRxStatus();
        result.sequence  = finalParsed.sequence;
        result.requester = parsed.src;
        result.elapsedMs = millis() - startMs;
        result.error     = finalLen == 0 ? M5Stamp_UWBError::RxTimeout : M5Stamp_UWBError::RangeFrameMismatch;
        setError(result.error);
        return result;
    }

    const uint64_t finalRxTs   = readRxTimestamp64();
    const uint32_t pollTxTs32  = get32le(&finalFrame[12]);
    const uint32_t respRxTs32  = get32le(&finalFrame[16]);
    const uint32_t finalTxTs32 = get32le(&finalFrame[20]);
    const uint32_t pollRxTs32  = static_cast<uint32_t>(pollRxTs);
    const uint32_t respTxTs32  = static_cast<uint32_t>(readTxTimestamp64());
    const uint32_t finalRxTs32 = static_cast<uint32_t>(finalRxTs);

    const double ra          = static_cast<double>(static_cast<uint32_t>(respRxTs32 - pollTxTs32));
    const double rb          = static_cast<double>(static_cast<uint32_t>(finalRxTs32 - respTxTs32));
    const double da          = static_cast<double>(static_cast<uint32_t>(finalTxTs32 - respRxTs32));
    const double db          = static_cast<double>(static_cast<uint32_t>(respTxTs32 - pollRxTs32));
    const double denominator = ra + rb + da + db;
    if (denominator <= 0.0) {
        result.sequence  = parsed.sequence;
        result.requester = parsed.src;
        result.elapsedMs = millis() - startMs;
        result.error     = M5Stamp_UWBError::RangeTimestampInvalid;
        setError(result.error);
        return result;
    }

    const double tofDtu = ((ra * rb) - (da * db)) / denominator;
    const double tof    = tofDtu * DWT_TIME_UNITS;
    result.distanceM    = static_cast<float>(tof * speedOfLight);
    result.distanceMm   = static_cast<int32_t>((result.distanceM * 1000.0f) + (result.distanceM >= 0 ? 0.5f : -0.5f));

    uint8_t distPayload[7] = {'D', 'W', 'D'};
    set32le(&distPayload[3], static_cast<uint32_t>(result.distanceMm));
    uint8_t distFrame[16] = {0};
    buildShortAddressFrame(distFrame, parsed.sequence, range.panId, range.responderAddress, parsed.src, distPayload,
                           sizeof(distPayload));

    uint8_t sentCount         = 0;
    const uint8_t repeatCount = range.resultRepeatCount == 0 ? 1 : range.resultRepeatCount;
    for (uint8_t i = 0; i < repeatCount; ++i) {
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        if (dwt_writetxdata(sizeof(distFrame), distFrame, 0) != DWT_SUCCESS) {
            continue;
        }
        dwt_writetxfctrl(sizeof(distFrame) + FCS_LEN, 0, 0);
        if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
            stopRadioAndClearTxStatus();
            continue;
        }

        bool txDone              = false;
        const uint32_t txStartMs = millis();
        while ((millis() - txStartMs) < 20) {
            const uint32_t txStatus = dwt_readsysstatuslo();
            if ((txStatus & DWT_INT_TXFRS_BIT_MASK) != 0) {
                dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
                txDone = true;
                sentCount++;
                break;
            }
            irqWait(txStatus, DWT_INT_TXFRS_BIT_MASK, txStartMs, 20);
        }
        if (!txDone) {
            stopRadioAndClearTxStatus();
        }
        if (i + 1 < repeatCount) {
            delay(range.resultRepeatGapMs);
        }
    }

    result.success   = sentCount > 0;
    result.sequence  = parsed.sequence;
    result.requester = parsed.src;
    result.elapsedMs = millis() - startMs;
    result.error     = result.success ? M5Stamp_UWBError::Ok : M5Stamp_UWBError::TxTimeout;
    setError(result.error);
    return result;
}

bool M5Stamp_UWB::isConnected() const
{
    return _impl->connected;
}

bool M5Stamp_UWB::isIrqActive() const
{
    return s_irq_attached && s_irq_enabled;
}

bool M5Stamp_UWB::isInterruptDriven() const
{
    return isIrqActive();
}

bool M5Stamp_UWB::isInitialized() const
{
    return _impl->initialized;
}

M5Stamp_UWBError M5Stamp_UWB::lastError() const
{
    return _impl->last_error;
}

const char* M5Stamp_UWB::lastErrorName() const
{
    switch (_impl->last_error) {
        case M5Stamp_UWBError::Ok:
            return "OK";
        case M5Stamp_UWBError::InvalidConfig:
            return "INVALID_CONFIG";
        case M5Stamp_UWBError::SpiNotReady:
            return "SPI_NOT_READY";
        case M5Stamp_UWBError::ProbeFailed:
            return "PROBE_FAILED";
        case M5Stamp_UWBError::DeviceIdMismatch:
            return "DEVICE_ID_MISMATCH";
        case M5Stamp_UWBError::InitFailed:
            return "INIT_FAILED";
        case M5Stamp_UWBError::ConfigFailed:
            return "CONFIG_FAILED";
        case M5Stamp_UWBError::TxDataFailed:
            return "TX_DATA_FAILED";
        case M5Stamp_UWBError::TxStartFailed:
            return "TX_START_FAILED";
        case M5Stamp_UWBError::TxTimeout:
            return "TX_TIMEOUT";
        case M5Stamp_UWBError::RxStartFailed:
            return "RX_START_FAILED";
        case M5Stamp_UWBError::RxTimeout:
            return "RX_TIMEOUT";
        case M5Stamp_UWBError::RxError:
            return "RX_ERROR";
        case M5Stamp_UWBError::RxBufferTooSmall:
            return "RX_BUFFER_TOO_SMALL";
        case M5Stamp_UWBError::FrameParseFailed:
            return "FRAME_PARSE_FAILED";
        case M5Stamp_UWBError::RangeTimestampInvalid:
            return "RANGE_TIMESTAMP_INVALID";
        case M5Stamp_UWBError::RangeFrameMismatch:
            return "RANGE_FRAME_MISMATCH";
        case M5Stamp_UWBError::InvalidArgument:
            return "INVALID_ARGUMENT";
        case M5Stamp_UWBError::Busy:
            return "BUSY";
        default:
            return "UNKNOWN";
    }
}

const M5Stamp_UWBConfig& M5Stamp_UWB::config() const
{
    return _impl->config;
}

int32_t M5Stamp_UWB::readFromSpi(uint16_t header_length, uint8_t* header_buffer, uint16_t read_length,
                                 uint8_t* read_buffer)
{
    if (_active == nullptr) {
        return DWT_ERROR;
    }
    return _active->readFromSpiImpl(header_length, header_buffer, read_length, read_buffer);
}

int32_t M5Stamp_UWB::writeToSpi(uint16_t header_length, const uint8_t* header_buffer, uint16_t body_length,
                                const uint8_t* body_buffer)
{
    if (_active == nullptr) {
        return DWT_ERROR;
    }
    return _active->writeToSpiImpl(header_length, header_buffer, body_length, body_buffer, nullptr);
}

int32_t M5Stamp_UWB::writeToSpiWithCrc(uint16_t header_length, const uint8_t* header_buffer, uint16_t body_length,
                                       const uint8_t* body_buffer, uint8_t crc8)
{
    if (_active == nullptr) {
        return DWT_ERROR;
    }
    return _active->writeToSpiImpl(header_length, header_buffer, body_length, body_buffer, &crc8);
}

void M5Stamp_UWB::setSlowRate()
{
    if (_active != nullptr) {
        _active->setSpiRate(_active->_impl->config.spi_slow_hz);
    }
}

void M5Stamp_UWB::setFastRate()
{
    if (_active != nullptr) {
        _active->setSpiRate(_active->_impl->config.spi_fast_hz);
    }
}

void M5Stamp_UWB::wakeupDeviceWithIo()
{
    if (_active != nullptr) {
        _active->wakeupDeviceWithIoImpl();
    }
}

int32_t M5Stamp_UWB::readFromSpiImpl(uint16_t header_length, uint8_t* header_buffer, uint16_t read_length,
                                     uint8_t* read_buffer)
{
    if ((_impl->config.spi == nullptr) || !validCs() || (header_buffer == nullptr) || (read_buffer == nullptr)) {
        return DWT_ERROR;
    }

    _impl->config.spi->beginTransaction(_impl->spi_settings);
    digitalWrite(_impl->config.pin_cs, LOW);
    for (uint16_t i = 0; i < header_length; ++i) {
        _impl->config.spi->transfer(header_buffer[i]);
    }
    for (uint16_t i = 0; i < read_length; ++i) {
        read_buffer[i] = _impl->config.spi->transfer(0x00);
    }
    digitalWrite(_impl->config.pin_cs, HIGH);
    _impl->config.spi->endTransaction();

    return DWT_SUCCESS;
}

int32_t M5Stamp_UWB::writeToSpiImpl(uint16_t header_length, const uint8_t* header_buffer, uint16_t body_length,
                                    const uint8_t* body_buffer, const uint8_t* crc8)
{
    if ((_impl->config.spi == nullptr) || !validCs() || (header_buffer == nullptr)) {
        return DWT_ERROR;
    }

    _impl->config.spi->beginTransaction(_impl->spi_settings);
    digitalWrite(_impl->config.pin_cs, LOW);
    for (uint16_t i = 0; i < header_length; ++i) {
        _impl->config.spi->transfer(header_buffer[i]);
    }
    for (uint16_t i = 0; i < body_length; ++i) {
        _impl->config.spi->transfer(body_buffer != nullptr ? body_buffer[i] : 0x00);
    }
    if (crc8 != nullptr) {
        _impl->config.spi->transfer(*crc8);
    }
    digitalWrite(_impl->config.pin_cs, HIGH);
    _impl->config.spi->endTransaction();

    return DWT_SUCCESS;
}

void M5Stamp_UWB::setSpiRate(uint32_t hz)
{
    _impl->spi_settings = SPISettings(hz, MSBFIRST, SPI_MODE0);
}

void M5Stamp_UWB::wakeupDeviceWithIoImpl()
{
    if (_impl->config.pin_wakeup != M5STAMP_UWB_PIN_UNUSED) {
        digitalWrite(_impl->config.pin_wakeup, HIGH);
        delay(2);
        digitalWrite(_impl->config.pin_wakeup, LOW);
        delay(5);
        return;
    }

    if (validCs()) {
        digitalWrite(_impl->config.pin_cs, LOW);
        delay(2);
        digitalWrite(_impl->config.pin_cs, HIGH);
        delay(5);
    }
}

bool M5Stamp_UWB::validCs() const
{
    return _impl->config.pin_cs != M5STAMP_UWB_PIN_UNUSED;
}

void M5Stamp_UWB::setError(M5Stamp_UWBError error)
{
    _impl->last_error = error;
}
