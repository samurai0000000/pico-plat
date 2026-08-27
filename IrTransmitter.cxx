/*
 * IrTransmitter.cxx
 *
 * Copyright (C) 2026, Charles Chiou
 */

#include <cstring>
#include <IrTransmitter.hxx>
#include <ir_tx.pio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"

IrTransmitter::IrTransmitter(uint pin, PIO pio, int sm)
    : _pin(pin)
    , _pio(pio)
    , _sm(sm)
    , _offset(0)
    , _curFreq(38000)
    , _initialized(false)
{

}

IrTransmitter::~IrTransmitter()
{
    deinit();
}

bool IrTransmitter::init(void)
{
    if (_initialized) {
        return true;
    }

    if (!pio_can_add_program(_pio, &ir_tx_program)) {
        return false;
    }

    _offset = pio_add_program(_pio, &ir_tx_program);

    if (_sm < 0) {
        _sm = pio_claim_unused_sm(_pio, false);
        if (_sm < 0) {
            pio_remove_program(_pio, &ir_tx_program, _offset);
            return false;
        }
    }

    ir_tx_program_init(_pio, (uint) _sm, _offset, _pin, _curFreq);
    pio_sm_set_enabled(_pio, (uint) _sm, true);

    _initialized = true;
    return true;
}

void IrTransmitter::deinit(void)
{
    if (!_initialized) {
        return;
    }

    pio_sm_set_enabled(_pio, (uint) _sm, false);
    pio_remove_program(_pio, &ir_tx_program, _offset);
    pio_sm_unclaim(_pio, (uint) _sm);

    gpio_init(_pin);
    gpio_set_dir(_pin, GPIO_OUT);
    gpio_put(_pin, false);

    _initialized = false;
}

uint IrTransmitter::getPin(void) const
{
    return _pin;
}

PIO IrTransmitter::getPio(void) const
{
    return _pio;
}

int IrTransmitter::getSm(void) const
{
    return _sm;
}

bool IrTransmitter::isInitialized(void) const
{
    return _initialized;
}

void IrTransmitter::setFrequency(uint32_t freq_hz)
{
    if (freq_hz == 0) {
        freq_hz = 38000;
    }

    if (_curFreq != freq_hz) {
        _curFreq = freq_hz;
        if (_initialized) {
            ir_tx_set_freq(_pio, (uint) _sm, _curFreq);
        }
    }
}

void IrTransmitter::sendPulse(bool isMark, uint32_t duration_us, uint32_t freq_hz)
{
    if (!_initialized && !init()) {
        return;
    }

    setFrequency(freq_hz);

    if (duration_us == 0) {
        return;
    }

    // Number of carrier cycles = (duration_us * freq_hz + 500000) / 1000000
    uint32_t cycles = (uint32_t)(((uint64_t)duration_us * freq_hz + 500000ULL) / 1000000ULL);
    if (cycles == 0) {
        cycles = 1;
    }

    uint32_t word = (isMark ? (1u << 31) : 0u) | ((cycles - 1) & 0x7FFFFFFFu);
    pio_sm_put_blocking(_pio, (uint) _sm, word);
}

void IrTransmitter::sendBits(uint64_t data, size_t nbits, uint32_t mark_us,
                            uint32_t zero_space_us, uint32_t one_space_us,
                            bool msb_first, uint32_t freq_hz)
{
    if (msb_first) {
        for (int i = (int)nbits - 1; i >= 0; i--) {
            bool bit = (data >> i) & 1ULL;
            sendPulse(true, mark_us, freq_hz);
            sendPulse(false, bit ? one_space_us : zero_space_us, freq_hz);
        }
    } else {
        for (size_t i = 0; i < nbits; i++) {
            bool bit = (data >> i) & 1ULL;
            sendPulse(true, mark_us, freq_hz);
            sendPulse(false, bit ? one_space_us : zero_space_us, freq_hz);
        }
    }
}

void IrTransmitter::sendBytes(const uint8_t *data, size_t nbytes, uint32_t mark_us,
                             uint32_t zero_space_us, uint32_t one_space_us,
                             bool msb_first, uint32_t freq_hz)
{
    for (size_t i = 0; i < nbytes; i++) {
        sendBits(data[i], 8, mark_us, zero_space_us, one_space_us, msb_first, freq_hz);
    }
}

void IrTransmitter::sendRaw(const uint32_t *pulses_us, size_t count, uint32_t freq_hz)
{
    if (pulses_us == NULL || count == 0) {
        return;
    }

    if (!_initialized && !init()) {
        return;
    }

    setFrequency(freq_hz);

    for (size_t i = 0; i < count; i++) {
        bool isMark = (i % 2) == 0;
        sendPulse(isMark, pulses_us[i], freq_hz);
    }
}

void IrTransmitter::sendPanasonicHvac(const struct IrHvacState &state, uint16_t repeat)
{
    // Panasonic AC 216-bit / 27-byte Protocol
    // Carrier: 36700 Hz (or 38000 Hz)
    const uint32_t freq = 36700;
    const uint32_t hdrMark = 3456;
    const uint32_t hdrSpace = 1728;
    const uint32_t bitMark = 432;
    const uint32_t oneSpace = 1296;
    const uint32_t zeroSpace = 432;
    const uint32_t sectionGap = 10000;

    uint8_t bytes[27];
    memset(bytes, 0, sizeof(bytes));

    // Block 1 (8 bytes)
    bytes[0] = 0x02;
    bytes[1] = 0x20;
    bytes[2] = 0xE0;
    bytes[3] = 0x04;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x06;

    // Block 2 (19 bytes)
    bytes[8]  = 0x02;
    bytes[9]  = 0x20;
    bytes[10] = 0xE0;
    bytes[11] = 0x04;
    bytes[12] = 0x00;

    // Byte 13: Power & Mode
    // Mode: 0=Auto, 2=Dry, 3=Cool (or 2 in some remotes, 3 in std), 4=Heat, 6=Fan
    uint8_t modeVal = 0;
    switch (state.mode) {
    case 0: modeVal = 0; break; // Auto
    case 1: modeVal = 3; break; // Cool
    case 2: modeVal = 2; break; // Dry
    case 3: modeVal = 4; break; // Heat
    case 4: modeVal = 6; break; // Fan
    default: modeVal = (state.mode & 0x07); break;
    }
    bytes[13] = (state.power ? 0x01 : 0x00) | ((modeVal & 0x07) << 4);

    // Byte 14: Target Temperature (16..30 C) in bits 1..5
    uint8_t temp = (uint8_t)(state.targetTemp + 0.5f);
    if (temp < 16) temp = 16;
    if (temp > 30) temp = 30;
    bytes[14] = (temp & 0x1F) << 1;

    // Byte 15
    bytes[15] = 0x80;

    // Byte 16: Fan Speed (bits 4..7) & Vertical Swing (bits 0..3)
    uint8_t fan = (state.fanSpeed == 0) ? 0x0A : (state.fanSpeed & 0x0F);
    uint8_t swingV = (state.swingV == 0) ? 0x0F : (state.swingV & 0x0F);
    bytes[16] = (fan << 4) | (swingV & 0x0F);

    // Byte 17: Horizontal Swing
    uint8_t swingH = (state.swingH == 0) ? 0x0D : (state.swingH & 0x0F);
    bytes[17] = swingH & 0x0F;

    bytes[18] = 0x00;
    bytes[19] = 0x0E;
    bytes[20] = 0xE0;

    // Byte 21: Powerful & Quiet
    bytes[21] = (state.powerful ? 0x20 : 0x00) | (state.quiet ? 0x01 : 0x00);

    bytes[22] = 0x00;
    bytes[23] = 0x81;
    bytes[24] = 0x00;
    bytes[25] = 0x00;

    // Byte 26: Checksum of bytes 8..25
    uint8_t checksum = 0;
    for (size_t i = 8; i < 26; i++) {
        checksum += bytes[i];
    }
    bytes[26] = checksum;

    for (uint16_t r = 0; r <= repeat; r++) {
        // Section 1
        sendPulse(true, hdrMark, freq);
        sendPulse(false, hdrSpace, freq);
        sendBytes(bytes, 8, bitMark, zeroSpace, oneSpace, false, freq);
        sendPulse(true, bitMark, freq);
        sendPulse(false, sectionGap, freq);

        // Section 2
        sendPulse(true, hdrMark, freq);
        sendPulse(false, hdrSpace, freq);
        sendBytes(bytes + 8, 19, bitMark, zeroSpace, oneSpace, false, freq);
        sendPulse(true, bitMark, freq);
        sendPulse(false, sectionGap, freq);
    }
}

void IrTransmitter::sendPanasonic48(uint16_t manufacturer, uint8_t device,
                                   uint8_t subdevice, uint8_t function, uint16_t repeat)
{
    const uint32_t freq = 36700;
    const uint32_t hdrMark = 3456;
    const uint32_t hdrSpace = 1728;
    const uint32_t bitMark = 432;
    const uint32_t oneSpace = 1296;
    const uint32_t zeroSpace = 432;
    const uint32_t gap = 74736;

    uint8_t checksum = device ^ subdevice ^ function;
    uint8_t bytes[6];
    bytes[0] = manufacturer & 0xFF;
    bytes[1] = (manufacturer >> 8) & 0xFF;
    bytes[2] = device;
    bytes[3] = subdevice;
    bytes[4] = function;
    bytes[5] = checksum;

    for (uint16_t r = 0; r <= repeat; r++) {
        sendPulse(true, hdrMark, freq);
        sendPulse(false, hdrSpace, freq);
        sendBytes(bytes, 6, bitMark, zeroSpace, oneSpace, false, freq);
        sendPulse(true, bitMark, freq);
        sendPulse(false, gap, freq);
    }
}

void IrTransmitter::sendPanasonicTv(enum TvCommand cmd, uint16_t repeat)
{
    uint8_t func = 0;
    switch (cmd) {
    case TV_CMD_POWER:
    case TV_CMD_POWER_ON:
    case TV_CMD_POWER_OFF:
        func = 0x3D; // Power toggle
        break;
    case TV_CMD_VOL_UP:
        func = 0x20;
        break;
    case TV_CMD_VOL_DOWN:
        func = 0x21;
        break;
    case TV_CMD_MUTE:
        func = 0x32;
        break;
    case TV_CMD_CHAN_UP:
        func = 0x22;
        break;
    case TV_CMD_CHAN_DOWN:
        func = 0x23;
        break;
    case TV_CMD_INPUT:
        func = 0x0A;
        break;
    case TV_CMD_DIGIT_0:
        func = 0x19;
        break;
    case TV_CMD_DIGIT_1:
    case TV_CMD_DIGIT_2:
    case TV_CMD_DIGIT_3:
    case TV_CMD_DIGIT_4:
    case TV_CMD_DIGIT_5:
    case TV_CMD_DIGIT_6:
    case TV_CMD_DIGIT_7:
    case TV_CMD_DIGIT_8:
    case TV_CMD_DIGIT_9:
        func = 0x10 + (cmd - TV_CMD_DIGIT_1);
        break;
    default:
        return;
    }

    sendPanasonic48(0x4004, 0x01, 0x00, func, repeat);
}

void IrTransmitter::sendSonySirc(uint32_t command, uint32_t address, uint16_t nbits, uint16_t repeat)
{
    const uint32_t freq = 40000;
    const uint32_t hdrMark = 2400;
    const uint32_t space = 600;
    const uint32_t oneMark = 1200;
    const uint32_t zeroMark = 600;
    const uint32_t minGap = 10000;

    // Sony sends 7-bit command LSB-first followed by address bits LSB-first
    uint32_t raw = (command & 0x7F) | ((address & 0x1FFF) << 7);

    for (uint16_t r = 0; r <= repeat; r++) {
        sendPulse(true, hdrMark, freq);
        sendPulse(false, space, freq);

        for (size_t i = 0; i < nbits; i++) {
            bool bit = (raw >> i) & 1;
            sendPulse(true, bit ? oneMark : zeroMark, freq);
            sendPulse(false, space, freq);
        }

        sendPulse(false, minGap, freq);
    }
}

void IrTransmitter::sendSonyTv(enum TvCommand cmd, uint16_t repeat)
{
    uint32_t commandCode = 0;
    switch (cmd) {
    case TV_CMD_POWER:
        commandCode = 0x15; // Power toggle
        break;
    case TV_CMD_POWER_ON:
        commandCode = 0x2F; // Power on
        break;
    case TV_CMD_POWER_OFF:
        commandCode = 0x2E; // Power off
        break;
    case TV_CMD_VOL_UP:
        commandCode = 0x12;
        break;
    case TV_CMD_VOL_DOWN:
        commandCode = 0x13;
        break;
    case TV_CMD_MUTE:
        commandCode = 0x14;
        break;
    case TV_CMD_CHAN_UP:
        commandCode = 0x10;
        break;
    case TV_CMD_CHAN_DOWN:
        commandCode = 0x11;
        break;
    case TV_CMD_INPUT:
        commandCode = 0x25;
        break;
    case TV_CMD_DIGIT_0:
        commandCode = 0x09;
        break;
    case TV_CMD_DIGIT_1:
    case TV_CMD_DIGIT_2:
    case TV_CMD_DIGIT_3:
    case TV_CMD_DIGIT_4:
    case TV_CMD_DIGIT_5:
    case TV_CMD_DIGIT_6:
    case TV_CMD_DIGIT_7:
    case TV_CMD_DIGIT_8:
    case TV_CMD_DIGIT_9:
        commandCode = (cmd - TV_CMD_DIGIT_1);
        break;
    default:
        return;
    }

    sendSonySirc(commandCode, 1, 12, repeat);
}

void IrTransmitter::sendSamsung32(uint32_t data, uint16_t repeat)
{
    const uint32_t freq = 38000;
    const uint32_t hdrMark = 4500;
    const uint32_t hdrSpace = 4500;
    const uint32_t bitMark = 560;
    const uint32_t oneSpace = 1690;
    const uint32_t zeroSpace = 560;
    const uint32_t gap = 45000;

    for (uint16_t r = 0; r <= repeat; r++) {
        sendPulse(true, hdrMark, freq);
        sendPulse(false, hdrSpace, freq);
        sendBits(data, 32, bitMark, zeroSpace, oneSpace, false, freq);
        sendPulse(true, bitMark, freq);
        sendPulse(false, gap, freq);
    }
}

void IrTransmitter::sendSamsungTv(enum TvCommand cmd, uint16_t repeat)
{
    uint32_t code = 0;
    switch (cmd) {
    case TV_CMD_POWER:
    case TV_CMD_POWER_ON:
    case TV_CMD_POWER_OFF:
        code = 0xE0E040BF;
        break;
    case TV_CMD_VOL_UP:
        code = 0xE0E0E01F;
        break;
    case TV_CMD_VOL_DOWN:
        code = 0xE0E0D02F;
        break;
    case TV_CMD_MUTE:
        code = 0xE0E0F00F;
        break;
    case TV_CMD_CHAN_UP:
        code = 0xE0E048B7;
        break;
    case TV_CMD_CHAN_DOWN:
        code = 0xE0E008F7;
        break;
    case TV_CMD_INPUT:
        code = 0xE0E0807F;
        break;
    default:
        return;
    }

    sendSamsung32(code, repeat);
}

void IrTransmitter::sendNec(uint32_t address, uint32_t command, uint16_t repeat)
{
    const uint32_t freq = 38000;
    const uint32_t hdrMark = 9000;
    const uint32_t hdrSpace = 4500;
    const uint32_t bitMark = 560;
    const uint32_t oneSpace = 1690;
    const uint32_t zeroSpace = 560;
    const uint32_t gap = 40000;

    uint32_t data = (address & 0xFF) |
                    ((~address & 0xFF) << 8) |
                    ((command & 0xFF) << 16) |
                    ((~command & 0xFF) << 24);

    for (uint16_t r = 0; r <= repeat; r++) {
        sendPulse(true, hdrMark, freq);
        sendPulse(false, hdrSpace, freq);
        sendBits(data, 32, bitMark, zeroSpace, oneSpace, false, freq);
        sendPulse(true, bitMark, freq);
        sendPulse(false, gap, freq);
    }
}
