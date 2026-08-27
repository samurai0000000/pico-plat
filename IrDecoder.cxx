/*
 * IrDecoder.cxx
 *
 * Copyright (C) 2025, Charles Chiou
 */

#include <cstring>
#include <pico/stdlib.h>
#include "IrDecoder.hxx"

static inline bool matchTiming(uint32_t actual, uint32_t expected, uint32_t tolerance = 0)
{
    if (tolerance == 0) {
        tolerance = (expected * 30) / 100;
        if (tolerance < 120) {
            tolerance = 120;
        }
    }

    return (actual >= expected - tolerance) && (actual <= expected + tolerance);
}

static bool decodeHvacBlock(const struct Imr2638Pulse *p, size_t count, size_t numBytes,
                            uint8_t *outBytes, size_t &pulsesConsumed)
{
    size_t needed = 2 + numBytes * 16 + 1;
    if (count < needed) {
        return false;
    }

    // Header: Mark ~3500us, Space ~1750us
    if (!p[0].isMark || !matchTiming(p[0].duration_us, 3500, 800)) {
        return false;
    }
    if (p[1].isMark || !matchTiming(p[1].duration_us, 1750, 500)) {
        return false;
    }

    size_t pIdx = 2;
    for (size_t b = 0; b < numBytes; b++) {
        uint8_t byteVal = 0;
        for (size_t bit = 0; bit < 8; bit++) {
            if (!p[pIdx].isMark || !matchTiming(p[pIdx].duration_us, 435, 250)) {
                return false;
            }
            if (p[pIdx + 1].isMark) {
                return false;
            }
            uint32_t sp = p[pIdx + 1].duration_us;
            if (matchTiming(sp, 435, 250)) {
                // bit 0
            } else if (matchTiming(sp, 1300, 450)) {
                byteVal |= (1 << bit);
            } else {
                return false;
            }
            pIdx += 2;
        }
        outBytes[b] = byteVal;
    }

    // Stop bit: Mark ~435us
    if (!p[pIdx].isMark || !matchTiming(p[pIdx].duration_us, 435, 250)) {
        return false;
    }
    pIdx++;

    pulsesConsumed = pIdx;
    return true;
}

IrDecoder::IrDecoder(size_t maxFramePulses)
    : _maxPulses(maxFramePulses)
    , _lastProtocol(IR_PROTO_UNKNOWN)
    , _lastAddress(0)
    , _lastCommand(0)
    , _lastRawData(0)
    , _repeatCount(0)
    , _lastTimestamp(0)
{
    _pulses.reserve(_maxPulses);
}

IrDecoder::~IrDecoder()
{

}

void IrDecoder::reset(void)
{
    _pulses.clear();
    _lastProtocol = IR_PROTO_UNKNOWN;
    _lastAddress = 0;
    _lastCommand = 0;
    _lastRawData = 0;
    _repeatCount = 0;
    _lastTimestamp = 0;
}

const std::vector<struct Imr2638Pulse> &IrDecoder::getFramePulses(void) const
{
    return _pulses;
}

size_t IrDecoder::getPulseCount(void) const
{
    return _pulses.size();
}

bool IrDecoder::addPulse(const struct Imr2638Pulse &pulse, struct IrMessage &msg,
                         enum IrProtocol filter)
{
    if (_pulses.empty() && !pulse.isMark) {
        // Ignore leading idle space
        return false;
    }

    _pulses.push_back(pulse);

    // If space exceeds 15ms or buffer reached capacity, process frame
    if (!pulse.isMark && (pulse.duration_us >= 15000 || _pulses.size() >= _maxPulses)) {
        bool res = decodeBuffer(msg, filter);
        _pulses.clear();
        return res;
    }

    return false;
}

bool IrDecoder::finishFrame(struct IrMessage &msg, enum IrProtocol filter)
{
    if (_pulses.size() < 3) {
        _pulses.clear();
        return false;
    }

    bool res = decodeBuffer(msg, filter);
    _pulses.clear();
    return res;
}

bool IrDecoder::decodeBuffer(struct IrMessage &msg, enum IrProtocol filter)
{
    size_t count = _pulses.size();
    if (count < 3) {
        return false;
    }

    // Strip trailing long space if present
    if (!_pulses[count - 1].isMark && _pulses[count - 1].duration_us >= 15000) {
        count--;
    }

    const struct Imr2638Pulse *p = _pulses.data();
    bool success = false;

    memset(&msg, 0, sizeof(msg));
    msg.timestamp_us = _pulses[0].timestamp_us;

    switch (filter) {
    case IR_PROTO_NEC:
        success = decodeNec(p, count, msg);
        break;
    case IR_PROTO_SONY:
        success = decodeSony(p, count, msg);
        break;
    case IR_PROTO_RC5:
        success = decodeRc5(p, count, msg);
        break;
    case IR_PROTO_SAMSUNG:
        success = decodeSamsung(p, count, msg);
        break;
    case IR_PROTO_PANASONIC:
        success = decodePanasonic(p, count, msg);
        break;
    case IR_PROTO_PANASONIC_HVAC:
        success = decodePanasonicHvac(p, count, msg);
        break;
    case IR_PROTO_SHARP:
        success = decodeSharp(p, count, msg);
        break;
    case IR_PROTO_UNKNOWN:
    default:
        success = decodeAuto(p, count, msg);
        break;
    }

    if (success) {
        if (msg.isRepeat) {
            _repeatCount++;
            msg.address = _lastAddress;
            msg.command = _lastCommand;
            msg.rawData = _lastRawData;
            msg.repeatCount = _repeatCount;
        } else {
            uint64_t now = msg.timestamp_us;
            if ((msg.protocol == _lastProtocol) &&
                (msg.address == _lastAddress) &&
                (msg.command == _lastCommand) &&
                (now - _lastTimestamp < 250000)) {
                _repeatCount++;
                msg.repeatCount = _repeatCount;
            } else {
                _repeatCount = 0;
                msg.repeatCount = 0;
            }
            _lastProtocol = msg.protocol;
            _lastAddress = msg.address;
            _lastCommand = msg.command;
            _lastRawData = msg.rawData;
            _lastTimestamp = now;
        }
    }

    return success;
}

bool IrDecoder::decodeNec(const struct Imr2638Pulse *pulses, size_t count,
                          struct IrMessage &msg)
{
    if (count < 3) {
        return false;
    }

    // Check NEC header: Mark ~9000us
    if (!pulses[0].isMark || !matchTiming(pulses[0].duration_us, 9000, 1500)) {
        return false;
    }

    // Check repeat header: Space ~2250us
    if (!pulses[1].isMark && matchTiming(pulses[1].duration_us, 2250, 600)) {
        if (count >= 3 && pulses[2].isMark && matchTiming(pulses[2].duration_us, 560, 250)) {
            msg.protocol = IR_PROTO_NEC;
            msg.protocolName = "NEC";
            msg.bitCount = 0;
            msg.isRepeat = true;
            return true;
        }
    }

    // Normal frame: Space ~4500us
    if (pulses[1].isMark || !matchTiming(pulses[1].duration_us, 4500, 1000)) {
        return false;
    }

    if (count < 66) {
        return false;
    }

    uint64_t raw = 0;
    for (size_t i = 0; i < 32; i++) {
        size_t idx = 2 + i * 2;
        if (!pulses[idx].isMark || !matchTiming(pulses[idx].duration_us, 560, 250)) {
            return false;
        }

        if (pulses[idx + 1].isMark) {
            return false;
        }

        uint32_t sp = pulses[idx + 1].duration_us;
        if (matchTiming(sp, 560, 250)) {
            // Bit 0
        } else if (matchTiming(sp, 1690, 450)) {
            raw |= ((uint64_t) 1) << i;
        } else {
            return false;
        }
    }

    uint8_t addr_lo = (uint8_t)(raw & 0xFF);
    uint8_t addr_hi = (uint8_t)((raw >> 8) & 0xFF);
    uint8_t cmd = (uint8_t)((raw >> 16) & 0xFF);
    uint8_t cmd_inv = (uint8_t)((raw >> 24) & 0xFF);

    if ((uint8_t)(cmd ^ cmd_inv) != 0xFF) {
        return false;
    }

    msg.protocol = IR_PROTO_NEC;
    msg.protocolName = "NEC";
    msg.rawData = raw;
    msg.bitCount = 32;
    msg.isRepeat = false;

    if ((uint8_t)(addr_lo ^ addr_hi) == 0xFF) {
        msg.address = addr_lo;
    } else {
        msg.address = (addr_hi << 8) | addr_lo;
    }
    msg.command = cmd;

    return true;
}

bool IrDecoder::decodeSony(const struct Imr2638Pulse *pulses, size_t count,
                           struct IrMessage &msg)
{
    if (count < 25) {
        return false;
    }

    // Header: Mark ~2400us, Space ~600us
    if (!pulses[0].isMark || !matchTiming(pulses[0].duration_us, 2400, 500)) {
        return false;
    }
    if (pulses[1].isMark || !matchTiming(pulses[1].duration_us, 600, 250)) {
        return false;
    }

    uint64_t raw = 0;
    uint32_t bitCount = 0;

    for (size_t i = 0; i < 20; i++) {
        size_t mIdx = 2 + i * 2;
        if (mIdx >= count || !pulses[mIdx].isMark) {
            break;
        }

        uint32_t mk = pulses[mIdx].duration_us;
        if (matchTiming(mk, 600, 250)) {
            // Bit 0
        } else if (matchTiming(mk, 1200, 350)) {
            raw |= ((uint64_t) 1) << i;
        } else {
            break;
        }
        bitCount++;

        if (mIdx + 1 < count) {
            if (pulses[mIdx + 1].isMark ||
                !matchTiming(pulses[mIdx + 1].duration_us, 600, 250)) {
                break;
            }
        }
    }

    if (bitCount != 12 && bitCount != 15 && bitCount != 20) {
        return false;
    }

    msg.protocol = IR_PROTO_SONY;
    msg.protocolName = "SONY";
    msg.rawData = raw;
    msg.bitCount = bitCount;
    msg.isRepeat = false;
    msg.command = (uint32_t)(raw & 0x7F);

    if (bitCount == 12) {
        msg.address = (uint32_t)((raw >> 7) & 0x1F);
    } else if (bitCount == 15) {
        msg.address = (uint32_t)((raw >> 7) & 0xFF);
    } else {
        msg.address = (uint32_t)((raw >> 7) & 0x1FFF);
    }

    return true;
}

bool IrDecoder::decodeRc5(const struct Imr2638Pulse *pulses, size_t count,
                          struct IrMessage &msg)
{
    if (count < 14) {
        return false;
    }

    if (!pulses[0].isMark) {
        return false;
    }

    uint8_t halfBits[28];
    size_t hbIdx = 0;

    for (size_t i = 0; i < count && hbIdx < 28; i++) {
        uint32_t d = pulses[i].duration_us;
        uint8_t level = pulses[i].isMark ? 1 : 0;

        if (matchTiming(d, 889, 300)) {
            halfBits[hbIdx++] = level;
        } else if (matchTiming(d, 1778, 450)) {
            if (hbIdx + 1 >= 28) {
                break;
            }
            halfBits[hbIdx++] = level;
            halfBits[hbIdx++] = level;
        } else {
            return false;
        }
    }

    if (hbIdx < 28) {
        return false;
    }

    uint32_t raw = 0;
    for (size_t b = 0; b < 14; b++) {
        uint8_t h0 = halfBits[b * 2];
        uint8_t h1 = halfBits[b * 2 + 1];

        if (h0 == 0 && h1 == 1) {
            raw = (raw << 1) | 1;
        } else if (h0 == 1 && h1 == 0) {
            raw = (raw << 1) | 0;
        } else {
            return false;
        }
    }

    if (!(raw & (1 << 13))) {
        return false;
    }

    msg.protocol = IR_PROTO_RC5;
    msg.protocolName = "RC5";
    msg.rawData = raw;
    msg.bitCount = 14;
    msg.isRepeat = false;
    msg.address = (raw >> 6) & 0x1F;
    msg.command = raw & 0x3F;
    if (!(raw & (1 << 12))) {
        msg.command |= 0x40;
    }

    return true;
}

bool IrDecoder::decodeSamsung(const struct Imr2638Pulse *pulses, size_t count,
                              struct IrMessage &msg)
{
    if (count < 66) {
        return false;
    }

    // Header: Mark ~4500us, Space ~4500us
    if (!pulses[0].isMark || !matchTiming(pulses[0].duration_us, 4500, 1000)) {
        return false;
    }
    if (pulses[1].isMark || !matchTiming(pulses[1].duration_us, 4500, 1000)) {
        return false;
    }

    uint64_t raw = 0;
    for (size_t i = 0; i < 32; i++) {
        size_t idx = 2 + i * 2;
        if (!pulses[idx].isMark || !matchTiming(pulses[idx].duration_us, 560, 250)) {
            return false;
        }

        if (pulses[idx + 1].isMark) {
            return false;
        }

        uint32_t sp = pulses[idx + 1].duration_us;
        if (matchTiming(sp, 560, 250)) {
            // Bit 0
        } else if (matchTiming(sp, 1690, 450)) {
            raw |= ((uint64_t) 1) << i;
        } else {
            return false;
        }
    }

    uint8_t cust_lo = (uint8_t)(raw & 0xFF);
    uint8_t cust_hi = (uint8_t)((raw >> 8) & 0xFF);
    uint8_t cmd = (uint8_t)((raw >> 16) & 0xFF);
    uint8_t cmd_inv = (uint8_t)((raw >> 24) & 0xFF);

    if ((uint8_t)(cmd ^ cmd_inv) != 0xFF) {
        return false;
    }

    msg.protocol = IR_PROTO_SAMSUNG;
    msg.protocolName = "SAMSUNG";
    msg.rawData = raw;
    msg.bitCount = 32;
    msg.isRepeat = false;
    msg.address = (cust_hi << 8) | cust_lo;
    msg.command = cmd;

    return true;
}

bool IrDecoder::decodePanasonic(const struct Imr2638Pulse *pulses, size_t count,
                                struct IrMessage &msg)
{
    if (count < 98) {
        return false;
    }

    // Header: Mark ~3500us, Space ~1750us
    if (!pulses[0].isMark || !matchTiming(pulses[0].duration_us, 3500, 700)) {
        return false;
    }
    if (pulses[1].isMark || !matchTiming(pulses[1].duration_us, 1750, 400)) {
        return false;
    }

    uint64_t raw = 0;
    for (size_t i = 0; i < 48; i++) {
        size_t idx = 2 + i * 2;
        if (!pulses[idx].isMark || !matchTiming(pulses[idx].duration_us, 430, 200)) {
            return false;
        }

        if (pulses[idx + 1].isMark) {
            return false;
        }

        uint32_t sp = pulses[idx + 1].duration_us;
        if (matchTiming(sp, 430, 200)) {
            // Bit 0
        } else if (matchTiming(sp, 1300, 350)) {
            raw |= ((uint64_t) 1) << i;
        } else {
            return false;
        }
    }

    uint8_t b0 = (uint8_t)(raw & 0xFF);
    uint8_t b1 = (uint8_t)((raw >> 8) & 0xFF);
    uint8_t b2 = (uint8_t)((raw >> 16) & 0xFF);
    uint8_t b3 = (uint8_t)((raw >> 24) & 0xFF);
    uint8_t b4 = (uint8_t)((raw >> 32) & 0xFF);
    uint8_t b5 = (uint8_t)((raw >> 40) & 0xFF);

    if (b5 != (uint8_t)(b2 ^ b3 ^ b4)) {
        return false;
    }

    msg.protocol = IR_PROTO_PANASONIC;
    msg.protocolName = "PANASONIC";
    msg.rawData = raw;
    msg.bitCount = 48;
    msg.isRepeat = false;
    msg.address = (b1 << 8) | b0;
    msg.command = (b3 << 8) | b4;

    return true;
}

bool IrDecoder::decodePanasonicHvac(const struct Imr2638Pulse *pulses, size_t count,
                                    struct IrMessage &msg)
{
    if (count < 438) {
        return false;
    }

    uint8_t bytes[27];
    size_t c1 = 0;

    // Decode Block 1 (8 bytes)
    if (!decodeHvacBlock(pulses, count, 8, bytes, c1)) {
        return false;
    }

    // Check Inter-Block Space (~10ms gap)
    if (c1 >= count || pulses[c1].isMark || !matchTiming(pulses[c1].duration_us, 10000, 4000)) {
        return false;
    }
    c1++;

    // Decode Block 2 (19 bytes)
    size_t c2 = 0;
    if (!decodeHvacBlock(pulses + c1, count - c1, 19, bytes + 8, c2)) {
        return false;
    }

    // Verify Checksum: sum of bytes 8..25 modulo 256
    uint8_t checksum = 0;
    for (size_t i = 8; i < 26; i++) {
        checksum += bytes[i];
    }
    if (checksum != bytes[26]) {
        return false;
    }

    msg.protocol = IR_PROTO_PANASONIC_HVAC;
    msg.protocolName = "PANASONIC_HVAC";
    msg.bitCount = 216;
    msg.payloadLen = 27;
    memcpy(msg.payload, bytes, 27);
    msg.isRepeat = false;
    msg.address = (bytes[1] << 8) | bytes[0];
    msg.command = (bytes[14] << 8) | bytes[13];

    // Decode HVAC fields
    // Byte 13: Power (bit 0) & Mode (bits 4..7)
    msg.hvac.power = (bytes[13] & 0x01) != 0;
    uint8_t modeBits = (bytes[13] >> 4) & 0x07;
    msg.hvac.mode = modeBits;
    switch (modeBits) {
    case 0:
        msg.hvac.modeName = "AUTO";
        break;
    case 2:
        msg.hvac.modeName = "COOL";
        break;
    case 3:
        msg.hvac.modeName = "DRY";
        break;
    case 4:
        msg.hvac.modeName = "HEAT";
        break;
    case 6:
        msg.hvac.modeName = "FAN";
        break;
    default:
        msg.hvac.modeName = "UNKNOWN";
        break;
    }

    // Byte 14: Target Temperature (bits 1..5)
    uint8_t tempVal = (bytes[14] >> 1) & 0x1F;
    msg.hvac.targetTemp = (float) tempVal;

    // Byte 16: Fan Speed (bits 4..7) & Vertical Swing (bits 0..3)
    uint8_t fanBits = (bytes[16] >> 4) & 0x0F;
    msg.hvac.fanSpeed = fanBits;
    switch (fanBits) {
    case 0xA:
        msg.hvac.fanName = "AUTO";
        break;
    case 0x3:
        msg.hvac.fanName = "1 (Quiet)";
        break;
    case 0x4:
        msg.hvac.fanName = "2";
        break;
    case 0x5:
        msg.hvac.fanName = "3";
        break;
    case 0x6:
        msg.hvac.fanName = "4";
        break;
    case 0x7:
        msg.hvac.fanName = "5 (Max)";
        break;
    default:
        msg.hvac.fanName = "MANUAL";
        break;
    }

    uint8_t swingVBits = bytes[16] & 0x0F;
    msg.hvac.swingV = swingVBits;
    switch (swingVBits) {
    case 0xF:
        msg.hvac.swingVName = "AUTO";
        break;
    case 0x1:
        msg.hvac.swingVName = "UP";
        break;
    case 0x2:
        msg.hvac.swingVName = "UP-MID";
        break;
    case 0x3:
        msg.hvac.swingVName = "MID";
        break;
    case 0x4:
        msg.hvac.swingVName = "DOWN-MID";
        break;
    case 0x5:
        msg.hvac.swingVName = "DOWN";
        break;
    default:
        msg.hvac.swingVName = "FIXED";
        break;
    }

    // Byte 17: Horizontal Swing (bits 0..3)
    uint8_t swingHBits = bytes[17] & 0x0F;
    msg.hvac.swingH = swingHBits;
    switch (swingHBits) {
    case 0xF:
    case 0x0:
        msg.hvac.swingHName = "AUTO";
        break;
    case 0x1:
        msg.hvac.swingHName = "LEFT";
        break;
    case 0x2:
        msg.hvac.swingHName = "LEFT-MID";
        break;
    case 0x3:
        msg.hvac.swingHName = "MID";
        break;
    case 0x4:
        msg.hvac.swingHName = "RIGHT-MID";
        break;
    case 0x5:
        msg.hvac.swingHName = "RIGHT";
        break;
    default:
        msg.hvac.swingHName = "FIXED";
        break;
    }

    // Byte 21: Powerful (bit 0) & Quiet (bit 1)
    msg.hvac.powerful = (bytes[21] & 0x01) != 0;
    msg.hvac.quiet = (bytes[21] & 0x02) != 0;

    // Byte 22: Ionizer / Nanoe (bit 0)
    msg.hvac.ionizer = (bytes[22] & 0x01) != 0;

    return true;
}

bool IrDecoder::decodeSharp(const struct Imr2638Pulse *pulses, size_t count,
                            struct IrMessage &msg)
{
    if (count < 30) {
        return false;
    }

    uint64_t raw = 0;
    for (size_t i = 0; i < 15; i++) {
        size_t idx = i * 2;
        if (!pulses[idx].isMark || !matchTiming(pulses[idx].duration_us, 320, 150)) {
            return false;
        }

        if (pulses[idx + 1].isMark) {
            return false;
        }

        uint32_t sp = pulses[idx + 1].duration_us;
        if (matchTiming(sp, 680, 250)) {
            // Bit 0
        } else if (matchTiming(sp, 1680, 450)) {
            raw |= ((uint64_t) 1) << i;
        } else {
            return false;
        }
    }

    msg.protocol = IR_PROTO_SHARP;
    msg.protocolName = "SHARP";
    msg.rawData = raw;
    msg.bitCount = 15;
    msg.isRepeat = false;
    msg.address = (uint32_t)(raw & 0x1F);
    msg.command = (uint32_t)((raw >> 5) & 0xFF);

    return true;
}

bool IrDecoder::decodeAuto(const struct Imr2638Pulse *pulses, size_t count,
                           struct IrMessage &msg)
{
    if (decodePanasonicHvac(pulses, count, msg)) {
        return true;
    }
    if (decodeNec(pulses, count, msg)) {
        return true;
    }
    if (decodeSony(pulses, count, msg)) {
        return true;
    }
    if (decodeSamsung(pulses, count, msg)) {
        return true;
    }
    if (decodeRc5(pulses, count, msg)) {
        return true;
    }
    if (decodePanasonic(pulses, count, msg)) {
        return true;
    }
    if (decodeSharp(pulses, count, msg)) {
        return true;
    }

    return false;
}

const char *IrDecoder::protocolToString(enum IrProtocol proto)
{
    switch (proto) {
    case IR_PROTO_NEC:
        return "NEC";
    case IR_PROTO_SONY:
        return "SONY";
    case IR_PROTO_RC5:
        return "RC5";
    case IR_PROTO_SAMSUNG:
        return "SAMSUNG";
    case IR_PROTO_PANASONIC:
        return "PANASONIC";
    case IR_PROTO_PANASONIC_HVAC:
        return "PANASONIC_HVAC";
    case IR_PROTO_SHARP:
        return "SHARP";
    case IR_PROTO_UNKNOWN:
    default:
        return "UNKNOWN";
    }
}

enum IrProtocol IrDecoder::stringToProtocol(const char *name)
{
    if (name == NULL) {
        return IR_PROTO_UNKNOWN;
    }

    if (strcasecmp(name, "nec") == 0) {
        return IR_PROTO_NEC;
    } else if (strcasecmp(name, "sony") == 0) {
        return IR_PROTO_SONY;
    } else if (strcasecmp(name, "rc5") == 0) {
        return IR_PROTO_RC5;
    } else if (strcasecmp(name, "samsung") == 0) {
        return IR_PROTO_SAMSUNG;
    } else if (strcasecmp(name, "panasonic") == 0) {
        return IR_PROTO_PANASONIC;
    } else if (strcasecmp(name, "panasonic_hvac") == 0 ||
               strcasecmp(name, "panasonic-hvac") == 0 ||
               strcasecmp(name, "hvac") == 0) {
        return IR_PROTO_PANASONIC_HVAC;
    } else if (strcasecmp(name, "sharp") == 0) {
        return IR_PROTO_SHARP;
    }

    return IR_PROTO_UNKNOWN;
}

/*
 * Local variables:
 * mode: C++
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
