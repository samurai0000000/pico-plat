/*
 * IrDecoder.hxx
 *
 * Copyright (C) 2025, Charles Chiou
 */

#ifndef IR_DECODER_HXX
#define IR_DECODER_HXX

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <vector>
#include <Imr2638.hxx>

enum IrProtocol {
    IR_PROTO_UNKNOWN = 0,
    IR_PROTO_NEC,
    IR_PROTO_SONY,
    IR_PROTO_RC5,
    IR_PROTO_SAMSUNG,
    IR_PROTO_PANASONIC,
    IR_PROTO_PANASONIC_HVAC,
    IR_PROTO_SHARP,
};

struct IrHvacState {
    bool power;              // true = ON, false = OFF
    uint8_t mode;            // 0=Auto, 1=Cool, 2=Dry, 3=Heat, 4=Fan
    const char *modeName;    // "AUTO", "COOL", "DRY", "HEAT", "FAN"
    float targetTemp;        // Target temperature in °C (16.0 to 30.0)
    uint8_t fanSpeed;        // 0=Auto, 1-5=Speed levels
    const char *fanName;     // "AUTO", "1 (Quiet)", "2", "3", "4", "5 (Max)"
    uint8_t swingV;          // 0=Auto, 1-5=Positions (Up to Down)
    const char *swingVName;  // "AUTO", "UP", "UP-MID", "MID", "DOWN-MID", "DOWN"
    uint8_t swingH;          // 0=Auto, 1-5=Positions
    const char *swingHName;  // "AUTO", "LEFT", "LEFT-MID", "MID", "RIGHT-MID", "RIGHT"
    bool powerful;           // Boost / Turbo mode
    bool quiet;              // Quiet / Silent mode
    bool ionizer;            // Nanoe / Ionizer active
};

struct IrMessage {
    enum IrProtocol protocol;
    const char *protocolName;
    uint32_t address;
    uint32_t command;
    uint64_t rawData;
    uint32_t bitCount;
    bool isRepeat;
    uint32_t repeatCount;
    uint64_t timestamp_us;
    uint8_t payload[32];
    size_t payloadLen;
    struct IrHvacState hvac;
};

class IrDecoder {

public:

    IrDecoder(size_t maxFramePulses = 512);
    ~IrDecoder();

    void reset(void);

    bool addPulse(const struct Imr2638Pulse &pulse, struct IrMessage &msg,
                  enum IrProtocol filter = IR_PROTO_UNKNOWN);
    bool finishFrame(struct IrMessage &msg,
                     enum IrProtocol filter = IR_PROTO_UNKNOWN);

    const std::vector<struct Imr2638Pulse> &getFramePulses(void) const;
    size_t getPulseCount(void) const;

    static bool decodeNec(const struct Imr2638Pulse *pulses, size_t count,
                          struct IrMessage &msg);
    static bool decodeSony(const struct Imr2638Pulse *pulses, size_t count,
                           struct IrMessage &msg);
    static bool decodeRc5(const struct Imr2638Pulse *pulses, size_t count,
                          struct IrMessage &msg);
    static bool decodeSamsung(const struct Imr2638Pulse *pulses, size_t count,
                              struct IrMessage &msg);
    static bool decodePanasonic(const struct Imr2638Pulse *pulses, size_t count,
                                struct IrMessage &msg);
    static bool decodePanasonicHvac(const struct Imr2638Pulse *pulses, size_t count,
                                    struct IrMessage &msg);
    static bool decodeSharp(const struct Imr2638Pulse *pulses, size_t count,
                            struct IrMessage &msg);

    static bool decodeAuto(const struct Imr2638Pulse *pulses, size_t count,
                           struct IrMessage &msg);

    static const char *protocolToString(enum IrProtocol proto);
    static enum IrProtocol stringToProtocol(const char *name);

private:

    bool decodeBuffer(struct IrMessage &msg, enum IrProtocol filter);

    size_t _maxPulses;
    std::vector<struct Imr2638Pulse> _pulses;
    enum IrProtocol _lastProtocol;
    uint32_t _lastAddress;
    uint32_t _lastCommand;
    uint64_t _lastRawData;
    uint32_t _repeatCount;
    uint64_t _lastTimestamp;

};

#endif

/*
 * Local variables:
 * mode: C++
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
