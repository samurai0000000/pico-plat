/*
 * IrTransmitter.hxx
 *
 * Copyright (C) 2026, Charles Chiou
 */

#ifndef IR_TRANSMITTER_HXX
#define IR_TRANSMITTER_HXX

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <IrDecoder.hxx>
#include "hardware/pio.h"

enum TvCommand {
    TV_CMD_POWER = 0,
    TV_CMD_POWER_ON,
    TV_CMD_POWER_OFF,
    TV_CMD_VOL_UP,
    TV_CMD_VOL_DOWN,
    TV_CMD_MUTE,
    TV_CMD_CHAN_UP,
    TV_CMD_CHAN_DOWN,
    TV_CMD_INPUT,
    TV_CMD_DIGIT_0,
    TV_CMD_DIGIT_1,
    TV_CMD_DIGIT_2,
    TV_CMD_DIGIT_3,
    TV_CMD_DIGIT_4,
    TV_CMD_DIGIT_5,
    TV_CMD_DIGIT_6,
    TV_CMD_DIGIT_7,
    TV_CMD_DIGIT_8,
    TV_CMD_DIGIT_9,
};

class IrTransmitter {

public:

    IrTransmitter(uint pin = 17, PIO pio = pio0, int sm = -1);
    ~IrTransmitter();

    bool init(void);
    void deinit(void);

    uint getPin(void) const;
    PIO getPio(void) const;
    int getSm(void) const;
    bool isInitialized(void) const;

    // Send raw mark/space pulse durations (in microseconds)
    void sendRaw(const uint32_t *pulses_us, size_t count, uint32_t freq_hz = 38000);

    // High-level device command transmitters
    void sendPanasonicHvac(const struct IrHvacState &state, uint16_t repeat = 0);
    void sendPanasonicTv(enum TvCommand cmd, uint16_t repeat = 0);
    void sendSonyTv(enum TvCommand cmd, uint16_t repeat = 2);
    void sendSamsungTv(enum TvCommand cmd, uint16_t repeat = 0);

    // Protocol-level transmitters
    void sendNec(uint32_t address, uint32_t command, uint16_t repeat = 0);
    void sendPanasonic48(uint16_t manufacturer, uint8_t device,
                         uint8_t subdevice, uint8_t function, uint16_t repeat = 0);
    void sendSonySirc(uint32_t command, uint32_t address, uint16_t nbits = 12, uint16_t repeat = 2);
    void sendSamsung32(uint32_t data, uint16_t repeat = 0);

private:

    void setFrequency(uint32_t freq_hz);
    void sendPulse(bool isMark, uint32_t duration_us, uint32_t freq_hz);
    void sendBits(uint64_t data, size_t nbits, uint32_t mark_us,
                  uint32_t zero_space_us, uint32_t one_space_us,
                  bool msb_first, uint32_t freq_hz);
    void sendBytes(const uint8_t *data, size_t nbytes, uint32_t mark_us,
                   uint32_t zero_space_us, uint32_t one_space_us,
                   bool msb_first, uint32_t freq_hz);

    uint _pin;
    PIO _pio;
    int _sm;
    uint _offset;
    uint32_t _curFreq;
    bool _initialized;

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
