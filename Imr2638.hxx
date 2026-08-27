/*
 * Imr2638.hxx
 *
 * Copyright (C) 2025, Charles Chiou
 */

#ifndef IMR2638_HXX
#define IMR2638_HXX

#include <stdint.h>
#include <stdbool.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <hardware/pio.h>

struct Imr2638Pulse {
    bool isMark;              // true = MARK (carrier active), false = SPACE (idle)
    uint32_t duration_us;     // Duration in microseconds
    uint64_t timestamp_us;    // Timestamp when pulse ended
};

class Imr2638 {

public:

    Imr2638(uint32_t pin, PIO pio = pio0, int sm = -1, size_t queueSize = 256);
    ~Imr2638();

    bool start(void);
    bool stop(void);
    bool isRunning(void) const;

    bool getPulse(struct Imr2638Pulse &pulse, TickType_t timeoutTicks = portMAX_DELAY);
    void flush(void);

    uint32_t pin(void) const;
    PIO pio(void) const;
    int sm(void) const;

private:

    static void pio0IrqHandler(void);
    static void pio1IrqHandler(void);
    static void handlePioIrq(PIO pio, uint pioIdx);

    static Imr2638 *s_instances[2][4];

    uint32_t _pin;
    PIO _pio;
    int _sm;
    bool _smClaimed;
    int _progOffset;
    size_t _queueSize;
    QueueHandle_t _queue;
    bool _running;

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
