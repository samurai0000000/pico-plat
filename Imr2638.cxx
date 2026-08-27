/*
 * Imr2638.cxx
 *
 * Copyright (C) 2025, Charles Chiou
 */

#include <pico/stdlib.h>
#include <hardware/clocks.h>
#include <hardware/irq.h>
#include "imr2638.pio.h"
#include "Imr2638.hxx"

Imr2638 *Imr2638::s_instances[2][4] = {
    { NULL, NULL, NULL, NULL },
    { NULL, NULL, NULL, NULL }
};

Imr2638::Imr2638(uint32_t pin, PIO pio, int sm, size_t queueSize)
    : _pin(pin)
    , _pio(pio)
    , _sm(sm)
    , _smClaimed(false)
    , _progOffset(-1)
    , _queueSize(queueSize)
    , _queue(NULL)
    , _running(false)
{

}

Imr2638::~Imr2638()
{
    stop();

    if (_queue != NULL) {
        vQueueDelete(_queue);
        _queue = NULL;
    }
}

bool Imr2638::start(void)
{
    if (_running) {
        return true;
    }

    if (_queue == NULL) {
        _queue = xQueueCreate(_queueSize, sizeof(struct Imr2638Pulse));
        if (_queue == NULL) {
            return false;
        }
    } else {
        xQueueReset(_queue);
    }

    if (_progOffset < 0) {
        if (!pio_can_add_program(_pio, &imr2638_rx_program)) {
            return false;
        }
        _progOffset = pio_add_program(_pio, &imr2638_rx_program);
    }

    if (_sm < 0) {
        int claimed = pio_claim_unused_sm(_pio, false);
        if (claimed < 0) {
            pio_remove_program(_pio, &imr2638_rx_program, _progOffset);
            _progOffset = -1;
            return false;
        }
        _sm = claimed;
        _smClaimed = true;
    } else {
        pio_sm_claim(_pio, (uint) _sm);
        _smClaimed = true;
    }

    uint pioIdx = pio_get_index(_pio);
    s_instances[pioIdx][_sm] = this;

    imr2638_rx_program_init(_pio, (uint) _sm, (uint) _progOffset, _pin);

    uint irqNum = (pioIdx == 0) ? PIO0_IRQ_0 : PIO1_IRQ_0;
    irq_handler_t handler = (pioIdx == 0) ? pio0IrqHandler : pio1IrqHandler;

    irq_set_exclusive_handler(irqNum, handler);
    irq_set_enabled(irqNum, true);

    pio_set_irqn_source_enabled(_pio, 0,
        (enum pio_interrupt_source)(pis_sm0_rx_fifo_not_empty + _sm), true);

    pio_sm_clear_fifos(_pio, (uint) _sm);
    pio_sm_set_enabled(_pio, (uint) _sm, true);
    _running = true;

    return true;
}

bool Imr2638::stop(void)
{
    if (!_running) {
        return true;
    }

    uint pioIdx = pio_get_index(_pio);

    pio_sm_set_enabled(_pio, (uint) _sm, false);
    pio_set_irqn_source_enabled(_pio, 0,
        (enum pio_interrupt_source)(pis_sm0_rx_fifo_not_empty + _sm), false);

    s_instances[pioIdx][_sm] = NULL;

    bool anyPioActive = false;
    for (int sm = 0; sm < 4; sm++) {
        if (s_instances[pioIdx][sm] != NULL) {
            anyPioActive = true;
            break;
        }
    }
    if (!anyPioActive) {
        uint irqNum = (pioIdx == 0) ? PIO0_IRQ_0 : PIO1_IRQ_0;
        irq_set_enabled(irqNum, false);
    }

    pio_sm_clear_fifos(_pio, (uint) _sm);

    if (_smClaimed) {
        pio_sm_unclaim(_pio, (uint) _sm);
        _smClaimed = false;
    }

    if (_progOffset >= 0) {
        pio_remove_program(_pio, &imr2638_rx_program, (uint) _progOffset);
        _progOffset = -1;
    }

    _running = false;

    return true;
}

bool Imr2638::isRunning(void) const
{
    return _running;
}

bool Imr2638::getPulse(struct Imr2638Pulse &pulse, TickType_t timeoutTicks)
{
    if (_queue == NULL) {
        return false;
    }

    return (xQueueReceive(_queue, &pulse, timeoutTicks) == pdTRUE);
}

void Imr2638::flush(void)
{
    if (_queue != NULL) {
        xQueueReset(_queue);
    }
}

uint32_t Imr2638::pin(void) const
{
    return _pin;
}

PIO Imr2638::pio(void) const
{
    return _pio;
}

int Imr2638::sm(void) const
{
    return _sm;
}

void Imr2638::pio0IrqHandler(void)
{
    handlePioIrq(pio0, 0);
}

void Imr2638::pio1IrqHandler(void)
{
    handlePioIrq(pio1, 1);
}

void Imr2638::handlePioIrq(PIO pio, uint pioIdx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    for (int sm = 0; sm < 4; sm++) {
        Imr2638 *inst = s_instances[pioIdx][sm];
        if ((inst != NULL) && inst->_running && (inst->_queue != NULL)) {
            while (!pio_sm_is_rx_fifo_empty(pio, (uint) sm)) {
                uint32_t val = pio_sm_get(pio, (uint) sm);
                struct Imr2638Pulse pulse;

                pulse.isMark = (val & 0x1) != 0;
                pulse.duration_us = (val >> 1);
                pulse.timestamp_us = time_us_64();

                xQueueSendFromISR(inst->_queue, &pulse, &xHigherPriorityTaskWoken);
            }
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
