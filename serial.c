/*
 * serial.c
 *
 * Copyright (C) 2025, Charles Chiou
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>
#include <strings.h>
#include <unistd.h>
#include <pico/stdio.h>
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <hardware/sync.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <pico-plat.h>

#ifndef UART0_TX_PIN
#define UART0_TX_PIN      0
#endif
#ifndef UART0_RX_PIN
#define UART0_RX_PIN      1
#endif
#ifndef UART0_BAUD_RATE
#define UART0_BAUD_RATE   115200
#endif

#ifndef UART1_TX_PIN
#define UART1_TX_PIN      4
#endif
#ifndef UART1_RX_PIN
#define UART1_RX_PIN      5
#endif
#ifndef UART1_BAUD_RATE
#define UART1_BAUD_RATE   115200
#endif

#define UART_DATA_BITS    8
#define UART_STOP_BITS    1
#define UART_PARITY       UART_PARITY_NONE

#define SERIAL_BUF_BUF_SIZE  512
#define SERIAL_PBUF_SIZE  512

struct serial_buf {
    volatile unsigned int rp;
    volatile unsigned int wp;
    uint32_t marker1;
    char buf[SERIAL_BUF_BUF_SIZE];
    uint32_t marker2;
    char pbuf[SERIAL_PBUF_SIZE];
    uint32_t marker3;
};

/* One slot is left empty so wp == rp always means empty, never full. */
static unsigned int serial_ring_used(const struct serial_buf *sb)
{
    unsigned int rp = sb->rp;
    unsigned int wp = sb->wp;

    if (wp < rp) {
        return SERIAL_BUF_BUF_SIZE - rp + wp;
    }

    return wp - rp;
}

static void serial_irq_read(uart_inst_t *uart, struct serial_buf *sb,
                            SemaphoreHandle_t sem)
{
    unsigned int wp = sb->wp;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    while (uart_is_readable(uart)) {
        char c = (char) uart_getc(uart);
        unsigned int next = (wp + 1u) % SERIAL_BUF_BUF_SIZE;

        /* Drain the UART FIFO even when the software buffer is full so the
         * IRQ does not stick asserted. Drop the new byte rather than
         * overwriting unread data (which would look like an empty buffer). */
        if (next != sb->rp) {
            sb->buf[wp] = c;
            wp = next;
        }
    }
    sb->wp = wp;

    xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static struct serial_buf uart0_buf = {
    .rp = 0,
    .wp = 0,
    .marker1 = 0x12345678,
    .buf = { 0, },
    .marker2 = 0x12345678,
    .pbuf = { 0, },
    .marker3 = 0x12345678,
};

static struct serial_buf uart1_buf = {
    .rp = 0,
    .wp = 0,
    .marker1 = 0x12345678,
    .buf = { 0, },
    .marker2 = 0x12345678,
    .pbuf = { 0, },
    .marker3 = 0x12345678,
};

SemaphoreHandle_t uart0_sem = NULL;
SemaphoreHandle_t uart1_sem = NULL;

static void serial0_interrupt_handler(void)
{
    serial_irq_read(uart0, &uart0_buf, uart0_sem);
}

static void serial1_interrupt_handler(void)
{
    serial_irq_read(uart1, &uart1_buf, uart1_sem);
}

void serial_init(void)
{
    uart0_sem = xSemaphoreCreateBinary();
    assert(uart0_sem != NULL);
    uart1_sem = xSemaphoreCreateBinary();
    assert(uart1_sem != NULL);

    uart_init(uart0, UART0_BAUD_RATE);
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(uart0, false, false);
    uart_set_fifo_enabled(uart0, true);
    uart_set_format(uart0, UART_DATA_BITS, UART_STOP_BITS, UART_PARITY);
    irq_set_exclusive_handler(UART0_IRQ, serial0_interrupt_handler);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);

    uart_init(uart1, UART1_BAUD_RATE);
    gpio_set_function(UART1_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART1_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(uart1, false, false);
    uart_set_fifo_enabled(uart1, true);
    uart_set_format(uart1, UART_DATA_BITS, UART_STOP_BITS, UART_PARITY);
    irq_set_exclusive_handler(UART1_IRQ, serial1_interrupt_handler);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(uart1, true, false);
}

void serial_deinit(void)
{
    vSemaphoreDelete(uart0_sem);
    uart0_sem = NULL;
    vSemaphoreDelete(uart1_sem);
    uart1_sem = NULL;
}

int serial_check_markers(unsigned int inst)
{
    int ret = 0;
    struct serial_buf *serial_buf = NULL;

    switch (inst) {
    case 0:  serial_buf = &uart0_buf; break;
    case 1:  serial_buf = &uart1_buf; break;
    default: ret = -1; goto done; break;
    }

    if (serial_buf->marker1 != 0x12345678) {
        ret = 1;
        goto done;
    } else if (serial_buf->marker2 != 0x12345678) {
        ret = 2;
        goto done;
    } else if (serial_buf->marker3 != 0x12345678) {
        ret = 4;
        goto done;
    }

done:

    return ret;
}

int serial_write(unsigned int inst, const uint8_t *data, size_t len)
{
    int ret = 0;
    uart_inst_t *uart = NULL;

    switch (inst) {
    case 0:  uart = uart0; break;
    case 1:  uart = uart1; break;
    default: ret = -1; goto done; break;
    }

    for (size_t i = 0; i < len; i++) {
        while (!uart_is_writable(uart)) {
            /* Hardware drains the TX FIFO; yield so other tasks can run. */
            if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
                taskYIELD();
            }
        }

        uart_get_hw(uart)->dr = data[i];
        ret++;
    }

done:

    return ret;
}

int serial_printf(unsigned int inst, const char *format, ...)
{
    int ret;
    va_list ap;

    va_start(ap, format);
    ret = serial_vprintf(inst, format, ap);
    va_end(ap);

    return ret;
}

int serial_vprintf(unsigned int inst, const char *format, va_list ap)
{
    int ret = 0;
    char *pbuf = NULL;

    switch (inst) {
    case 0:  pbuf = uart0_buf.pbuf; break;
    case 1:  pbuf = uart1_buf.pbuf; break;
    default: ret = -1; goto done; break;
    }

    ret = vsnprintf(pbuf, SERIAL_PBUF_SIZE - 1, format, ap);
    if (ret < 0) {
        ret = -1;
        goto done;
    }
    if (ret > (int) (SERIAL_PBUF_SIZE - 1)) {
        ret = SERIAL_PBUF_SIZE - 1;
    }

    for (int i = 0; i < ret; ) {
        int n;

        if (pbuf[i] == '\n') {
            n = serial_write(inst, (const uint8_t *) "\r\n", 2);
            if (n != 2) {
                ret = i;
                goto done;
            }
            i++;
            continue;
        }

        {
            int run = i + 1;
            while ((run < ret) && (pbuf[run] != '\n')) {
                run++;
            }
            n = serial_write(inst, (const uint8_t *) (pbuf + i),
                             (size_t) (run - i));
            if (n != (run - i)) {
                ret = i + ((n > 0) ? n : 0);
                goto done;
            }
            i = run;
        }
    }

done:

    return ret;
}

int serial_rx_ready(unsigned int inst)
{
    int ret = 0;
    struct serial_buf *serial_buf = NULL;

    switch (inst) {
    case 0:  serial_buf = &uart0_buf; break;
    case 1:  serial_buf = &uart1_buf; break;
    default: ret = -1; goto done; break;
    }

    ret = (int) serial_ring_used(serial_buf);

done:

    return ret;
}

int serial_read(unsigned int inst, uint8_t *data, size_t len)
{
    int ret = 0;
    struct serial_buf *serial_buf = NULL;
    const uint8_t *src;
    unsigned int rp, wp;
    size_t size;

    switch (inst) {
    case 0:  serial_buf = &uart0_buf; break;
    case 1:  serial_buf = &uart1_buf; break;
    default: ret = -1; goto done; break;
    }

    src = (const uint8_t *) serial_buf->buf;
    size = SERIAL_BUF_BUF_SIZE;
    rp = serial_buf->rp;
    wp = serial_buf->wp;
    while ((len > 0) && (rp != wp)) {
        *data = src[rp];
        data++;
        len--;
        ret++;
        rp = (rp + 1u) % size;
    }

    serial_buf->rp = rp;

done:

    return ret;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
