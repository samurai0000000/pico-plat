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
    unsigned int rp;
    unsigned int wp;
    uint32_t marker1;
    char buf[SERIAL_BUF_BUF_SIZE];
    uint32_t marker2;
    char pbuf[SERIAL_PBUF_SIZE];
    uint32_t marker3;
};

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
    unsigned int wp;
    volatile char *dst;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    dst = uart0_buf.buf;
    wp = uart0_buf.wp;
    while (uart_is_readable(uart0)) {
        dst[wp] = (char) uart_get_hw(uart0)->dr;
        wp = ((wp + 1) % SERIAL_BUF_BUF_SIZE);
    }
    uart0_buf.wp = wp;

    xSemaphoreGiveFromISR(uart0_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void serial1_interrupt_handler(void)
{
    unsigned int wp;
    volatile char *dst;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    dst = uart1_buf.buf;
    wp = uart1_buf.wp;
    while (uart_is_readable(uart1)) {
        dst[wp] = (char) uart_get_hw(uart1)->dr;
        wp = ((wp + 1) % SERIAL_BUF_BUF_SIZE);
    }
    uart1_buf.wp = wp;

    xSemaphoreGiveFromISR(uart1_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
        if (!uart_is_writable(uart)) {
            break;
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

    for (int i = 0; (i < ret) && (i < SERIAL_PBUF_SIZE); i++) {
        if (pbuf[i] == '\n') {
            while (serial_write(inst, (const uint8_t *) "\r", 1) != 1);
        }

        while (serial_write(inst, (const uint8_t *) (pbuf + i), 1) != 1);
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

    if (serial_buf->wp < serial_buf->rp) {
        ret = SERIAL_BUF_BUF_SIZE - serial_buf->rp + serial_buf->wp;
    } else {
        ret = serial_buf->wp - serial_buf->rp;
    }

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
        rp = (rp + 1) % size;
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
