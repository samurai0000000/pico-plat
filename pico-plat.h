/*
 * pico-plat.h
 *
 * Copyright (C) 2025, Charles Chiou
 */

#ifndef PICO_PLAT_H
#define PICO_PLAT_H

#include <stdarg.h>

#if !defined(EXTERN_C_BEGIN)
#if defined(__cplusplus)
#define EXTERN_C_BEGIN extern "C" {
#else
#define EXTERN_C_BEGIN
#endif
#endif

#if !defined(EXTERN_C_END)
#if defined(__cplusplus)
#define EXTERN_C_END }
#else
#define EXTERN_C_END
#endif
#endif

EXTERN_C_BEGIN

extern void serial_init(void);
extern void serial_deinit(void);

extern int serial_check_markers(unsigned int inst);
extern int serial_write(unsigned int inst, const uint8_t *data, size_t size);
extern int serial_printf(unsigned int inst, const char *format, ...);
extern int serial_vprintf(unsigned int inst, const char *format, va_list ap);
extern int serial_rx_ready(unsigned int inst);
extern int serial_read(unsigned int inst, uint8_t *data, size_t size);

static inline int serial0_check_markers(void)
{
    return serial_check_markers(0);
}

static inline int serial0_write(const uint8_t *data, size_t size)
{
    return serial_write(0, data, size);
}

static inline int serial0_printf(const char *format, ...)
{
    int ret = 0;
    va_list ap;

    va_start(ap, format);
    serial_vprintf(0, format, ap);
    va_end(ap);

    return ret;
}

static inline int serial0_vprintf(const char *format, va_list ap)
{
    return serial_vprintf(0, format, ap);
}

static inline int serial0_rx_ready(void)
{
    return serial_rx_ready(0);
}

static inline int serial0_read(uint8_t *data, size_t size)
{
    return serial_read(0, data, size);
}

static inline int serial1_check_markers(void)
{
    return serial_check_markers(1);
}

static inline int serial1_write(const uint8_t *data, size_t size)
{
    return serial_write(1, data, size);
}

static inline int serial1_printf(const char *format, ...)
{
    int ret = 0;
    va_list ap;

    va_start(ap, format);
    serial_vprintf(1, format, ap);
    va_end(ap);

    return ret;
}

static inline int serial1_vprintf(const char *format, va_list ap)
{
    return serial_vprintf(1, format, ap);
}

static inline int serial1_rx_ready(void)
{
    return serial_rx_ready(1);
}

static inline int serial1_read(uint8_t *data, size_t size)
{
    return serial_read(1, data, size);
}

#if defined(SEMAPHORE_H)
extern SemaphoreHandle_t uart0_sem;
extern SemaphoreHandle_t uart1_sem;
#endif

#if !defined(LIB_PICO_STDIO_USB)

extern void usbcdc_init(void);
extern void usbcdc_deinit(void);
extern void usbcdc_task(void);
extern int usbcdc_is_connected(void);
extern int usbcdc_write(const void *buf, size_t len);
extern int usbcdc_printf(const char *format, ...);
extern int usbcdc_vprintf(const char *format, va_list ap);
extern int usbcdc_rx_ready(void);
extern int usbcdc_read(void *buf, size_t len);

#if defined(SEMAPHORE_H)
extern SemaphoreHandle_t cdc_sem;
#endif

#endif  // !LIBPICO_STDIO_USB

EXTERN_C_END

#endif  // PICO_PLAT_H

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
