/*
 * usbcdc.c
 *
 * Copyright (C) 2025, Charles Chiou
 */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <tusb.h>
#include <bsp/board_api.h>
#include <pico/stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <pico-plat.h>

#if !defined(LIB_PICO_STDIO_USB)

#define LIBPICO_CDC_VID     0xcafe
#define LIBPICO_CDC_PID     0x4000
#define LIBPICO_CDC_BCD     0x0200
#define LIBPICO_CDC_REL     0x0100

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + \
                             (CFG_TUD_CDC * TUD_CDC_DESC_LEN))

#define EPNUM_CDC_0_NOTIF   0x81
#define EPNUM_CDC_0_OUT     0x02
#define EPNUM_CDC_0_IN      0x82

#if !defined(SERIAL_PBUF_SIZE)
#define SERIAL_PBUF_SIZE  512
#endif

#define SERIAL_BUF_BUF_SIZE  512

enum {
    ITF_NUM_CDC_0 = 0,
    ITF_NUM_CDC_0_DATA,
    ITF_NUM_TOTAL
};

enum {
    STRID_LANGID = 0,   // 0: supported language ID
    STRID_MANUFACTURER, // 1: Manufacturer
    STRID_PRODUCT,      // 2: Product
    STRID_SERIAL,       // 3: Serials
    STRID_CDC_0,        // 4: CDC Interface 0
};

tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = LIBPICO_CDC_BCD,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = LIBPICO_CDC_VID,
    .idProduct = LIBPICO_CDC_PID,
    .bcdDevice = LIBPICO_CDC_REL,
    .iManufacturer = STRID_MANUFACTURER,
    .iProduct = STRID_PRODUCT,
    .iSerialNumber = STRID_SERIAL,
    .bNumConfigurations = 0x01,
};

static const char *string_desc_arr[] = {
    (const char []) { 0x09, 0x04, },  // 0: English (0x0409)
    "LibPico",                        // 1: Manufacturer
    "Generic",                        // 2: Product
    NULL,                             // 3: Serials
    "CDC",                            // 4: CDC Interface 0
    "LibPicoCDCReset"                 // 5: Reset Interface
};

static const uint8_t desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4,
                       EPNUM_CDC_0_NOTIF, 8,
                       EPNUM_CDC_0_OUT,
                       EPNUM_CDC_0_IN, 64),
};

static tusb_desc_device_qualifier_t const desc_device_qualifier = {
    .bLength = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB = LIBPICO_CDC_BCD,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 0x01,
    .bReserved = 0x00
};

struct serial_buf {
    volatile unsigned int rp;
    volatile unsigned int wp;
    char buf[SERIAL_BUF_BUF_SIZE];
};

static struct serial_buf cdc_rx_buf = {
    .rp = 0,
    .wp = 0,
    .buf = { 0, },
};

static char pbuf[SERIAL_PBUF_SIZE];

SemaphoreHandle_t cdc_sem = NULL;
static SemaphoreHandle_t cdc_mutex = NULL;

/* One slot is left empty so wp == rp always means empty, never full. */
static unsigned int cdc_ring_used(void)
{
    unsigned int rp = cdc_rx_buf.rp;
    unsigned int wp = cdc_rx_buf.wp;

    if (wp < rp) {
        return SERIAL_BUF_BUF_SIZE - rp + wp;
    }

    return wp - rp;
}

/* Copy TinyUSB CDC RX into the ring without overwriting unread bytes.
 * Leftover bytes stay in TinyUSB so the host is NAKed (flow control). */
static void cdc_rx_drain_tud(uint8_t itf)
{
    uint8_t tmp[CFG_TUD_CDC_RX_BUFSIZE];
    unsigned int wp = cdc_rx_buf.wp;
    bool wrote = false;

    for (;;) {
        unsigned int rp = cdc_rx_buf.rp;
        unsigned int used;
        unsigned int space;
        unsigned int contig;
        uint32_t avail;
        uint32_t n;

        if (wp < rp) {
            used = SERIAL_BUF_BUF_SIZE - rp + wp;
        } else {
            used = wp - rp;
        }
        space = SERIAL_BUF_BUF_SIZE - 1u - used;
        avail = tud_cdc_n_available(itf);
        if ((space == 0) || (avail == 0)) {
            break;
        }

        n = avail;
        if (n > space) {
            n = space;
        }
        if (n > sizeof(tmp)) {
            n = sizeof(tmp);
        }
        contig = SERIAL_BUF_BUF_SIZE - wp;
        if (n > contig) {
            n = contig;
        }

        n = tud_cdc_n_read(itf, tmp, n);
        if (n == 0) {
            break;
        }

        memcpy(&cdc_rx_buf.buf[wp], tmp, n);
        wp = (wp + n) % SERIAL_BUF_BUF_SIZE;
        wrote = true;
    }

    if (wrote) {
        cdc_rx_buf.wp = wp;
        if (cdc_sem) {
            xSemaphoreGive(cdc_sem);
        }
    }
}

const uint8_t *tud_descriptor_device_cb(void)
{
    return (uint8_t const *) &desc_device;
}

const uint8_t *tud_descriptor_device_qualifier_cb(void)
{
    return (uint8_t const *) &desc_device_qualifier;
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)(index);
    return desc_configuration;
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    static uint16_t descstr[32 + 1];
    size_t char_count;

    (void) langid;

    switch (index) {
    case STRID_LANGID:
        memcpy(&descstr[1], string_desc_arr[STRID_LANGID], 2);
        char_count = 1;
        break;
    case STRID_SERIAL:
        char_count = board_usb_get_serial(descstr + 1, 32);
        break;
    default:
        if ((index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
            == 0) {
            return NULL;
        }

        const char *str = string_desc_arr[index];
        char_count = strlen(str);
        size_t const max_count = sizeof(descstr) / sizeof(descstr[0]) - 1;
        if (char_count > max_count) {
            char_count = max_count;
        }
        for (size_t i = 0; i < char_count; i++) {
            descstr[1 + i] = str[i];
        }
        break;
    }

    descstr[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (char_count * 2 + 2));

    return descstr;
}

void tud_cdc_rx_cb(uint8_t itf)
{
    cdc_rx_drain_tud(itf);
}

void tud_mount_cb(void)
{
    //serial0_printf("%s\n", __FUNCTION__);
}

void tud_umount_cb(void)
{
    //serial0_printf("%s\n", __FUNCTION__);
}

void tud_suspend_cb(__unused bool remote_wakeup_en)
{
    //serial0_printf("%s:%d\n", __FUNCTION__, remote_wakeup_en);
}

void tud_resume_cb(void)
{
    //serial0_printf("%s\n", __FUNCTION__);
}

void usbcdc_init(void)
{
    if (cdc_sem == NULL) {
        cdc_sem = xSemaphoreCreateBinary();
    }
    if (cdc_mutex == NULL) {
        cdc_mutex = xSemaphoreCreateMutex();
    }
}

void usbcdc_deinit(void)
{
    if (cdc_sem) {
        vSemaphoreDelete(cdc_sem);
        cdc_sem = NULL;
    }
    if (cdc_mutex) {
        vSemaphoreDelete(cdc_mutex);
    }
}

void usbcdc_task(void)
{
    xSemaphoreTake(cdc_mutex, portMAX_DELAY);
    tud_task();
    /* Pull any leftover CDC RX now that the ring may have space. */
    cdc_rx_drain_tud(ITF_NUM_CDC_0);
    xSemaphoreGive(cdc_mutex);
}

int usbcdc_is_connected(void)
{
   int itf = ITF_NUM_CDC_0;

   return tud_cdc_n_connected(itf);
}

int usbcdc_write(const void *buf, size_t len)
{
    int ret = 0;
    int itf = ITF_NUM_CDC_0;
    const uint8_t *data = (const uint8_t *) buf;

    while (len > 0) {
        uint32_t avail;
        uint32_t n;
        int wl = 0;

        xSemaphoreTake(cdc_mutex, portMAX_DELAY);

        if (!tud_cdc_n_connected(itf)) {
            xSemaphoreGive(cdc_mutex);
            goto done;
        }

        avail = tud_cdc_n_write_available(itf);
        if (avail == 0) {
            /* Drop the mutex so usbcdc_task() can run tud_task() and drain
             * the IN endpoint. Holding it here would deadlock and drop data. */
            tud_cdc_n_write_flush(itf);
            xSemaphoreGive(cdc_mutex);
            vTaskDelay(1);
            continue;
        }

        n = (avail < (uint32_t) len) ? avail : (uint32_t) len;
        wl = (int) tud_cdc_n_write(itf, data, n);
        if (wl > 0) {
            tud_cdc_n_write_flush(itf);
            len -= (size_t) wl;
            data += wl;
            ret += wl;
        }

        xSemaphoreGive(cdc_mutex);

        if (wl <= 0) {
            vTaskDelay(1);
        }
    }

done:

    return ret;
}

int usbcdc_printf(const char *format, ...)
{
    int ret = 0;
    va_list ap;

    va_start(ap, format);
    ret = usbcdc_vprintf(format, ap);
    va_end(ap);

    return ret;
}

int usbcdc_vprintf(const char *format, va_list ap)
{
    int ret = 0;
    int len;

    len = vsnprintf(pbuf, SERIAL_PBUF_SIZE - 1, format, ap);
    if (len < 0) {
        return -1;
    }
    if (len > (int) (SERIAL_PBUF_SIZE - 1)) {
        len = SERIAL_PBUF_SIZE - 1;
    }

    for (int i = 0; i < len; ) {
        int n;

        if (pbuf[i] == '\n') {
            n = usbcdc_write("\r\n", 2);
            if (n != 2) {
                break;
            }
            i++;
            ret++;
            continue;
        }

        {
            int run = i + 1;
            while ((run < len) && (pbuf[run] != '\n')) {
                run++;
            }
            n = usbcdc_write(pbuf + i, (size_t) (run - i));
            if (n != (run - i)) {
                ret += (n > 0) ? n : 0;
                break;
            }
            ret += n;
            i = run;
        }
    }

    return ret;
}

int usbcdc_rx_ready(void)
{
    return (int) cdc_ring_used();
}

int usbcdc_read(void *buf, size_t len)
{
    int ret = 0;
    uint8_t *dst;
    const uint8_t *src;
    unsigned int rp, wp;
    size_t size;

    dst = (uint8_t *) buf;
    src = (const uint8_t *) cdc_rx_buf.buf;
    size = SERIAL_BUF_BUF_SIZE;
    rp = cdc_rx_buf.rp;
    wp = cdc_rx_buf.wp;
    while ((len > 0) && (rp != wp)) {
        *dst = src[rp];
        dst++;
        len--;
        ret++;
        rp = (rp + 1u) % size;
    }

    cdc_rx_buf.rp = rp;

    return ret;
}

#endif  // !LIB_PICO_STDIO_USB

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
