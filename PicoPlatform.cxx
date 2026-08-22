/*
 * PicoPlatform.cxx
 *
 * Copyright (C) 2025, Charles Chiou
 */

#include <assert.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>
#include <hardware/watchdog.h>
#include <hardware/gpio.h>
#include <hardware/adc.h>
#include <hardware/clocks.h>
#include <pico/cyw43_arch.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <pico-plat.h>
#include <PicoPlatform.hxx>

using namespace std;

shared_ptr<PicoPlatform> PicoPlatform::pp = NULL;

/* -1 unknown, 0 Pico, 1 Pico W */
static int s_hasW = -1;
static int s_cyw43_ok = 0;
static SemaphoreHandle_t s_get_mutex = NULL;
static shared_ptr<PicoPlatform> (*s_factory)(void) = NULL;

static void lock_get(void)
{
    if (s_get_mutex == NULL) {
        s_get_mutex = xSemaphoreCreateMutex();
        assert(s_get_mutex != NULL);
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        xSemaphoreTake(s_get_mutex, portMAX_DELAY);
    }
}

static void unlock_get(void)
{
    if ((s_get_mutex != NULL) &&
        (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
        xSemaphoreGive(s_get_mutex);
    }
}

void PicoPlatform::setFactory(shared_ptr<PicoPlatform> (*factory)(void))
{
    s_factory = factory;
}

shared_ptr<PicoPlatform> PicoPlatform::get(void)
{
    shared_ptr<PicoPlatform> result;
    ::
    lock_get();
    if (PicoPlatform::pp == NULL) {
        if (s_factory != NULL) {
            PicoPlatform::pp = s_factory();
        } else {
            PicoPlatform::pp =
                shared_ptr<PicoPlatform>(new PicoPlatform(),
                                         [](PicoPlatform *p) {
                    delete p;
                });
        }
    }
    result = PicoPlatform::pp;
    unlock_get();

    return result;
}

bool PicoPlatform::detectWireless(void)
{
    static const float conversion_factor = 3.3f / (1 << 12);
    float voltage;

    if (s_hasW >= 0) {
        return s_hasW != 0;
    }

    adc_init();
    adc_set_temp_sensor_enabled(true);

    adc_select_input(3);
    voltage = adc_read() * conversion_factor;
    if ((voltage >= 0.35f) && (voltage < 0.45f)) {
        s_hasW = false;
    } else {
        s_hasW = true;
    }

    return s_hasW == true;
}

bool PicoPlatform::initWireless(void)
{
    /* Create the get() mutex on the main thread before the scheduler. */
    lock_get();
    unlock_get();

    if (!detectWireless()) {
        s_cyw43_ok = 0;
        return false;
    }

    if (s_cyw43_ok) {
        return true;
    }

    if (cyw43_arch_init() == 0) {
        s_cyw43_ok = 1;
        return true;
    }

    s_cyw43_ok = 0;
    return false;
}

PicoPlatform::PicoPlatform()
{
    adc_init();
    adc_set_temp_sensor_enabled(true);

    _hasW = detectWireless();
    _onboardLed = false;
    if (!hasWireless()) {
        gpio_init(25);
        gpio_set_dir(25, GPIO_OUT);
        gpio_put(25, false);
    }
}

PicoPlatform::~PicoPlatform()
{

}

string PicoPlatform::getName(void) const
{
    if (_hasW) {
        return "PicoW";
    } else {
        return "Pico";
    }
}

bool PicoPlatform::hasWireless(void) const
{
    return _hasW;
}

void PicoPlatform::flipOnboardLed(void)
{
    _onboardLed = !_onboardLed;
    if (hasWireless()) {
        if (s_cyw43_ok) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, _onboardLed);
        }
    } else {
        gpio_put(25, _onboardLed);
    }
}

float PicoPlatform::getOnboardTempC(void) const
{
    static const float conversionFactor = 3.3f / (1 << 12);
    float adc;
    float temperature_c = 0.0;

    adc_select_input(4);
    adc = (float) adc_read() * conversionFactor;
    temperature_c = 27.0f - (adc - 0.706f) / 0.001721f;

    return temperature_c;
}

void PicoPlatform::reboot(void)
{
    watchdog_enable(1, 0);
    for(;;);
}

void PicoPlatform::bootsel(void)
{
    reset_usb_boot(0, 0);
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
