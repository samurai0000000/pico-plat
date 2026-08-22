/*
 * PicoPlatform.cxx
 *
 * Copyright (C) 2025, Charles Chiou
 */

#include <pico/stdlib.h>
#include <pico/bootrom.h>
#include <hardware/watchdog.h>
#include <hardware/gpio.h>
#include <hardware/adc.h>
#include <hardware/clocks.h>
#include <pico/cyw43_arch.h>
#include <pico-plat.h>
#include <PicoPlatform.hxx>

shared_ptr<PicoPlatform> PicoPlatform::pp = NULL;

/* -1 unknown, 0 Pico, 1 Pico W */
static int s_hasW = -1;

shared_ptr<PicoPlatform> PicoPlatform::get(void)
{
    if (PicoPlatform::pp == NULL) {
        PicoPlatform::pp =
            shared_ptr<PicoPlatform>(new PicoPlatform(), [](PicoPlatform *p) {
                delete p;
            });
    }

    return PicoPlatform::pp;
}

bool PicoPlatform::detectWireless(void)
{
    static const float conversion_factor = 3.3f / (1 << 12);
    float voltage;

    if (s_hasW >= 0) {
        return s_hasW != 0;
    }

    /*
     * Pico: ADC3 is VSYS/3 (~1.65 V on USB).
     * Pico W: GPIO25 is CYW43 CS. Held low, ADC3 is not VSYS and reads
     * well below 1 V (often ~0.4 V). The old 0.40–0.45 V window treated
     * that as a non-W board.
     */
    adc_init();
    adc_gpio_init(29);
    adc_select_input(3);

    gpio_init(25);
    gpio_set_dir(25, GPIO_IN);
    gpio_pull_down(25);
    sleep_us(20);

    voltage = adc_read() * conversion_factor;
    s_hasW = (voltage < 1.0f) ? 1 : 0;

    /* Hand GPIO25/29 back so cyw43_arch_init() can claim them on Pico W. */
    gpio_deinit(25);
    gpio_deinit(29);

    return s_hasW != 0;
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
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, _onboardLed);
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
