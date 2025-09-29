/*
 * PicoPlatform.cxx
 *
 * Copyright (C) 2025, Charles Chiou
 */

#include <pico/bootrom.h>
#include <hardware/watchdog.h>
#include <hardware/gpio.h>
#include <hardware/adc.h>
#include <hardware/clocks.h>
#include <pico/cyw43_arch.h>
#include <pico-plat.h>
#include <PicoPlatform.hxx>

shared_ptr<PicoPlatform> PicoPlatform::pp = NULL;

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

PicoPlatform::PicoPlatform()
{
    static const  float conversion_factor = 3.3f / (1 << 12);
    float voltage;

    adc_init();
    adc_set_temp_sensor_enabled(true);

    adc_select_input(3);
    voltage = adc_read() * conversion_factor;
    if ((voltage >= 0.40f) && (voltage < 0.45f)) {
        _hasW = false;
    } else {
        _hasW = true;
    }

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
