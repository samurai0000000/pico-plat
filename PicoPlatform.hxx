/*
 * PicoPlatform.hxx
 *
 * Copyright (C) 2025, Charles Chiou
 */

#ifndef PICOPLATFORM_HXX
#define PICOPLATFORM_HXX

#include <string>
#include <memory>

class PicoPlatform : public std::enable_shared_from_this<PicoPlatform> {

public:

    static std::shared_ptr<PicoPlatform> get(void);

    /*
     * Distinguish Pico vs Pico W from ADC3. Must run before
     * cyw43_arch_init(): on Pico W, GPIO29 is the wireless clock after that.
     */
    static bool detectWireless(void);
    static bool initWireless(void);

    std::string getName(void) const;

    bool hasWireless(void) const;
    void flipOnboardLed(void);
    float getOnboardTempC(void) const;
    void reboot(void);
    void bootsel(void);

protected:

    friend std::shared_ptr<PicoPlatform> std::make_shared<PicoPlatform>();
    static std::shared_ptr<PicoPlatform> pp;

    PicoPlatform();
    ~PicoPlatform();

    bool _hasW;
    bool _onboardLed;

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
