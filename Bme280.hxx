/*
 * Bme280.hxx
 *
 * Copyright (C) 2025, Charles Chiou
 */

#ifndef BME280_HXX
#define BME280_HXX

#include <string>
#include <memory>
#include <pico-bme280/bme280.h>

using namespace std;

class Bme280 {

public:

    Bme280(uint32_t spiPort, uint32_t spiSck,
           uint32_t spiTx, uint32_t spiRx, uint32_t spiCs);
    Bme280(uint32_t i2cPort, uint32_t i2cSda, uint32_t i2cScl);
    ~Bme280();

    bool isInitialized(void) const;

private:

    static void delay_us(uint32_t period, void *intf_ptr);
    static int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data,
                           uint32_t len, void *intf_ptr);
    static int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data,
                            uint32_t len, void *intf_ptr);
    static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                           uint32_t len, void *intf_ptr);
    static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                            uint32_t len, void *intf_ptr);

    struct bme280_dev _dev;
    void *_spiPort;
    uint32_t _spiSck;
    uint32_t _spiTx;
    uint32_t _spiRx;
    uint32_t _spiCs;
    void *_i2cPort;
    uint32_t _i2cSda;
    uint32_t _i2cScl;

    bool _initialized;

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
