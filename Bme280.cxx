/*
 * Bme280.cxx
 *
 * Copyright (C) 2025, Charles Chiou
 */

#include <cstring>
#include <FreeRTOS.h>
#include <task.h>
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <hardware/i2c.h>
#include <pico-plat.h>
#include <Bme280.hxx>

Bme280::Bme280(uint32_t spiPort, uint32_t spiSck,
               uint32_t spiTx, uint32_t spiRx, uint32_t spiCs)
{
    int8_t result = BME280_OK;

    _initialized = false;

    switch (spiPort) {
    case 0: _spiPort = spi0; break;
    case 1: _spiPort = spi1; break;
    default: goto done; break;
    }

    _spiSck = spiSck;
    _spiTx = spiTx;
    _spiRx = spiRx;
    _spiCs = spiCs;

    bzero(&_dev, sizeof(_dev));

    _dev.intf_ptr = this;
    _dev.intf     = BME280_SPI_INTF;
    _dev.read     = this->spi_read;
    _dev.write    = this->spi_write;
    _dev.delay_us = this->delay_us;

    spi_init((spi_inst_t *) _spiPort, 1000000);  // 1MHz
    gpio_set_function(_spiSck,GPIO_FUNC_SPI);
    gpio_set_function(_spiRx, GPIO_FUNC_SPI);
    gpio_set_function(_spiTx, GPIO_FUNC_SPI);
    gpio_init(_spiCs);
    gpio_set_dir(_spiCs, GPIO_OUT);
    gpio_put(_spiCs, true);

    result = bme280_init(&_dev);
    if (result != BME280_OK) {
        bzero(&_dev, sizeof(_dev));
        goto done;
    }

    _initialized = true;

done:

    return;
}

Bme280::Bme280(uint32_t i2cPort, uint32_t i2cSda, uint32_t i2cScl)
{
    int8_t result = BME280_OK;

    _initialized = false;

    bzero(&_dev, sizeof(_dev));

    switch (i2cPort) {
    case 0: _i2cPort = i2c0; break;
    case 1: _i2cPort = i2c1; break;
    default: goto done; break;
    }

    _i2cSda = i2cSda;
    _i2cScl = i2cScl;

    _dev.intf_ptr = this;
    _dev.intf     = BME280_I2C_INTF;
    _dev.read     = this->i2c_read;
    _dev.write    = this->i2c_write;
    _dev.delay_us = this->delay_us;

    i2c_init((i2c_inst_t *) _i2cPort, 400000); // 400kHz
    gpio_set_function(_i2cSda, GPIO_FUNC_I2C);
    gpio_set_function(_i2cScl, GPIO_FUNC_I2C);
    gpio_pull_up(_i2cSda);
    gpio_pull_up(_i2cScl);

    result = bme280_init(&_dev);
    if (result != BME280_OK) {
        goto done;
    }

    _dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    _dev.settings.osr_p = BME280_OVERSAMPLING_16X;
    _dev.settings.osr_t = BME280_OVERSAMPLING_2X;
    _dev.settings.filter = BME280_FILTER_COEFF_16;

    result = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL |
                                        BME280_OSR_TEMP_SEL |
                                        BME280_OSR_HUM_SEL |
                                        BME280_FILTER_SEL,
                                        &_dev);
    if (result != BME280_OK) {
        goto done;
    }

    _delay = bme280_cal_meas_delay(&_dev.settings);

    _initialized = true;

done:

    return;
}

Bme280::~Bme280()
{

}

bool Bme280::isInitialized(void) const
{
    return _initialized;
}

bool Bme280::readSensorData(struct bme280_data &data)
{
    bool result = false;
    int8_t ret;

    bzero(&data, sizeof(data));

    if (_initialized == false) {
        result = false;
        goto done;
    }

    ret = bme280_set_sensor_mode(BME280_FORCED_MODE, &_dev);
    if (ret != BME280_OK) {
        result = false;
        goto done;
    }

    _dev.delay_us(_delay, _dev.intf_ptr);

    ret = bme280_get_sensor_data(BME280_ALL, &data, &_dev);
    if (ret != BME280_OK) {
        result = false;
        goto done;
    }

    result = true;

done:

    return result;
}

void Bme280::delay_us(uint32_t period, void *intf_ptr)
{
    static const uint32_t os_tick_us = (1000000 / configTICK_RATE_HZ);

    (void)(intf_ptr);

    if (period < os_tick_us) {
        sleep_us(period);
    } else {
        vTaskDelay(pdMS_TO_TICKS(period / 1000));
    }
}

int8_t Bme280::spi_read(uint8_t reg_addr, uint8_t *reg_data,
                        uint32_t len, void *intf_ptr)
{
    int8_t ret = BME280_OK;
    Bme280 *bme280 = (Bme280 *) intf_ptr;

    if (bme280 == NULL) {
        ret = BME280_E_NULL_PTR;
        goto done;
    }

    gpio_put(bme280->_spiCs, 0);
    spi_write_blocking((spi_inst_t *) bme280->_spiPort, &reg_addr, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    spi_read_blocking((spi_inst_t *) bme280->_spiPort, 0, reg_data, len);
    gpio_put(bme280->_spiCs, 1);

done:

    return ret;
}

int8_t Bme280::spi_write(uint8_t reg_addr, const uint8_t *reg_data,
                         uint32_t len, void *intf_ptr)
{
    int8_t ret = BME280_OK;
    Bme280 *bme280 = (Bme280 *) intf_ptr;

    if (bme280 == NULL) {
        ret = BME280_E_NULL_PTR;
        goto done;
    }

    gpio_put(bme280->_spiCs, 0);
    spi_write_blocking((spi_inst_t *) bme280->_spiPort, &reg_addr, 1);
    spi_write_blocking((spi_inst_t *) bme280->_spiPort, reg_data, len);
    gpio_put(bme280->_spiCs, 1);

done:

    return ret;
}

int8_t Bme280::i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                        uint32_t len, void *intf_ptr)
{
    int8_t ret = BME280_OK;
    Bme280 *bme280 = (Bme280 *) intf_ptr;

    if (bme280 == NULL) {
        ret = BME280_E_NULL_PTR;
        goto done;
    }

    i2c_write_blocking((i2c_inst_t *) bme280->_i2cPort,
                       BME280_I2C_ADDR_PRIM,
                       &reg_addr,
                       sizeof(reg_addr),
                       true);
    i2c_read_blocking((i2c_inst_t *) bme280->_i2cPort,
                      BME280_I2C_ADDR_PRIM,
                      reg_data,
                      len,
                      false);

done:

    return ret;
}

int8_t Bme280::i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                         uint32_t len, void *intf_ptr)
{
    int8_t ret = BME280_OK;
    Bme280 *bme280 = (Bme280 *) intf_ptr;

    if (bme280 == NULL) {
        ret = BME280_E_NULL_PTR;
        goto done;
    }

    i2c_write_blocking((i2c_inst_t *) bme280->_i2cPort,
                       BME280_I2C_ADDR_PRIM,
                       &reg_addr,
                       sizeof(reg_addr),
                       true);
    i2c_write_blocking((i2c_inst_t *) bme280->_i2cPort,
                       BME280_I2C_ADDR_PRIM,
                       reg_data,
                       len,
                       false);

done:

    return ret;
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
