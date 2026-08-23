/*
 * Bme280.cxx
 *
 * Copyright (C) 2025, Charles Chiou
 */

#include <cstring>
#include <FreeRTOS.h>
#include <task.h>
#include <pico/stdlib.h>
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
    _delay = 0;
    _spiPort = NULL;
    _i2cPort = NULL;
    _i2cAddr = 0;
    _i2cSda = 0;
    _i2cScl = 0;

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
    gpio_set_function(_spiSck, GPIO_FUNC_SPI);
    gpio_set_function(_spiRx, GPIO_FUNC_SPI);
    gpio_set_function(_spiTx, GPIO_FUNC_SPI);
    gpio_init(_spiCs);
    gpio_set_dir(_spiCs, GPIO_OUT);
    gpio_put(_spiCs, true);

    /* Datasheet: 2 ms after power-on */
    sleep_ms(2);

    result = bme280_init(&_dev);
    if (result != BME280_OK) {
        goto done;
    }

    result = this->configure();
    if (result != BME280_OK) {
        goto done;
    }

    _initialized = true;

done:

    return;
}

Bme280::Bme280(uint32_t i2cPort, uint32_t i2cSda, uint32_t i2cScl)
{
    int8_t result = BME280_OK;
    static const uint8_t addrs[] = {
        BME280_I2C_ADDR_PRIM,
        BME280_I2C_ADDR_SEC,
    };
    unsigned int i;

    _initialized = false;
    _delay = 0;
    _spiPort = NULL;
    _spiSck = 0;
    _spiTx = 0;
    _spiRx = 0;
    _spiCs = 0;
    _i2cPort = NULL;
    _i2cAddr = 0;

    switch (i2cPort) {
    case 0: _i2cPort = i2c0; break;
    case 1: _i2cPort = i2c1; break;
    default: goto done; break;
    }

    _i2cSda = i2cSda;
    _i2cScl = i2cScl;

    bzero(&_dev, sizeof(_dev));

    _dev.intf_ptr = this;
    _dev.intf     = BME280_I2C_INTF;
    _dev.read     = this->i2c_read;
    _dev.write    = this->i2c_write;
    _dev.delay_us = this->delay_us;

    /* 100 kHz: onboard pull-ups are weak; 400 kHz often NACKs. */
    plat_i2c_setup(_i2cPort, _i2cSda, _i2cScl);

    /* Datasheet: 2 ms after power-on; clones often need a bit more. */
    sleep_ms(10);

    result = BME280_E_DEV_NOT_FOUND;
    for (i = 0; i < (sizeof(addrs) / sizeof(addrs[0])); i++) {
        _i2cAddr = addrs[i];
        result = bme280_init(&_dev);
        if (result == BME280_OK) {
            break;
        }
    }
    if (result != BME280_OK) {
        _i2cAddr = 0;
        goto done;
    }

    result = this->configure();
    if (result != BME280_OK) {
        goto done;
    }

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

uint8_t Bme280::i2cAddr(void) const
{
    return _i2cAddr;
}

int8_t Bme280::configure(void)
{
    int8_t result;

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

done:

    return result;
}

int8_t Bme280::wait_meas_done(void)
{
    int8_t result = BME280_OK;
    unsigned int i;
    uint8_t status;

    for (i = 0; i < 5; i++) {
        status = 0x08;
        result = _dev.read(BME280_STATUS_REG_ADDR, &status, 1, _dev.intf_ptr);
        if (result != BME280_INTF_RET_SUCCESS) {
            goto done;
        }
        if ((status & UINT8_C(0x08)) == 0) {
            result = BME280_OK;
            goto done;
        }
        delay_us(10000, _dev.intf_ptr);
    }

    result = BME280_E_COMM_FAIL;

done:

    return result;
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

    /* cal_meas_delay is a minimum; add 1 ms so the first sample is ready. */
    _dev.delay_us(_delay + 1000u, _dev.intf_ptr);

    ret = this->wait_meas_done();
    if (ret != BME280_OK) {
        result = false;
        goto done;
    }

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
    uint32_t ms;

    (void)(intf_ptr);

    ms = (period + 999u) / 1000u;
    if ((period < os_tick_us) ||
        (ms == 0) ||
        (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)) {
        sleep_us(period);
    } else {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

int8_t Bme280::spi_read(uint8_t reg_addr, uint8_t *reg_data,
                        uint32_t len, void *intf_ptr)
{
    int8_t ret = BME280_INTF_RET_SUCCESS;
    Bme280 *bme280 = (Bme280 *) intf_ptr;
    int n;

    if ((bme280 == NULL) || (reg_data == NULL)) {
        ret = BME280_E_NULL_PTR;
        goto done;
    }

    gpio_put(bme280->_spiCs, 0);
    n = spi_write_blocking((spi_inst_t *) bme280->_spiPort, &reg_addr, 1);
    if (n != 1) {
        gpio_put(bme280->_spiCs, 1);
        ret = BME280_E_COMM_FAIL;
        goto done;
    }
    n = spi_read_blocking((spi_inst_t *) bme280->_spiPort, 0, reg_data, len);
    gpio_put(bme280->_spiCs, 1);
    if (n != (int) len) {
        ret = BME280_E_COMM_FAIL;
        goto done;
    }

done:

    return ret;
}

int8_t Bme280::spi_write(uint8_t reg_addr, const uint8_t *reg_data,
                         uint32_t len, void *intf_ptr)
{
    int8_t ret = BME280_INTF_RET_SUCCESS;
    Bme280 *bme280 = (Bme280 *) intf_ptr;
    int n;

    if ((bme280 == NULL) || (reg_data == NULL)) {
        ret = BME280_E_NULL_PTR;
        goto done;
    }

    gpio_put(bme280->_spiCs, 0);
    n = spi_write_blocking((spi_inst_t *) bme280->_spiPort, &reg_addr, 1);
    if (n != 1) {
        gpio_put(bme280->_spiCs, 1);
        ret = BME280_E_COMM_FAIL;
        goto done;
    }
    n = spi_write_blocking((spi_inst_t *) bme280->_spiPort, reg_data, len);
    gpio_put(bme280->_spiCs, 1);
    if (n != (int) len) {
        ret = BME280_E_COMM_FAIL;
        goto done;
    }

done:

    return ret;
}

#ifndef BME280_I2C_TIMEOUT_US
#define BME280_I2C_TIMEOUT_US  50000
#endif

int8_t Bme280::i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                        uint32_t len, void *intf_ptr)
{
    int8_t ret = BME280_INTF_RET_SUCCESS;
    Bme280 *bme280 = (Bme280 *) intf_ptr;
    i2c_inst_t *inst;
    int n;

    if ((bme280 == NULL) || (reg_data == NULL)) {
        ret = BME280_E_NULL_PTR;
        goto done;
    }

    inst = (i2c_inst_t *) bme280->_i2cPort;
    plat_i2c_lock();

    n = i2c_write_timeout_us(inst,
                             bme280->_i2cAddr,
                             &reg_addr,
                             sizeof(reg_addr),
                             true,
                             BME280_I2C_TIMEOUT_US);
    if (n != (int) sizeof(reg_addr)) {
        plat_i2c_recover(inst, bme280->_i2cSda, bme280->_i2cScl);
        ret = BME280_E_COMM_FAIL;
        goto unlock;
    }

    n = i2c_read_timeout_us(inst,
                            bme280->_i2cAddr,
                            reg_data,
                            len,
                            false,
                            BME280_I2C_TIMEOUT_US);
    if (n != (int) len) {
        plat_i2c_recover(inst, bme280->_i2cSda, bme280->_i2cScl);
        ret = BME280_E_COMM_FAIL;
        goto unlock;
    }

unlock:

    plat_i2c_unlock();

done:

    return ret;
}

int8_t Bme280::i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                         uint32_t len, void *intf_ptr)
{
    int8_t ret = BME280_INTF_RET_SUCCESS;
    Bme280 *bme280 = (Bme280 *) intf_ptr;
    i2c_inst_t *inst;
    uint8_t buf[32];
    int n;

    if ((bme280 == NULL) || (reg_data == NULL)) {
        ret = BME280_E_NULL_PTR;
        goto done;
    }

    if ((len == 0) || ((len + 1) > sizeof(buf))) {
        ret = BME280_E_INVALID_LEN;
        goto done;
    }

    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, len);

    inst = (i2c_inst_t *) bme280->_i2cPort;
    plat_i2c_lock();

    n = i2c_write_timeout_us(inst,
                             bme280->_i2cAddr,
                             buf,
                             len + 1,
                             false,
                             BME280_I2C_TIMEOUT_US);
    if (n != (int) (len + 1)) {
        plat_i2c_recover(inst, bme280->_i2cSda, bme280->_i2cScl);
        ret = BME280_E_COMM_FAIL;
        goto unlock;
    }

unlock:

    plat_i2c_unlock();

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
