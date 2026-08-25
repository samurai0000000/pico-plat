/*
 * Bmp280.cxx
 *
 * Copyright (C) 2025, Charles Chiou
 */

#include <cstring>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <FreeRTOS.h>
#include <task.h>
#include <pico-bme280/bme280.h>
#include <pico-plat.h>
#include <Bmp280.hxx>

#define NUM_CALIB_PARAMS         24
#define BMP280_STATUS_MEASURING  UINT8_C(0x08)
#define BMP280_STATUS_IM_UPDATE  UINT8_C(0x01)
#define BMP280_CTRL_MEAS_FORCED  \
    ((0x01 << 5) | (0x03 << 2) | BME280_FORCED_MODE)

#ifndef BMP280_I2C_TIMEOUT_US
#define BMP280_I2C_TIMEOUT_US    50000
#endif

static void bmp280_delay_us(uint32_t us)
{
    static const uint32_t os_tick_us = (1000000 / configTICK_RATE_HZ);
    uint32_t ms;

    ms = (us + 999u) / 1000u;
    if ((us < os_tick_us) ||
        (ms == 0) ||
        (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)) {
        sleep_us(us);
    } else {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

static void bmp280_delay_ms(uint32_t ms)
{
    bmp280_delay_us(ms * 1000u);
}

static bool bmp280_is_chip_id(uint8_t id)
{
    return (id == UINT8_C(0x56)) ||
           (id == UINT8_C(0x57)) ||
           (id == UINT8_C(0x58));
}

static uint16_t le16u(const uint8_t *p)
{
    return (uint16_t) (((uint16_t) p[1] << 8) | p[0]);
}

static int16_t le16s(const uint8_t *p)
{
    return (int16_t) le16u(p);
}

Bmp280::Bmp280(uint32_t i2cPort, uint32_t i2cSda, uint32_t i2cScl)
{
    _i2cPort = NULL;
    _i2cSda = i2cSda;
    _i2cScl = i2cScl;
    _i2cAddr = 0;
    _initialized = false;
    _present = false;
    bzero(&_params, sizeof(_params));

    switch (i2cPort) {
    case 0: _i2cPort = i2c0; break;
    case 1: _i2cPort = i2c1; break;
    default: return;
    }

    probe();
}

Bmp280::~Bmp280()
{

}

bool Bmp280::isPresent(void) const
{
    return _present;
}

bool Bmp280::isInitialized(void) const
{
    return _initialized;
}

uint8_t Bmp280::i2cAddr(void) const
{
    return _i2cAddr;
}

bool Bmp280::softReset(void)
{
    uint8_t chip_id = 0;

    if ((_i2cPort == NULL) || (_i2cAddr == 0)) {
        return false;
    }

    if (!i2c_write(BME280_RESET_ADDR, BME280_SOFT_RESET_COMMAND)) {
        return false;
    }

    /* Datasheet: 2 ms for NVM copy after reset */
    bmp280_delay_ms(3);

    /* Poll status im_update bit until NVM copy is complete */
    for (unsigned int retry = 0; retry < 5; retry++) {
        uint8_t status = 0;
        if (i2c_read(BME280_STATUS_REG_ADDR, &status, sizeof(status))) {
            if ((status & BMP280_STATUS_IM_UPDATE) == 0) {
                break;
            }
        }
        bmp280_delay_ms(1);
    }

    if (!i2c_read(BME280_CHIP_ID_ADDR, &chip_id, sizeof(chip_id))) {
        return false;
    }

    if (!bmp280_is_chip_id(chip_id)) {
        return false;
    }

    /*
     * CONFIG can only be written in sleep mode (post-reset).
     * t_sb unused in forced mode; filter coeff 16, 3-wire SPI disabled.
     */
    if (!i2c_write(BME280_CONFIG_ADDR, (0x04 << 5) | (0x04 << 2))) {
        return false;
    }

    return true;
}

void Bmp280::probe(void)
{
    static const uint8_t addrs[] = {
        BME280_I2C_ADDR_PRIM,
        BME280_I2C_ADDR_SEC,
    };
    uint8_t buf[NUM_CALIB_PARAMS];
    unsigned int i, attempt;

    if ((_initialized) || (_i2cPort == NULL)) {
        return;
    }

    /* 100 kHz: same as BME280; onboard pull-ups are weak. */
    plat_i2c_setup(_i2cPort, _i2cSda, _i2cScl);

    /* Datasheet: 2 ms after power-on */
    bmp280_delay_ms(3);

    _present = false;

    for (i = 0; i < (sizeof(addrs) / sizeof(addrs[0])); i++) {
        _i2cAddr = addrs[i];
        for (attempt = 0; attempt < 3; attempt++) {
            if (softReset()) {
                _present = true;
                break;
            }
            bmp280_delay_ms(10);
        }
        if (_present) {
            break;
        }
    }

    if (_present == false) {
        _i2cAddr = 0;
        goto done;
    }

    bzero(buf, sizeof(buf));
    if (!i2c_read(BME280_TEMP_PRESS_CALIB_DATA_ADDR, buf, NUM_CALIB_PARAMS)) {
        _present = false;
        goto done;
    }

    _params.dig_t1 = le16u(&buf[0]);
    _params.dig_t2 = le16s(&buf[2]);
    _params.dig_t3 = le16s(&buf[4]);
    _params.dig_p1 = le16u(&buf[6]);
    _params.dig_p2 = le16s(&buf[8]);
    _params.dig_p3 = le16s(&buf[10]);
    _params.dig_p4 = le16s(&buf[12]);
    _params.dig_p5 = le16s(&buf[14]);
    _params.dig_p6 = le16s(&buf[16]);
    _params.dig_p7 = le16s(&buf[18]);
    _params.dig_p8 = le16s(&buf[20]);
    _params.dig_p9 = le16s(&buf[22]);

    _initialized = true;

done:

    if (_present == false) {
        _initialized = false;
        bzero(&_params, sizeof(_params));
    }

    return;
}

bool Bmp280::wait_meas_done(void)
{
    /*
     * osrs_t x1 + osrs_p x4 typical measurement time is ~10 ms, max ~14 ms.
     * Wait 15 ms, then poll status register measuring bit (bit 3)
     * and im_update bit (bit 0) to ensure conversion has completed.
     */
    bmp280_delay_ms(15);

    for (unsigned int i = 0; i < 5; i++) {
        uint8_t status = 0x08;
        if (!i2c_read(BME280_STATUS_REG_ADDR, &status, sizeof(status))) {
            return false;
        }

        if ((status & (BMP280_STATUS_MEASURING | BMP280_STATUS_IM_UPDATE)) == 0) {
            return true;
        }

        bmp280_delay_ms(2);
    }

    return false;
}

bool Bmp280::read(float &temperature, float &pressure)
{
    bool result = false;
    uint8_t buf[6];
    uint8_t ctrl_meas = 0;
    int32_t t, p;
    double t_fine = 0.0;
    double temp_degc, press_hpa;

    temperature = 0.0f;
    pressure = 0.0f;

    if (!_initialized || !_present) {
        probe();
        if (!_initialized || !_present) {
            result = false;
            goto done;
        }
    }

    /* Check current mode; if not in sleep mode, put sensor to sleep */
    if (i2c_read(BME280_CTRL_MEAS_ADDR, &ctrl_meas, sizeof(ctrl_meas))) {
        if ((ctrl_meas & 0x03) != BME280_SLEEP_MODE) {
            if (!softReset()) {
                result = false;
                goto done;
            }
        }
    }

    /* osrs_t x1, osrs_p x4, forced mode — trigger one measurement. */
    if (!i2c_write(BME280_CTRL_MEAS_ADDR, BMP280_CTRL_MEAS_FORCED)) {
        softReset();
        result = false;
        goto done;
    }

    if (!wait_meas_done()) {
        softReset();
        result = false;
        goto done;
    }

    if (!i2c_read(BME280_DATA_ADDR, buf, sizeof(buf))) {
        softReset();
        result = false;
        goto done;
    }

    // Store the 20-bit read in signed 32-bit integers for conversion
    p = ((int32_t) buf[0] << 12) | ((int32_t) buf[1] << 4) | ((int32_t) buf[2] >> 4);
    t = ((int32_t) buf[3] << 12) | ((int32_t) buf[4] << 4) | ((int32_t) buf[5] >> 4);

    // 0x80000 is outputted by BMP280 when measurement is skipped or incomplete
    if ((t == 0x80000) || (p == 0x80000) || ((t == 0) && (p == 0))) {
        softReset();
        result = false;
        goto done;
    }

    temp_degc = compensateTemp(t, t_fine);
    press_hpa = compensatePressure(p, t_fine);

    temperature = (float) temp_degc;
    pressure = (float) press_hpa;

    result = true;

done:

    return result;
}

double Bmp280::compensateTemp(int32_t raw_temp, double &t_fine)
{
    double var1, var2, temp;

    var1 = (((double) raw_temp) / 16384.0 - ((double) _params.dig_t1) / 1024.0) * ((double) _params.dig_t2);
    var2 = ((((double) raw_temp) / 131072.0 - ((double) _params.dig_t1) / 8192.0) *
            (((double) raw_temp) / 131072.0 - ((double) _params.dig_t1) / 8192.0)) * ((double) _params.dig_t3);
    t_fine = var1 + var2;
    temp = (var1 + var2) / 5120.0;

    if (temp < -40.0) {
        temp = -40.0;
    } else if (temp > 85.0) {
        temp = 85.0;
    }

    return temp;
}

double Bmp280::compensatePressure(int32_t raw_press, double t_fine)
{
    double var1, var2, var3, press;

    var1 = (t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double) _params.dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double) _params.dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double) _params.dig_p4) * 65536.0);
    var3 = ((double) _params.dig_p3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double) _params.dig_p2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double) _params.dig_p1);

    if (var1 > 0.0) {
        press = 1048576.0 - (double) raw_press;
        press = (press - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double) _params.dig_p9) * press * press / 2147483648.0;
        var2 = press * ((double) _params.dig_p8) / 32768.0;
        press = press + (var1 + var2 + ((double) _params.dig_p7)) / 16.0;

        if (press < 30000.0) {
            press = 30000.0;
        } else if (press > 110000.0) {
            press = 110000.0;
        }
    } else {
        press = 30000.0;
    }

    return press / 100.0; // Return in hPa
}

bool Bmp280::i2c_read(uint8_t addr, uint8_t *data, size_t len)
{
    bool result = false;
    i2c_inst_t *inst = (i2c_inst_t *) _i2cPort;
    int n;

    if ((inst == NULL) || (data == NULL) || (len == 0)) {
        goto done;
    }

    plat_i2c_lock();

    n = i2c_write_timeout_us(inst, _i2cAddr, &addr, 1, true,
                             BMP280_I2C_TIMEOUT_US);
    if (n != 1) {
        plat_i2c_recover(inst, _i2cSda, _i2cScl);
        goto unlock;
    }

    n = i2c_read_timeout_us(inst, _i2cAddr, data, len, false,
                            BMP280_I2C_TIMEOUT_US);
    if (n != (int) len) {
        plat_i2c_recover(inst, _i2cSda, _i2cScl);
        goto unlock;
    }

    result = true;

unlock:

    plat_i2c_unlock();

done:

    return result;
}

bool Bmp280::i2c_write(uint8_t addr, const uint8_t *data, size_t len)
{
    bool result = false;
    i2c_inst_t *inst = (i2c_inst_t *) _i2cPort;
    uint8_t buf[32];
    int n;

    if ((inst == NULL) || (data == NULL) || (len == 0) ||
        ((len + 1) > sizeof(buf))) {
        goto done;
    }

    buf[0] = addr;
    memcpy(&buf[1], data, len);

    plat_i2c_lock();

    n = i2c_write_timeout_us(inst, _i2cAddr, buf, len + 1, false,
                             BMP280_I2C_TIMEOUT_US);
    if (n != (int) (len + 1)) {
        plat_i2c_recover(inst, _i2cSda, _i2cScl);
        goto unlock;
    }

    result = true;

unlock:

    plat_i2c_unlock();

done:

    return result;
}

bool Bmp280::i2c_write(uint8_t addr, uint8_t data)
{
    return i2c_write(addr, &data, 1);
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
