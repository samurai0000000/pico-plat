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

#define NUM_CALIB_PARAMS    24
#define BMP280_STATUS_MEASURING  UINT8_C(0x08)
#define BMP280_CTRL_MEAS_FORCED  \
    ((0x01 << 5) | (0x03 << 2) | BME280_FORCED_MODE)

#ifndef BMP280_I2C_TIMEOUT_US
#define BMP280_I2C_TIMEOUT_US  50000
#endif

static void bmp280_delay_ms(uint32_t ms)
{
    if ((ms == 0) ||
        (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)) {
        sleep_ms(ms);
    } else {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
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
    bmp280_delay_ms(2);

    _present = false;

    for (i = 0; i < (sizeof(addrs) / sizeof(addrs[0])); i++) {
        _i2cAddr = addrs[i];
        for (attempt = 0; attempt < 3; attempt++) {
            uint8_t chip_id = 0;

            if (!i2c_write(BME280_RESET_ADDR, BME280_SOFT_RESET_COMMAND)) {
                bmp280_delay_ms(10);
                continue;
            }

            /* Datasheet: 2 ms for NVM copy after reset */
            bmp280_delay_ms(2);

            if (!i2c_read(BME280_CHIP_ID_ADDR, &chip_id, sizeof(chip_id))) {
                bmp280_delay_ms(10);
                continue;
            }

            if (bmp280_is_chip_id(chip_id)) {
                _present = true;
                break;
            }
        }
        if (_present) {
            break;
        }
    }

    if (_present == false) {
        _i2cAddr = 0;
        goto done;
    }

    /*
     * CONFIG can only be written in sleep mode (post-reset).
     * t_sb unused in forced mode; filter coeff 16, 3-wire SPI disabled.
     */
    if (!i2c_write(BME280_CONFIG_ADDR, (0x04 << 5) | (0x04 << 2))) {
        _present = false;
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

bool Bmp280::isPresent(void) const
{
    return _present;
}

uint8_t Bmp280::i2cAddr(void) const
{
    return _i2cAddr;
}

bool Bmp280::read(float &temperature, float &pressure)
{
    bool result = false;
    uint8_t buf[6];
    int32_t t, p;

    temperature = 0.0;
    pressure = 0.0;

    if (!_initialized || !_present) {
        result = false;
        goto done;
    }

    /* osrs_t x1, osrs_p x4, forced mode — one sample, then sleep. */
    if (!i2c_write(BME280_CTRL_MEAS_ADDR, BMP280_CTRL_MEAS_FORCED)) {
        result = false;
        goto done;
    }

    if (!wait_meas_done()) {
        result = false;
        goto done;
    }

    if (!i2c_read(BME280_DATA_ADDR, buf, sizeof(buf))) {
        result = false;
        goto done;
    }

    // Store the 20 bit read in a 32 bit signed integer for conversion
    p = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    t = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);

    if ((t == 0) && (p == 0)) {
        result = false;
        goto done;
    }

    temperature = ((float) convertTemp(t)) / 100.f;
    /* Datasheet 32-bit compensation returns Pa; convert to hPa. */
    pressure = ((float) convertPressure(p, t)) / 100.f;

    result = true;

done:

    return result;
}

bool Bmp280::wait_meas_done(void)
{
    /*
     * osrs_t x1 + osrs_p x4 is ~12 ms. Tick is 10 ms, so wait two ticks
     * instead of polling status (those I2C reads stole Ctrl-C).
     */
    bmp280_delay_ms(20);

    return true;
}

int32_t Bmp280::convert(int32_t temp)
{
    int32_t var1, var2;

    var1 = ((((temp >> 3) - ((int32_t) _params.dig_t1 << 1))) *
            ((int32_t) _params.dig_t2)) >> 11;
    var2 = (((((temp >> 4) - ((int32_t) _params.dig_t1)) *
              ((temp >> 4) - ((int32_t) _params.dig_t1))) >> 12) *
            ((int32_t) _params.dig_t3)) >> 14;

    return var1 + var2;
}


int32_t Bmp280::convertTemp(int32_t temp)
{
    int32_t t_fine = convert(temp);

    return (t_fine * 5 + 128) >> 8;
}

int32_t Bmp280::convertPressure(int32_t pressure, int32_t temp)
{
    int32_t t_fine = convert(temp);
    int32_t var1, var2;
    uint32_t converted = 0;

    var1 = (((int32_t) t_fine) >> 1) - (int32_t) 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t) _params.dig_p6);
    var2 += ((var1 * ((int32_t) _params.dig_p5)) << 1);
    var2 = (var2 >> 2) + (((int32_t) _params.dig_p4) << 16);
    var1 = (((_params.dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
            ((((int32_t) _params.dig_p2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t) _params.dig_p1)) >> 15);

    if (var1 == 0) {
        // Avoid division by zero
        converted = 0;
    } else {
        converted = (((uint32_t)(((int32_t) 1048576) - pressure) -
                      (var2 >> 12))) * 3125;
        if (converted < 0x80000000) {
            converted = (converted << 1) / ((uint32_t) var1);
        } else {
            converted = (converted / (uint32_t) var1) * 2;
        }
        var1 = (((int32_t) _params.dig_p9) *
                ((int32_t)(((converted >> 3) * (converted >> 3)) >> 13)))
            >> 12;
        var2 = (((int32_t) (converted >> 2)) * ((int32_t) _params.dig_p8))
            >> 13;
        converted = (uint32_t)((int32_t) converted +
                               ((var1 + var2 + _params.dig_p7) >> 4));
    }

    return converted;
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
