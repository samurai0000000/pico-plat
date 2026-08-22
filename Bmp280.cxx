/*
 * Bmp280.cxx
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
#include <pico-bme280/bme280.h>
#include <Bmp280.hxx>

#define BMP280_CHIP_ID      UINT8_C(0x58)
#define NUM_CALIB_PARAMS    24

Bmp280::Bmp280(uint32_t i2cPort, uint32_t i2cSda, uint32_t i2cScl)
{
    switch (i2cPort) {
    case 1:
        _i2cPort = i2c1;
        break;
    case 0:
    default:
        _i2cPort = i2c0;
        break;
    }
    _i2cSda = i2cSda;
    _i2cScl = i2cScl;
    _i2cAddr = BME280_I2C_ADDR_PRIM;
    _initialized = false;
    _present = false;

    probe();
}

Bmp280::~Bmp280()
{

}

void Bmp280::probe(void)
{
    bool result;
    uint8_t buf[NUM_CALIB_PARAMS];

    if (_initialized) {
        return;
    }

    i2c_init((i2c_inst_t *) _i2cPort, 100000); // 100kHz
    gpio_set_function(_i2cSda, GPIO_FUNC_I2C);
    gpio_set_function(_i2cScl, GPIO_FUNC_I2C);
    gpio_pull_up(_i2cSda);
    gpio_pull_up(_i2cScl);

    _initialized = true;
    _present = false;

    for (_i2cAddr = BME280_I2C_ADDR_PRIM;
         !_present && (_i2cAddr <= BME280_I2C_ADDR_SEC);
         _i2cAddr++) {
        for (unsigned int attempt = 0; attempt < 5; attempt++) {
            uint8_t chip_id = 0x0;

            /* Soft-reset the device */
            i2c_write(BME280_RESET_ADDR,
                      BME280_SOFT_RESET_COMMAND);
            vTaskDelay(pdMS_TO_TICKS(250));

            /* Read chip ID */
            result = i2c_read(BME280_CHIP_ID_ADDR, &chip_id, sizeof(chip_id));
            if (result == false) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            if (chip_id == BMP280_CHIP_ID) {
                _present = true;
                break;
            }
        }
    }

    if (_present == false) {
        goto done;
    }

    /* Configure 500ms sampling time, x16 filter */
    i2c_write(BME280_CONFIG_ADDR,
              ((0x04 << 5) | (0x05 << 2)) & 0xfc);

    /* Set osrs_t x1, osrs_p x4, normal mode operation */
    i2c_write(BME280_CTRL_MEAS_ADDR,
              (0x01 << 5) | (0x03 << 2) | (0x03));

    /* Get calibration parameters */
    bzero(buf, sizeof(buf));
    i2c_read(BME280_TEMP_PRESS_CALIB_DATA_ADDR, buf, NUM_CALIB_PARAMS);
    _params.dig_t1 = (uint16_t) (buf[1]  << 8) | buf[0];
    _params.dig_t2 =  (int16_t) (buf[3]  << 8) | buf[2];
    _params.dig_t3 =  (int16_t) (buf[5]  << 8) | buf[4];
    _params.dig_p1 = (uint16_t) (buf[7]  << 8) | buf[6];
    _params.dig_p2 =  (int16_t) (buf[9]  << 8) | buf[8];
    _params.dig_p3 =  (int16_t) (buf[11] << 8) | buf[10];
    _params.dig_p4 =  (int16_t) (buf[13] << 8) | buf[12];
    _params.dig_p5 =  (int16_t) (buf[15] << 8) | buf[14];
    _params.dig_p6 =  (int16_t) (buf[17] << 8) | buf[16];
    _params.dig_p7 =  (int16_t) (buf[19] << 8) | buf[18];
    _params.dig_p8 =  (int16_t) (buf[21] << 8) | buf[20];
    _params.dig_p9 =  (int16_t) (buf[23] << 8) | buf[22];

    vTaskDelay(pdMS_TO_TICKS(250));

    serial0_printf("buf=");
    for (unsigned int i = 0; i < NUM_CALIB_PARAMS; i++) {
        serial0_printf("%.2x", buf[i]);
    }
    serial0_printf("\n");

    serial0_printf("dig_t1=%u\n", _params.dig_t1);
    serial0_printf("dig_t2=%d\n", _params.dig_t2);
    serial0_printf("dig_t3=%d\n", _params.dig_t3);
    serial0_printf("dig_p1=%u\n", _params.dig_p1);
    serial0_printf("dig_p2=%d\n", _params.dig_p2);
    serial0_printf("dig_p3=%d\n", _params.dig_p3);
    serial0_printf("dig_p4=%d\n", _params.dig_p4);
    serial0_printf("dig_p5=%d\n", _params.dig_p5);
    serial0_printf("dig_p6=%d\n", _params.dig_p6);
    serial0_printf("dig_p7=%d\n", _params.dig_p7);
    serial0_printf("dig_p8=%d\n", _params.dig_p8);
    serial0_printf("dig_p9=%d\n", _params.dig_p9);

    i2c_read(0x8f, &buf[0], 1);
    i2c_read(0x8e, &buf[1], 1);
    serial0_printf("p1_msb=0x%.2x p1_lsb=0x%.2x\n", buf[0], buf[1]);

done:

    return;
}

bool Bmp280::isPresent(void) const
{
    return _present;
}

bool Bmp280::read(float &temperature, float &pressure)
{
    bool result = false;
    uint8_t buf[6];
    int32_t t, p;

    if (!_initialized || !_present) {
        result = false;
        temperature = 0.0;
        pressure = 0.0;
        goto done;
    }

    i2c_read(BME280_DATA_ADDR, buf, sizeof(buf));

    // Store the 20 bit read in a 32 bit signed integer for conversion
    p = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    t = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
    serial0_printf("p=%d, t=%d\n", p, t);

    temperature = ((float) convertTemp(t)) / 100.f;
    pressure = ((float) convertPressure(p, t)) / 1000.f;

    result = true;

done:

    return result;
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

    if (!_initialized) {
        result = false;
        goto done;
    }

    i2c_write_blocking(inst, _i2cAddr, &addr, 1, true);
    i2c_read_blocking(inst, _i2cAddr, data, len, false);
    result = true;

done:

    return result;
}

bool Bmp280::i2c_write(uint8_t addr, const uint8_t *data, size_t len)
{
    bool result = false;
    i2c_inst_t *inst = (i2c_inst_t *) _i2cPort;

    if (!_initialized) {
        result = false;
        goto done;
    }

    i2c_write_blocking(inst, _i2cAddr, &addr, 1, true);
    i2c_write_blocking(inst, _i2cAddr, data, len, false);

    result = true;

done:

    return result;
}


bool Bmp280::i2c_write(uint8_t addr, uint8_t data)
{
    bool result = false;
    i2c_inst_t *inst = (i2c_inst_t *) _i2cPort;
    uint8_t buf[2];

    if (!_initialized) {
        result = false;
        goto done;
    }

    buf[0] = addr;
    buf[1] = data;
    i2c_write_blocking(inst, _i2cAddr, buf, 2, false);

    result = true;

done:

    return result;
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
