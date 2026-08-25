/*
 * Bmp280.hxx
 *
 * Copyright (C) 2025, Charles Chiou
 */

#ifndef BMP280_HXX
#define BMP280_HXX

#include <cstddef>
#include <cstdint>

class Bmp280 {

public:

    Bmp280(uint32_t i2cPort, uint32_t i2cSda, uint32_t i2cScl);
    ~Bmp280();

    bool isPresent(void) const;
    bool isInitialized(void) const;
    uint8_t i2cAddr(void) const;
    bool read(float &temperature, float &pressure);

private:

    void probe(void);
    bool softReset(void);
    bool wait_meas_done(void);
    double compensateTemp(int32_t raw_temp, double &t_fine);
    double compensatePressure(int32_t raw_press, double t_fine);
    bool i2c_read(uint8_t addr, uint8_t *data, size_t len);
    bool i2c_write(uint8_t addr, const uint8_t *data, size_t len);
    bool i2c_write(uint8_t addr, uint8_t data);

    void *_i2cPort;
    uint32_t _i2cSda;
    uint32_t _i2cScl;
    uint8_t _i2cAddr;
    bool _initialized;
    bool _present;
    struct calib_params {
        // temperature params
        uint16_t dig_t1;
        int16_t  dig_t2;
        int16_t  dig_t3;
        // pressure params
        uint16_t dig_p1;
        int16_t  dig_p2;
        int16_t  dig_p3;
        int16_t  dig_p4;
        int16_t  dig_p5;
        int16_t  dig_p6;
        int16_t  dig_p7;
        int16_t  dig_p8;
        int16_t  dig_p9;
    } _params;

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
