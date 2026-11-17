#include "sensors.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0

static void wybierz_kanal(uint8_t k) {
    uint8_t b = (uint8_t)(1u << k);
    i2c_write_blocking(I2C_PORT, TCA_ADDR, &b, 1, false);
    sleep_us(300);
}

static bool zapisz_tlv(uint8_t rej, uint8_t val) {
    uint8_t buf[2] = {rej, val};
    return i2c_write_blocking(I2C_PORT, TLV_ADDR, buf, 2, false) == 2;
}

static bool odczyt_kanalu(uint8_t k, double *x, double *y, double *z) {
    wybierz_kanal(k);

    if (!zapisz_tlv(0x03, 0x01)) return false;
    sleep_ms(8);                        // konwersja TLV493D (~3.5 ms + margines)

    uint8_t rej = 0x00;
    if (i2c_write_blocking(I2C_PORT, TLV_ADDR, &rej, 1, true) != 1) return false;

    uint8_t b[7];
    if (i2c_read_blocking(I2C_PORT, TLV_ADDR, b, 7, false) != 7) return false;

    int16_t xr = (int16_t)(((uint16_t)(b[0] & 0x0F) << 8) | b[4]);
    int16_t yr = (int16_t)(((uint16_t)(b[1] & 0x0F) << 8) | b[5]);
    int16_t zr = (int16_t)(((uint16_t)(b[2] & 0x0F) << 8) | b[6]);

    if (xr > 2047) xr -= 4096;
    if (yr > 2047) yr -= 4096;
    if (zr > 2047) zr -= 4096;

    *x = xr * 0.098;
    *y = yr * 0.098;
    *z = zr * 0.098;
    return true;
}

bool sensors_init(void) {
    i2c_init(I2C_PORT, 50000);          // 50 kHz — sprawdzone jako stabilne
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    sleep_ms(50);

    for (uint8_t k = 0; k < 3; k++) {
        wybierz_kanal(k);
        sleep_ms(20);
        if (!zapisz_tlv(0x03, 0x00)) return false;
        sleep_ms(20);
    }
    return true;
}

bool sensors_read9(double v[9]) {
    uint8_t udane = 0;
    for (uint8_t k = 0; k < 3; k++) {
        if (odczyt_kanalu(k, &v[k*3+0], &v[k*3+1], &v[k*3+2])) udane++;
    }
    return (udane == 3);
}
