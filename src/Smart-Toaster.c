#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "lcd.h"

#include <stdint.h>

// SPI Defines
#define SPI_PORT spi1
#define PIN_MISO 12
#define PIN_CS   13
#define PIN_SCK  10
#define PIN_MOSI 11

// I2C defines
#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5

#define PIN_RELAY 7

float read_temp() {
    uint8_t buffer[2];

    gpio_put(PIN_CS, 0);
    sleep_ms(10);
    spi_read_blocking(SPI_PORT, 0, buffer, 2);
    sleep_ms(10);
    gpio_put(PIN_CS, 1);

    printf("Data: %d %d\n", buffer[0], buffer[1]);
    uint16_t raw = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    raw = raw >> 3;
    return (float)raw * 0.25f;
}

int main() {
    stdio_init_all();

    // SPI initialisation at 4MHz, 8 bits, mode 0 (CPOL=0, CPHA=0)
    spi_init(SPI_PORT, 4000000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // I2C Initialisation. Using it at 100Khz.
    i2c_init(I2C_PORT, 100000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    while (true) {
        printf("Hello, world! Temp is %f\n", read_temp());
        sayHello();
        sleep_ms(1000);
    }
}
