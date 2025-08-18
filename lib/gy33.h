#ifndef GY33_H
#define GY33_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1
#define GY33_I2C_ADDR 0x29 // Endereço do GY-33 no barramento I2C
// Registros do sensor
#define ENABLE_REG 0x80
#define ATIME_REG 0x81
#define CONTROL_REG 0x8F
#define ID_REG 0x92
#define STATUS_REG 0x93
#define CDATA_REG 0x94 //  "Clear"
#define RDATA_REG 0x96 //  "Red"
#define GDATA_REG 0x98 //  "Green"
#define BDATA_REG 0x9A //  "Blue"


typedef struct {
    uint16_t r; 
    uint16_t g;
    uint16_t b;
    uint16_t c; // Clear
} gy33_color_t;

void gy33_write_register(uint8_t reg, uint8_t value);
uint16_t gy33_read_register(uint8_t reg);
void gy33_init();
void gy33_read_color(gy33_color_t *gy33_data);

#endif