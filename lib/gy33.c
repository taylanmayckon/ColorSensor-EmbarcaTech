#include "gy33.h"

// Função para escrever um valor em um registro do GY-33
void gy33_write_register(uint8_t reg, uint8_t value){
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, GY33_I2C_ADDR, buffer, 2, false);
}

// Função para ler um valor de um registro do GY-33
uint16_t gy33_read_register(uint8_t reg){
    uint8_t buffer[2];
    i2c_write_blocking(I2C_PORT, GY33_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, GY33_I2C_ADDR, buffer, 2, false);
    return (buffer[1] << 8) | buffer[0]; // Combina os bytes em um valor de 16 bits
}

// Inicializa o GY-33
void gy33_init(){
    gy33_write_register(ENABLE_REG, 0x03);  // Ativa o sensor (Power ON e Ativação do ADC)
    gy33_write_register(ATIME_REG, 0xF5);   // Tempo de integração (ajusta a sensibilidade) D5 => 103ms
    gy33_write_register(CONTROL_REG, 0x00); // Configuração de ganho padrão (1x) (pode ir até 60x)
}

// Lê os valores RGB e Clear do GY-33
void gy33_read_color(gy33_color_t *gy33_data){
    gy33_data->c = gy33_read_register(CDATA_REG);
    gy33_data->r = gy33_read_register(RDATA_REG);
    gy33_data->g = gy33_read_register(GDATA_REG);
    gy33_data->b = gy33_read_register(BDATA_REG);
}