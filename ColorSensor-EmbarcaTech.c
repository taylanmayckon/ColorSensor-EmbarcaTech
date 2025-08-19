#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "ssd1306.h"
#include "font.h"
#include "led_matrix.h"
#include "gy33.h"
#include "bh1750.h"

// GPIO utilizada
#define LED_RED 13
#define LED_GREEN 11
#define LED_BLUE 12
#define LED_MATRIX_PIN 7 
#define BUTTON_A 5
#define BUTTON_B 6
#define BUZZER_A 21
#define BUZZER_B 10
// Para a matriz de leds
#define IS_RGBW false

// Configurações da I2C do display
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define endereco 0x3C
bool cor = true;
ssd1306_t ssd;

// Configurações da I2C dos sensores
#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1

// Estrutura para armazenar os dados dos sensores
gy33_color_t gy33_data;
uint16_t lux_level;

// Estrutura para os alertas
typedef struct {
    int lux_threshold; // Limite de luminosidade
    int color_threshold; // Limite de cor
} alerts_t;

// Configurações para o PWM
uint wrap = 2000;
uint clkdiv = 25;
// Variáveis da PIO declaradas no escopo global
PIO pio;
uint sm;

int led_state = 0;

// Maximos observados nos sensores
#define MAX_LUX 20


// -> Funções Auxiliares =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Função para configurar o PWM e iniciar com 0% de DC
void set_pwm(uint gpio, uint wrap){
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(slice_num, clkdiv);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_enabled(slice_num, true); 
    pwm_set_gpio_level(gpio, 0);
}


// -> ISR dos Botões =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Tratamento de interrupções 
uint32_t last_isr_time = 0;
void gpio_irq_handler(uint gpio, uint32_t events){
    uint32_t current_isr_time = to_us_since_boot(get_absolute_time());
    if(current_isr_time-last_isr_time > 200000){ // Debounce
        last_isr_time = current_isr_time;
        
        if(gpio==BUTTON_A) {
            led_state--;
            if(led_state < 0) led_state = 5;
        }
        else{
            led_state++;
            if(led_state > 5) led_state = 0;
        }
    }
}


int main(){
    stdio_init_all();

     // Iniciando o display
    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Iniciando o I2C dos sensores
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Iniciando a matriz de leds
    pio = pio0;
    sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, LED_MATRIX_PIN, 800000, IS_RGBW);

    // Iniciando os buzzers
    set_pwm(BUZZER_A, wrap);
    set_pwm(BUZZER_B, wrap);

    // Inicializando os LEDs
    gpio_init(LED_RED);
    gpio_init(LED_GREEN);
    gpio_init(LED_BLUE);

    gpio_set_dir(LED_RED, GPIO_OUT);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    gpio_set_dir(LED_BLUE, GPIO_OUT);

    alerts_t alerts = {
        .lux_threshold = 5, // Limite de luminosidade (LUX)
        .color_threshold = 150  // Limite de cor 
    };

    // Iniciando os botões
    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);
    gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);
    gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Inicializando o sensor de cor
    gy33_init();

    while (true) {
        lux_level = bh1750_read_measurement(I2C_PORT);
        gy33_read_color(&gy33_data);
        // Normmalizando os valores de cor e intensidade
        Led_color color;
        color.blue = (uint8_t)((gy33_data.b * 255) / gy33_data.c); 
        color.green = (uint8_t)((gy33_data.g * 255) / gy33_data.c);
        color.red = (uint8_t)((gy33_data.r * 255) / gy33_data.c);

        float led_intensity = (float)lux_level / MAX_LUX; 

        printf("Lux = %d\n", lux_level);
        printf("Intensidade Luz = %.2f\n", led_intensity);
        printf("Cor detectada - R: %d, G: %d, B: %d, Clear: %d\n", gy33_data.r, gy33_data.g, gy33_data.b, gy33_data.c);
        printf("Cor MATRIZ - R: %d, G: %d, B: %d, Clear: %d\n", color.red, color.green, color.blue);

        // Enviando a cor para a matriz de LEDs
        fill_matrix(color, led_intensity);

        // Geraçao de cores para o sensor
        switch(led_state){
            case 0:
                gpio_put(LED_RED, 1);
                gpio_put(LED_GREEN, 0);
                gpio_put(LED_BLUE, 0);
                break;
            case 1:
                gpio_put(LED_RED, 1);
                gpio_put(LED_GREEN, 1);
                gpio_put(LED_BLUE, 0);
                break;
            case 2:
                gpio_put(LED_RED, 0);
                gpio_put(LED_GREEN, 1);
                gpio_put(LED_BLUE, 0);
                break;
            case 3:
                gpio_put(LED_RED, 0);
                gpio_put(LED_GREEN, 1);
                gpio_put(LED_BLUE, 1);
                break;
            case 4:
                gpio_put(LED_RED, 0);
                gpio_put(LED_GREEN, 0);
                gpio_put(LED_BLUE, 1);
                break;
            case 5:
                gpio_put(LED_RED, 1);
                gpio_put(LED_GREEN, 0);
                gpio_put(LED_BLUE, 1);
                break;
        }

        char str_lux[20];
        char str_blue[20];
        char str_green[20];
        char str_red[20];
        snprintf(str_lux, sizeof(str_lux), "Lux: %d", lux_level);
        snprintf(str_blue, sizeof(str_blue), "B: %d", color.blue);
        snprintf(str_green, sizeof(str_green), "G: %d", color.green);
        snprintf(str_red, sizeof(str_red), "R: %d", color.red);

        // Atualizando o display
        ssd1306_fill(&ssd, false);
        ssd1306_rect(&ssd, 0, 0, 128, 64, cor, !cor);
        // Linha 1 - Alerta 
        ssd1306_rect(&ssd, 0, 0, 128, 12, cor, cor); // Fundo do cabeçalho

        ssd1306_draw_string(&ssd, str_red, 4, 14, false);
        ssd1306_draw_string(&ssd, str_green, 4, 26, false);
        ssd1306_draw_string(&ssd, str_blue, 4, 38, false);
        ssd1306_draw_string(&ssd, str_lux, 4, 50, false);

        ssd1306_line(&ssd, 68, 0, 68, 63, cor);

        ssd1306_draw_string(&ssd, "COR", 87, 26, false);
        if(color.blue>color.red && color.blue>color.green) ssd1306_draw_string(&ssd, "B", 95, 38, false);
        else if(color.red>color.blue && color.red>color.green) ssd1306_draw_string(&ssd, "R", 95, 38, false);
        else if(color.green>color.red && color.green>color.blue) ssd1306_draw_string(&ssd, "G", 95, 38, false);
        else ssd1306_draw_string(&ssd, "-", 95, 38, false);

        // Alertas
        if(lux_level < alerts.lux_threshold || color.blue > alerts.color_threshold){
            // Linha 1 - Alerta 
            ssd1306_draw_string(&ssd, "ALERTA!", 4, 3, true);
            pwm_set_gpio_level(BUZZER_A, wrap*0.02);
            pwm_set_gpio_level(BUZZER_B, wrap*0.02);
            sleep_ms(50);
            pwm_set_gpio_level(BUZZER_A, 0);
            pwm_set_gpio_level(BUZZER_B, 0);
        }
        else{
            // Linha 1 - Alerta 
            ssd1306_draw_string(&ssd, "NORMAL", 4, 3, true);
            pwm_set_gpio_level(BUZZER_A, 0);
            pwm_set_gpio_level(BUZZER_B, 0);
        }

        
        ssd1306_send_data(&ssd);

        sleep_ms(50);
    }
}
