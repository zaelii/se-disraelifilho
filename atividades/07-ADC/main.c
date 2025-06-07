#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "int_i2c.h"

#define LED_0       4
#define LED_1       5
#define LED_2       6
#define LED_3       7
#define BOTAO_MAIS  10
#define BOTAO_MENOS 11
#define PINO_BUZZER 38
#define PINO_I2C_SDA 48
#define PINO_I2C_SCL 47
#define CANAL_SENSOR_ADC ADC1_CHANNEL_0
#define TEMPO_DEBOUNCE_US 20000

#define CONST_BETA      3950.0
#define VALOR_MAX_ADC   4095.0
#define TEMPERATURA_0K  298.15  // 25 °C em Kelvin

static int temperatura_limite = 25;
static int temperatura_lida = 20;
static int ultima_temperatura_exibida = -1000;

lcd_i2c_handle_t lcd = {
    .address = 0x27,
    .num = I2C_NUM_0,
    .backlight = 1,
    .size = DISPLAY_16X02
};

// Configura o PWM para o buzzer
static void configurar_buzzer(void) {
    ledc_timer_config_t config_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 2000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&config_timer);

    ledc_channel_config_t config_canal = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = PINO_BUZZER,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&config_canal);
}

static void ativar_buzzer(void) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void desativar_buzzer(void) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// Converte a leitura do sensor em graus Celsius
static float obter_temperatura(void) {
    int leitura_adc = adc1_get_raw(CANAL_SENSOR_ADC);
    float resistencia = 10000.0 / ((VALOR_MAX_ADC / (float)leitura_adc) - 1.0);
    float temp_kelvin = 1.0 / (log(resistencia / 10000.0) / CONST_BETA + (1.0 / TEMPERATURA_0K));
    return temp_kelvin - 273.15;
}

// Atualiza o display LCD se houver mudança
static void atualizar_lcd(void) {
    static int ultimo_valor_alarme = -999;

    if (temperatura_lida != ultima_temperatura_exibida || temperatura_limite != ultimo_valor_alarme) {
        char linha1[17], linha2[17];

        lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);
        sprintf(linha1, "Alarme: %2d C", temperatura_limite);
        sprintf(linha2, "Temp:   %2d C", temperatura_lida);

        lcd_i2c_cursor_set(&lcd, 0, 0);
        lcd_i2c_print(&lcd, linha1);
        lcd_i2c_cursor_set(&lcd, 0, 1);
        lcd_i2c_print(&lcd, linha2);

        ultima_temperatura_exibida = temperatura_lida;
        ultimo_valor_alarme = temperatura_limite;
    }
}

// Atualiza os LEDs com base na temperatura
static void atualizar_leds(int temperatura, int alarme, bool em_alarme) {
    bool piscar = em_alarme && ((esp_timer_get_time() / 500000) % 2);

    gpio_set_level(LED_0, em_alarme ? piscar : (temperatura <= alarme - 2));
    gpio_set_level(LED_1, em_alarme ? piscar : (temperatura <= alarme - 10));
    gpio_set_level(LED_2, em_alarme ? piscar : (temperatura <= alarme - 15));
    gpio_set_level(LED_3, em_alarme ? piscar : (temperatura <= alarme - 20));
}

// Faz a leitura dos botões com debounce
static void ler_botoes(void) {
    static int estado_antigo_mais = 1, estado_antigo_menos = 1;
    static int64_t tempo_ultimo_mais = 0, tempo_ultimo_menos = 0;
    int64_t agora = esp_timer_get_time();

    int estado_atual_mais = gpio_get_level(BOTAO_MAIS);
    int estado_atual_menos = gpio_get_level(BOTAO_MENOS);

    if (estado_antigo_mais == 1 && estado_atual_mais == 0 && (agora - tempo_ultimo_mais) > TEMPO_DEBOUNCE_US) {
        temperatura_limite += 5;
        tempo_ultimo_mais = agora;
    }

    if (estado_antigo_menos == 1 && estado_atual_menos == 0 && (agora - tempo_ultimo_menos) > TEMPO_DEBOUNCE_US) {
        temperatura_limite -= 5;
        tempo_ultimo_menos = agora;
    }

    estado_antigo_mais = estado_atual_mais;
    estado_antigo_menos = estado_atual_menos;
}

void app_main(void) {
    // Inicialização do barramento I2C
    i2c_config_t config_i2c = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PINO_I2C_SDA,
        .scl_io_num = PINO_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &config_i2c);
    i2c_driver_install(I2C_NUM_0, config_i2c.mode, 0, 0, 0);
    lcd_i2c_init(&lcd);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configura os LEDs
    gpio_config_t config_leds = {
        .pin_bit_mask = (1ULL << LED_0) | (1ULL << LED_1) |
                        (1ULL << LED_2) | (1ULL << LED_3),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&config_leds);

    // Configura os botões com pull-up
    gpio_config_t config_botoes = {
        .pin_bit_mask = (1ULL << BOTAO_MAIS) | (1ULL << BOTAO_MENOS),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1
    };
    gpio_config(&config_botoes);

    configurar_buzzer();

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(CANAL_SENSOR_ADC, ADC_ATTEN_DB_11);

    while (true) {
        ler_botoes();

        temperatura_lida = (int)obter_temperatura();
        bool alarme_disparado = temperatura_lida >= temperatura_limite;

        if (alarme_disparado) ativar_buzzer();
        else desativar_buzzer();

        atualizar_leds(temperatura_lida, temperatura_limite, alarme_disparado);
        atualizar_lcd();

        printf("ALARME: %d C | TEMP: %d C\n", temperatura_limite, temperatura_lida);

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}
