#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "int_i2c.h"

#define TAG "CONTADOR"

#define LED0 4
#define LED1 5
#define LED2 6
#define LED3 7
#define LED_PWM 1 // led pwm
#define BTN_INC 10 // botao incremento
#define BTN_DEC 11 // botao decremento
#define I2C_SDA 48 //led i2c
#define I2C_SCL 47 //led i2c
#define PWM_FREQ 1000
#define PWM_RESOLUTION LEDC_TIMER_8_BIT

// display lcd
lcd_i2c_handle_t lcd = {
    .address = 0x27,
    .num = I2C_NUM_0,
    .backlight = 1,
    .size = DISPLAY_16X02
};

volatile int contador = 0;
volatile int64_t ultimo_inc = 0;
volatile int64_t ultimo_dec = 0;
const int DEBOUNCE_US = 150000;

void atualizar_leds_binarios(int valor) {
    gpio_set_level(LED0, valor & 0x01);
    gpio_set_level(LED1, valor & 0x02);
    gpio_set_level(LED2, valor & 0x04);
    gpio_set_level(LED3, valor & 0x08);
}

// atualiza brilho pwm
void atualizar_pwm(int valor) {
    // valores entre 0 e 15 (4 bits)
    int duty = (valor * 255) / 15;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// atualiza led
void atualizar_lcd(int valor) {
    char linha1[17], linha2[17];
    lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);
    sprintf(linha1, "Hex: 0x%X", valor);
    sprintf(linha2, "Dec: %d", valor);
    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_print(&lcd, linha1);
    lcd_i2c_cursor_set(&lcd, 0, 1);
    lcd_i2c_print(&lcd, linha2);
}

// botao de incrementoo
static void IRAM_ATTR isr_inc(void *arg) {
    int64_t agora = esp_timer_get_time();
    if (agora - ultimo_inc > DEBOUNCE_US) {
        contador = (contador + 1) & 0x0F;
        ultimo_inc = agora;
        ets_printf("incrementou para %d (0x%X)\n", contador, contador);
    }
}

// botao de decremento
static void IRAM_ATTR isr_dec(void *arg) {
    int64_t agora = esp_timer_get_time();
    if (agora - ultimo_dec > DEBOUNCE_US) {
        contador = (contador - 1) & 0x0F; 
        ultimo_dec = agora;
        ets_printf("decrementou para %d (0x%X)\n", contador, contador);
    }
}

void app_main(void) {
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0);
    lcd_i2c_init(&lcd);
    vTaskDelay(pdMS_TO_TICKS(100));

    gpio_config_t leds_conf = {
        .pin_bit_mask = (1ULL << LED0) | (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&leds_conf);

    // led pwm
    ledc_timer_config_t pwm_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = LED_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&pwm_channel);

    // boboes
    gpio_config_t botoes = {
        .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_DEC),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&botoes);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_INC, isr_inc, NULL);
    gpio_isr_handler_add(BTN_DEC, isr_dec, NULL);

    int valor_anterior = -1;

    while (1) {
        if (valor_anterior != contador) {
            atualizar_leds_binarios(contador);
            atualizar_pwm(contador);
            atualizar_lcd(contador);
            valor_anterior = contador;
            ESP_LOGI(TAG, "Contador: %d (0x%X)", contador, contador);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
