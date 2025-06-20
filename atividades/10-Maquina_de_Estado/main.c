

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/spi_common.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "int_i2c.h"

#define BOTAO_MAIS    10
#define BOTAO_MENOS   11
#define PINO_BUZZER   38
#define PINO_I2C_SDA  48
#define PINO_I2C_SCL  47
#define ADC_NTC       ADC1_CHANNEL_0

#define SEG_A 2
#define SEG_B 3
#define SEG_C 4
#define SEG_D 5
#define SEG_E 6
#define SEG_F 7
#define SEG_G 8


#define PIN_NUM_MISO  42
#define PIN_NUM_MOSI  41
#define PIN_NUM_CLK   40
#define PIN_NUM_CS    21
#define MOUNT_POINT   "/sdcard"

#define TEMPO_DEBOUNCE_US 200000
#define CONST_BETA    3950.0
#define VALOR_MAX_ADC 4095.0
#define TEMPERATURA_0K 298.15

static int temperatura_limite = 25;
static int temperatura_lida = 20;

volatile int64_t ultima_mais = 0;
volatile int64_t ultima_menos = 0;

lcd_i2c_handle_t lcd = {
    .address = 0x27,
    .num = I2C_NUM_0,
    .backlight = 1,
    .size = DISPLAY_16X02
};

const bool* get_estado_display(int diff, bool *piscar) {
    static const bool SEG0[7] = {1, 1, 1, 1, 1, 1, 0}; // 0
    static const bool SEG3[7] = {1, 0, 1, 1, 0, 1, 1}; // 3
    static const bool SEG7[7] = {1, 1, 1, 0, 0, 0, 1}; // 7
    static const bool SEGD[7] = {0, 1, 1, 1, 1, 1, 1}; // D
    static const bool SEGF[7] = {1, 1, 1, 1, 1, 1, 1}; // F
    static const bool SEG_OFF[7] = {0, 0, 0, 0, 0, 0, 0};

    *piscar = false;

    if (diff <= -1) {
        *piscar = true;
        return SEGF;
    } else if (diff <= 2) {
        return SEGD;
    } else if (diff <= 10) {
        return SEG7;
    } else if (diff <= 15) {
        return SEG3;
    } else if (diff <= 20) {
        return SEG0;
    } else {
        return SEG_OFF;
    }
}


void inicializar_sdcard(void) {
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) return;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t* card;
    esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    printf("SDCard montado com sucesso.\n");
}

float obter_temperatura(void) {
    int leitura_adc = adc1_get_raw(ADC_NTC);
    float resistencia = 10000.0 / ((VALOR_MAX_ADC / (float)leitura_adc) - 1.0);
    float temp_kelvin = 1.0 / (log(resistencia / 10000.0) / CONST_BETA + (1.0 / TEMPERATURA_0K));
    return temp_kelvin - 273.15;
}

void salvar_temperatura_sdcard(float temp) {
    FILE* f = fopen("/sdcard/teste.txt", "a");
    if (f) {
        fprintf(f, "Temperatura atual: %.2f\n", temp);
        fclose(f);
        printf("Temperatura salva no SDCard: %.2f\n", temp);
    } else {
        printf("Erro ao escrever no SDCard\n");
    }
}

void tarefa_sensor_ntc(void *pv) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_NTC, ADC_ATTEN_DB_12);
    while (1) {
        temperatura_lida = (int)obter_temperatura();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void tarefa_lcd(void *pv) {
    char l1[17], l2[17];
    static int ultima_temp = -1000;
    static int ultimo_limite = -1000;
    while (1) {
        if (temperatura_lida != ultima_temp || temperatura_limite != ultimo_limite) {
            lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);
            sprintf(l1, "Temp: %2d C", temperatura_lida);
            sprintf(l2, "Alarme: %2d C", temperatura_limite);
            lcd_i2c_cursor_set(&lcd, 0, 0);
            lcd_i2c_print(&lcd, l1);
            lcd_i2c_cursor_set(&lcd, 0, 1);
            lcd_i2c_print(&lcd, l2);
            ultima_temp = temperatura_lida;
            ultimo_limite = temperatura_limite;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void tarefa_sdcard(void *pv) {
    inicializar_sdcard();
    while (1) {
        salvar_temperatura_sdcard((float)temperatura_lida);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void configurar_buzzer(void) {
    ledc_timer_config_t timer_conf = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 2000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_conf = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = PINO_BUZZER,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&ch_conf);
}

void tarefa_alarme_buzzer(void *pv) {
    configurar_buzzer();
    while (1) {
        int duty = (temperatura_lida >= temperatura_limite) ? 128 : 0;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void IRAM_ATTR isr_mais(void* arg) {
    int64_t agora = esp_timer_get_time();
    if (agora - ultima_mais > TEMPO_DEBOUNCE_US) {
        temperatura_limite += 5;
        ultima_mais = agora;
    }
}

void IRAM_ATTR isr_menos(void* arg) {
    int64_t agora = esp_timer_get_time();
    if (agora - ultima_menos > TEMPO_DEBOUNCE_US) {
        temperatura_limite -= 5;
        ultima_menos = agora;
    }
}

void tarefa_botoes(void *pv) {
    gpio_config_t botoes = {
        .pin_bit_mask = (1ULL << BOTAO_MAIS) | (1ULL << BOTAO_MENOS),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&botoes);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(BOTAO_MAIS, isr_mais, NULL);
    gpio_isr_handler_add(BOTAO_MENOS, isr_menos, NULL);
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}

void tarefa_display_7segmentos(void *pv) {
    int pinos[7] = {SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G};
    for (int i = 0; i < 7; i++) {
        gpio_reset_pin(pinos[i]);
        gpio_set_direction(pinos[i], GPIO_MODE_OUTPUT);
    }
    while (1) {
        int diff = temperatura_limite - temperatura_lida;
        bool piscar;
        const bool* estado_segmentos = get_estado_display(diff, &piscar);
        bool mostrar = true;

        if (piscar) {
            mostrar = (esp_timer_get_time() / 500000) % 2;
        }

        for (int i = 0; i < 7; i++) {
            gpio_set_level(pinos[i], mostrar ? estado_segmentos[i] : 0);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void) {
    i2c_config_t conf_i2c = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PINO_I2C_SDA,
        .scl_io_num = PINO_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &conf_i2c);
    i2c_driver_install(I2C_NUM_0, conf_i2c.mode, 0, 0, 0);
    lcd_i2c_init(&lcd);
    lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);

    xTaskCreate(tarefa_sensor_ntc, "SensorNTC", 2048, NULL, 2, NULL);
    xTaskCreate(tarefa_lcd, "LCD", 2048, NULL, 1, NULL);
    xTaskCreate(tarefa_botoes, "Botoes", 2048, NULL, 2, NULL);
    xTaskCreate(tarefa_alarme_buzzer, "Alarme", 2048, NULL, 2, NULL);
    xTaskCreate(tarefa_sdcard, "SD", 4096, NULL, 1, NULL);
    xTaskCreate(tarefa_display_7segmentos, "Display7", 2048, NULL, 1, NULL);
}
