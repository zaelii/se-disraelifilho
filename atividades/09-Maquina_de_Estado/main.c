#include <stdio.h>
#include <math.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/spi_common.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
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
#define ADC_NTC     ADC1_CHANNEL_0

#define PIN_NUM_MISO 42
#define PIN_NUM_MOSI 41
#define PIN_NUM_CLK  40
#define PIN_NUM_CS   21
#define MOUNT_POINT "/sdcard"

#define TEMPO_DEBOUNCE_US 200000
#define CONST_BETA      3950.0
#define VALOR_MAX_ADC   4095.0
#define TEMPERATURA_0K  298.15

static int temperatura_limite = 25;
static int temperatura_lida = 20;
static int ultima_temperatura_exibida = -1000;

volatile int64_t ultima_mais = 0;
volatile int64_t ultima_menos = 0;

lcd_i2c_handle_t lcd = {
    .address = 0x27,
    .num = I2C_NUM_0,
    .backlight = 1,
    .size = DISPLAY_16X02
};

typedef enum {
    ESTADO_INICIALIZAR,
    ESTADO_LER_SENSOR,
    ESTADO_VERIFICAR_ALARME,
    ESTADO_ATUALIZAR_SAIDAS,
    ESTADO_SALVAR_SD,
    ESTADO_EXIBIR_SD,
    ESTADO_ESPERAR
} estado_t;

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
    if (ret != ESP_OK) {
        printf("Falha ao inicializar SPI: %s\n", esp_err_to_name(ret));
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t* card;
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        printf("Erro ao montar SDCard: %s\n", esp_err_to_name(ret));
    } else {
        printf("SDCard montado com sucesso.\n");
    }
}

void ler_arquivo_sdcard(void) {
    FILE* f = fopen("/sdcard/teste.txt", "r");
    if (f != NULL) {
        char linha[64];
        printf("\n[Conteúdo de /sdcard/teste.txt]:\n");
        while (fgets(linha, sizeof(linha), f)) {
            printf("%s", linha);
        }
        fclose(f);
        printf("[Fim do arquivo]\n\n");
    } else {
        printf("Erro ao abrir o arquivo para leitura\n");
    }
}

void salvar_temperatura_sdcard(float temp) {
    FILE* f = fopen("/sdcard/teste.txt", "a");
    if (f != NULL) {
        fprintf(f, "Temperatura atual: %.2f\n", temp);
        fclose(f);
        printf("Temperatura salva no SDCard: %.2f\n", temp);
    } else {
        printf("Erro ao escrever no SDCard\n");
    }
}

float obter_temperatura(void) {
    int leitura_adc = adc1_get_raw(ADC_NTC);
    float resistencia = 10000.0 / ((VALOR_MAX_ADC / (float)leitura_adc) - 1.0);
    float temp_kelvin = 1.0 / (log(resistencia / 10000.0) / CONST_BETA + (1.0 / TEMPERATURA_0K));
    return temp_kelvin - 273.15;
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

void ativar_buzzer(void) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void desativar_buzzer(void) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void atualizar_lcd(void) {
    static int ultimo_alarme = -999;
    if (temperatura_lida != ultima_temperatura_exibida || temperatura_limite != ultimo_alarme) {
        char l1[17], l2[17];
        lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);
        sprintf(l1, "Temp: %2d C", temperatura_lida);
        sprintf(l2, "Alarme: %2d C", temperatura_limite);
        lcd_i2c_cursor_set(&lcd, 0, 0);
        lcd_i2c_print(&lcd, l1);
        lcd_i2c_cursor_set(&lcd, 0, 1);
        lcd_i2c_print(&lcd, l2);
        ultima_temperatura_exibida = temperatura_lida;
        ultimo_alarme = temperatura_limite;
    }
}

void atualizar_leds(int temp, int alarme, bool alarme_on) {
    bool piscar = alarme_on && ((esp_timer_get_time() / 500000) % 2);
    gpio_set_level(LED_0, alarme_on ? piscar : (temp >= alarme - 20));
    gpio_set_level(LED_1, alarme_on ? piscar : (temp >= alarme - 15));
    gpio_set_level(LED_2, alarme_on ? piscar : (temp >= alarme - 10));
    gpio_set_level(LED_3, alarme_on ? piscar : (temp >= alarme - 2));
}

void IRAM_ATTR isr_botao_mais(void* arg) {
    int64_t agora = esp_timer_get_time();
    if (agora - ultima_mais > TEMPO_DEBOUNCE_US) {
        temperatura_limite += 5;
        ultima_mais = agora;
    }
}

void IRAM_ATTR isr_botao_menos(void* arg) {
    int64_t agora = esp_timer_get_time();
    if (agora - ultima_menos > TEMPO_DEBOUNCE_US) {
        temperatura_limite -= 5;
        ultima_menos = agora;
    }
}

void app_main(void) {
    estado_t estado_atual = ESTADO_INICIALIZAR;

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
    vTaskDelay(pdMS_TO_TICKS(100));

    gpio_config_t leds = {
        .pin_bit_mask = (1ULL << LED_0) | (1ULL << LED_1) | (1ULL << LED_2) | (1ULL << LED_3),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&leds);

    gpio_config_t botoes = {
        .pin_bit_mask = (1ULL << BOTAO_MAIS) | (1ULL << BOTAO_MENOS),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&botoes);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(BOTAO_MAIS, isr_botao_mais, NULL);
    gpio_isr_handler_add(BOTAO_MENOS, isr_botao_menos, NULL);

    configurar_buzzer();
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_NTC, ADC_ATTEN_DB_12);
    inicializar_sdcard();

    while (1) {
        switch (estado_atual) {
            case ESTADO_INICIALIZAR:
                temperatura_lida = (int)obter_temperatura();
                estado_atual = ESTADO_LER_SENSOR;
                break;

            case ESTADO_LER_SENSOR:
                temperatura_lida = (int)obter_temperatura();
                estado_atual = ESTADO_VERIFICAR_ALARME;
                break;

            case ESTADO_VERIFICAR_ALARME:
                if (temperatura_lida >= temperatura_limite)
                    ativar_buzzer();
                else
                    desativar_buzzer();
                estado_atual = ESTADO_ATUALIZAR_SAIDAS;
                break;

            case ESTADO_ATUALIZAR_SAIDAS:
                atualizar_leds(temperatura_lida, temperatura_limite, temperatura_lida >= temperatura_limite);
                atualizar_lcd();
                estado_atual = ESTADO_SALVAR_SD;
                break;

            case ESTADO_SALVAR_SD:
                salvar_temperatura_sdcard(temperatura_lida);
                estado_atual = ESTADO_EXIBIR_SD;
                break;

            case ESTADO_EXIBIR_SD:
                ler_arquivo_sdcard();
                estado_atual = ESTADO_ESPERAR;
                break;

            case ESTADO_ESPERAR:
                vTaskDelay(pdMS_TO_TICKS(2000));
                estado_atual = ESTADO_LER_SENSOR;
                break;

            default:
                estado_atual = ESTADO_INICIALIZAR;
                break;
        }
    }
}
