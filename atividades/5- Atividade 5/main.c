#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h" 

#define BOTAO_INC 36
#define BOTAO_MODO 35

#define DEBOUNCE_US 150000 

static uint8_t contador = 0;
static uint8_t unid_incremento = 1; 

// Armazena o tempo do último clique
int64_t ult_click_inc = 0;
int64_t ult_click_modo = 0;

void atualizaled(uint8_t value) {
    gpio_set_level(4, (value >> 0) & 1);
    gpio_set_level(5, (value >> 1) & 1);
    gpio_set_level(6, (value >> 2) & 1);
    gpio_set_level(7, (value >> 3) & 1);
}

void app_main(void) {
    // Configuração dos LEDs como saída
    gpio_set_direction(4, GPIO_MODE_OUTPUT);
    gpio_set_direction(5, GPIO_MODE_OUTPUT);
    gpio_set_direction(6, GPIO_MODE_OUTPUT);
    gpio_set_direction(7, GPIO_MODE_OUTPUT);

    // Configuração dos botões como entrada com pull-up (opcional se circuito tiver pull-up externo)
    gpio_set_direction(BOTAO_INC, GPIO_MODE_INPUT);
    gpio_set_direction(BOTAO_MODO, GPIO_MODE_INPUT);

    while (1) {
        int64_t agora = esp_timer_get_time();

        // Botão INC (GPIO 36)
        if (gpio_get_level(BOTAO_INC) == 1 && (agora - ult_click_inc > DEBOUNCE_US)) {
            contador = (contador + unid_incremento) % 16;
            atualizaled(contador);
            ult_click_inc = agora;
        }

        // Botão MODO (GPIO 35)
        if (gpio_get_level(BOTAO_MODO) == 1 && (agora - ult_click_modo > DEBOUNCE_US)) {
            unid_incremento = (unid_incremento == 1) ? 2 : 1;
            ult_click_modo = agora;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Pequeno delay para aliviar a CPU
    }
}
