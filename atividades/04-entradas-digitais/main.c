#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static uint8_t contador = 0;
static uint8_t unid_incremento = 1; 

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

    // Configuração dos botões como entrada
    gpio_set_direction(36, GPIO_MODE_INPUT);
    gpio_set_direction(35, GPIO_MODE_INPUT);

    // Variáveis para debounce
    int last_estado_a = 0;
    int last_estado_b = 0;

    while (1) {
        // Leitura botao a
        int atual_estado_a = gpio_get_level(36);
        if (atual_estado_a == 1 && last_estado_a == 0) {
            // Incrementa o contador com a unidade de incremento atual
            contador = (contador + unid_incremento) % 16; // Circular entre 0x0 e 0xF
            atualizaled(contador);
        }
        last_estado_a = atual_estado_a;

        // Leitura botao b
        int atual_estado_b = gpio_get_level(35);
        if (atual_estado_b == 1 && last_estado_b == 0) {
            // Alterna a unidade de incremento entre 1 e 2
            unid_incremento = (unid_incremento == 1) ? 2 : 1;
        }
        last_estado_b = atual_estado_b;

        // correção do debounce quando aperta o botao (sem ele a saida sai toda errada)
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}