#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"


 
void app_main() {
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << 25) | (1ULL << 27);
    gpio_config(&io_conf);

    int estado_led25 = 0;
    int estado_led27 = 0;
    int time_led25 = 0;
    int time_led27 = 0;

    while(1) {
      if (time_led25 >= 1000) {
        estado_led25 = !estado_led25;
        gpio_set_level(GPIO_NUM_25, estado_led25);
        time_led25 = 0;
      }
      if (time_led27 >= 200) {
        estado_led27 = !estado_led27;
        gpio_set_level(GPIO_NUM_27, estado_led27);
        time_led27 = 0;
      }
      
      vTaskDelay(10 / portTICK_PERIOD_MS);
      time_led25 += 10;
      time_led27 += 10;
    }
}
