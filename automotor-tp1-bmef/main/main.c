#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/timers.h"

#define GPIO_LED        2
#define GPIO_BUTTON     0   // Uses external pull-up resistor


esp_err_t init_gpios(void) {
    // Led GPIO
    gpio_reset_pin(GPIO_LED);
    ESP_ERROR_CHECK( gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT) );
    gpio_set_level(GPIO_LED, 0);
    
    // Button GPIO
    gpio_reset_pin(GPIO_BUTTON);
    ESP_ERROR_CHECK( gpio_set_direction(GPIO_BUTTON, GPIO_MODE_INPUT) );
    ESP_ERROR_CHECK( gpio_set_pull_mode(GPIO_BUTTON, GPIO_FLOATING) ); 

    return ESP_OK;
}





void app_main(void) {
    ESP_ERROR_CHECK( init_gpios() );

    while(1) {
        if (gpio_get_level(GPIO_BUTTON) == 0) {
            gpio_set_level(GPIO_LED, 1);
        } else {
            gpio_set_level(GPIO_LED, 0);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
