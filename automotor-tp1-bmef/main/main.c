#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/timers.h"
#include "esp_adc/adc_oneshot.h"

#define MY_GPIO_LED             2
#define MY_GPIO_BUTTON          0   // Uses external pull-up resistor
#define MY_ADC1_CHAN_SPEED      ADC_CHANNEL_0


adc_oneshot_unit_handle_t adc1_handle;


esp_err_t init_gpios(void) {
    // Led GPIO
    gpio_reset_pin(MY_GPIO_LED);
    ESP_ERROR_CHECK( gpio_set_direction(MY_GPIO_LED, GPIO_MODE_OUTPUT) );
    gpio_set_level(MY_GPIO_LED, 0);
    
    // Button GPIO
    gpio_reset_pin(MY_GPIO_BUTTON);
    ESP_ERROR_CHECK( gpio_set_direction(MY_GPIO_BUTTON, GPIO_MODE_INPUT) );
    ESP_ERROR_CHECK( gpio_set_pull_mode(MY_GPIO_BUTTON, GPIO_FLOATING) ); 

    return ESP_OK;
}


esp_err_t init_adc(void) {
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK( adc_oneshot_new_unit(&init_config1, &adc1_handle) );

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK( adc_oneshot_config_channel(adc1_handle, MY_ADC1_CHAN_SPEED, &config) );

    return ESP_OK;
}


void app_main(void) {
    ESP_ERROR_CHECK( init_gpios() );
    ESP_ERROR_CHECK( init_adc() );

    while(1) {
        // if (gpio_get_level(GPIO_BUTTON) == 0) {
        //     gpio_set_level(MY_GPIO_LED, 1);
        // } else {
        //     gpio_set_level(MY_GPIO_LED, 0);
        // }
        // vTaskDelay(10 / portTICK_PERIOD_MS);

        int adc_raw;
        ESP_ERROR_CHECK( adc_oneshot_read(adc1_handle, MY_ADC1_CHAN_SPEED, &adc_raw) );
        printf("/*%d*/\r\n", adc_raw);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}
