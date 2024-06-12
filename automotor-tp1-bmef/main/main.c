#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/timers.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"

#define MY_GPIO_LED             2
#define MY_GPIO_BUTTON          0   // Uses external pull-up resistor
#define MY_ADC1_CHAN_SPEED      ADC_CHANNEL_0
#define MY_ADC1_CHAN_BMEF       ADC_CHANNEL_3
#define MY_ADC1_CHAN_CURRENT    ADC_CHANNEL_6

#define MY_PWM_FREQ             1000

#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define TASKS_STACK_SIZE        2048


adc_oneshot_unit_handle_t adc1_handle;
uint32_t pwm_duty = 0;
bool pwm_enabled = false;
int adc_raw_speed, adc_raw_bmef, adc_raw_current;


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
    ESP_ERROR_CHECK( adc_oneshot_config_channel(adc1_handle, MY_ADC1_CHAN_BMEF, &config) );
    ESP_ERROR_CHECK( adc_oneshot_config_channel(adc1_handle, MY_ADC1_CHAN_CURRENT, &config) );

    return ESP_OK;
}


esp_err_t init_pwm(void) {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_12_BIT,  // TODO check right resolution
        .freq_hz          = MY_PWM_FREQ,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MY_GPIO_LED,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    
    return ESP_OK;
}


void vTaskReadInputs(void *pvParameters) {
    while(1) {
        // Read ADC
        ESP_ERROR_CHECK( adc_oneshot_read(adc1_handle, MY_ADC1_CHAN_SPEED, &adc_raw_speed) );
        // Button turn pwm on/off
        if (gpio_get_level(MY_GPIO_BUTTON) == 0) {
            if (pwm_enabled) {
                pwm_enabled = false;
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
            } else {
                pwm_enabled = true;
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, adc_raw_speed);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void vTaskDebug(void *pvParameters) {
    while(1) {
        printf("/*%d*/\n", adc_raw_speed);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}


TaskHandle_t xHandleReadInputs = NULL;
TaskHandle_t xHandleDebug = NULL;

esp_err_t init_tasks(void) {
    xTaskCreate( vTaskReadInputs, "vTaskReadInputs", TASKS_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleReadInputs );
    configASSERT( xHandleReadInputs );
    xTaskCreate( vTaskDebug, "vTaskDebug", TASKS_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandleDebug );
    configASSERT( xHandleDebug );

    return ESP_OK;
}


void app_main(void) {
    ESP_ERROR_CHECK( init_gpios() );
    ESP_ERROR_CHECK( init_adc() );
    ESP_ERROR_CHECK( init_pwm() );

    ESP_ERROR_CHECK( init_tasks() );

    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // vTaskSuspend(NULL); // Suspend main task
    }
}
