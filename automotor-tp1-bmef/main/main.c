#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/timers.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"

static const char *TAG = "main";

#define MY_GPIO_LED             2
#define MY_GPIO_BUTTON          0   // Uses external pull-up resistor
#define MY_GPIO_PCNT_SPEED      23
#define MY_ADC1_CHAN_SPEED      ADC_CHANNEL_0
#define MY_ADC1_CHAN_BMEF       ADC_CHANNEL_3
#define MY_ADC1_CHAN_CURRENT    ADC_CHANNEL_6

#define MY_PWM_FREQ             1000
#define MY_READ_INTERVAL_MS     10

#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define PCNT_HIGH_LIMIT         10
#define TASKS_STACK_SIZE        2048


adc_oneshot_unit_handle_t adc1_handle;
uint32_t pwm_duty = 0;
bool pwm_enabled = false;
int adc_raw_speed, adc_raw_bmef, adc_raw_current;

pcnt_unit_handle_t pcnt_unit = NULL;
QueueHandle_t pcnt_queue = NULL;
int pcnt_freq = 0.0;


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


static bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    static int64_t last_time = 0;
    int time_delta = esp_timer_get_time() - last_time;
    last_time = esp_timer_get_time();

    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &time_delta, &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}


esp_err_t init_pcnt(void) {
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = -1,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channel");
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = MY_GPIO_PCNT_SPEED,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_EDGE_ACTION_INCREASE));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, 0));
    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach,
    };
    pcnt_queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, pcnt_queue));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    return ESP_OK;
}


// Usar este llamado en lugar del simple xQueueReceive
// Permite refresscar la frecuencia, para poder detectar freq = 0
void pcnt_get_freq(void) {
    static int no_change_count = 0;
    if (xQueueReceive(pcnt_queue, &pcnt_freq, 0)) {
        no_change_count = 0;
        pcnt_freq = PCNT_HIGH_LIMIT * 500000 / pcnt_freq;   // 2 pulses per revolution
    }
    else {
        no_change_count++;
        if (no_change_count > (1000/MY_READ_INTERVAL_MS)) pcnt_freq = 0;
    }
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
            }
        }
        // Update PWM duty
        if (pwm_enabled) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, adc_raw_speed);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
        }
        // Calculate PCNT frequency
        pcnt_get_freq();

        vTaskDelay(MY_READ_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}


void vTaskDebug(void *pvParameters) {
    while(1) {
        printf("/*");

        printf("%d,", adc_raw_speed);
        printf("%d,", pcnt_freq);

        printf("*/\n");
        vTaskDelay(200 / portTICK_PERIOD_MS);
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
    ESP_ERROR_CHECK( init_pcnt() );

    ESP_ERROR_CHECK( init_tasks() );

    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // vTaskSuspend(NULL); // Suspend main task
    }
}
