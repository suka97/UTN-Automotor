#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"

static const char *TAG = "UART TEST";

#define TASKS_STACK_SIZE        2048

#define MY_UART_PORT_NUM       UART_NUM_1
#define MY_UART_BAUD_RATE      115200
#define MY_UART_BUFFER_SIZE    1024


esp_err_t init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = MY_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(MY_UART_PORT_NUM, MY_UART_BUFFER_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(MY_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(MY_UART_PORT_NUM, 10, 9, -1, -1));

    return ESP_OK;
}


void vTaskUartRx(void *pvParameters) {
    uint8_t *data = (uint8_t *) malloc(MY_UART_BUFFER_SIZE);
    while(1) {
        int len = uart_read_bytes(MY_UART_PORT_NUM, data, MY_UART_BUFFER_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            ESP_LOGI(TAG, "Read %d bytes: %s", len, data);
        }
    }
    free(data);
    vTaskDelete(NULL);
}


void vTaskUartTx(void *pvParameters) {
    int counter = 0;
    const uint8_t buffersize = 32;
    char data[buffersize];
    while(1) {
        ESP_LOGI(TAG, "Sending counter: %d", counter);
        sprintf(data, "Counter: %d\n", counter++);
        uart_write_bytes(MY_UART_PORT_NUM, data, buffersize);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


esp_err_t init_tasks(void) {
    ESP_LOGI(TAG, "Starting tasks");
    // xTaskCreate( vTaskUartRx, "vTaskUartRx", TASKS_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
    xTaskCreate( vTaskUartTx, "vTaskUartTx", TASKS_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );

    return ESP_OK;
}


void app_main(void) {
    ESP_ERROR_CHECK( init_uart() );
    
    ESP_ERROR_CHECK( init_tasks() );

    while(1) {
        vTaskSuspend(NULL); // Suspend main task
    }
}
