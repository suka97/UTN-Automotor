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
#define MY_UART_TIMEOUT_MS     20

char rx_buffer[MY_UART_BUFFER_SIZE];
char tx_buffer[MY_UART_BUFFER_SIZE];


// Calculate LIN PID Parity bits
uint8_t _lin_calc_pid(uint8_t id) {
    // Ensure ID is within 6-bit range
    id &= 0x3F;
    // Calculate P0: Parity of bits 0, 1, 2, and 4 (odd parity)
    uint8_t p0 = ((id >> 0) & 1) ^ ((id >> 1) & 1) ^ ((id >> 2) & 1) ^ ((id >> 4) & 1);
    // Calculate P1: Parity of bits 1, 3, 4, and 5 (odd parity)
    uint8_t p1 = ~(((id >> 1) & 1) ^ ((id >> 3) & 1) ^ ((id >> 4) & 1) ^ ((id >> 5) & 1)) & 1;
    // Combine ID and parity bits into a PID
    uint8_t pid = (p0 << 6) | (p1 << 7) | id;
    return pid;
}


esp_err_t lin_send(uint8_t id) {
    uint8_t pid = _lin_calc_pid(id);

    // TODO: Implement SyncBreack with additional GPIO
    uart_write_bytes(MY_UART_PORT_NUM, "\x55", 1); // Sync Byte
    uart_write_bytes(MY_UART_PORT_NUM, &pid, 1); // Sync Byte

    return ESP_OK;
}


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

    ESP_ERROR_CHECK(uart_driver_install(MY_UART_PORT_NUM, MY_UART_BUFFER_SIZE, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(MY_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(MY_UART_PORT_NUM, 10, 9, -1, -1));

    return ESP_OK;
}


void vTaskUartRx(void *pvParameters) {
    while(1) {
        int len = uart_read_bytes(MY_UART_PORT_NUM, rx_buffer, MY_UART_BUFFER_SIZE, MY_UART_TIMEOUT_MS/portTICK_PERIOD_MS);
        if (len > 0) {
            ESP_LOGI(TAG, "Read %d bytes: %s", len, rx_buffer);
        }
    }
}


void vTaskUartTx(void *pvParameters) {
    int counter = 0;
    while(1) {
        for ( counter = 0 ; counter < 10 ; counter++ ) {
            ESP_LOGI(TAG, "Sending id: %d", counter);
            ESP_ERROR_CHECK( lin_send(counter) );

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}


esp_err_t init_tasks(void) {
    ESP_LOGI(TAG, "Starting tasks");
    xTaskCreate( vTaskUartTx, "vTaskUartTx", TASKS_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
    // xTaskCreate( vTaskUartRx, "vTaskUartRx", TASKS_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );

    return ESP_OK;
}


void app_main(void) {
    ESP_ERROR_CHECK( init_uart() );
    
    ESP_ERROR_CHECK( init_tasks() );

    while(1) {
        vTaskSuspend(NULL); // Suspend main task
    }
}
