#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "UART TEST";

#define TASKS_STACK_SIZE        2048

#define MY_UART_PORT_NUM       UART_NUM_1
#define MY_UART_BAUD_RATE      9600
#define MY_UART_BUFFER_SIZE    1024
#define MY_UART_TIMEOUT_MS     20

#define LIN_SYNC_PIN           GPIO_NUM_13
#define LIN_SYNC_DELAY_MS       2
#define LIN_MAX_DATA_SIZE       24

char rx_buffer[MY_UART_BUFFER_SIZE];
char tx_buffer[MY_UART_BUFFER_SIZE];


// Calculate standard checksum (only data bytes)
uint8_t _lin_checksum(uint8_t *data, int len) {
    uint8_t checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum += data[i];
    }
    return ~checksum;
}


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


void lin_sync_break(void) {
    // Send Sync Break
    gpio_set_level(LIN_SYNC_PIN, 1);
    vTaskDelay(LIN_SYNC_DELAY_MS / portTICK_PERIOD_MS);
    // Sync Break End (como hay buffer, cambio el gpio durante ele envio)
    uart_write_bytes(MY_UART_PORT_NUM, "\x00", 1);
    gpio_set_level(LIN_SYNC_PIN, 0);
}


esp_err_t lin_send(uint8_t id, uint8_t *data_buffer, int max_len) {
    uint8_t pid = _lin_calc_pid(id);

    lin_sync_break();
    uart_write_bytes(MY_UART_PORT_NUM, "\x55", 1); // Sync Byte
    uart_write_bytes(MY_UART_PORT_NUM, &pid, 1); // Sync Byte

    // clear rx buffer for echo
    uart_wait_tx_done(MY_UART_PORT_NUM, portMAX_DELAY);
    uart_flush(MY_UART_PORT_NUM);

    // Get response
    int len = uart_read_bytes(MY_UART_PORT_NUM, rx_buffer, MY_UART_BUFFER_SIZE, MY_UART_TIMEOUT_MS/portTICK_PERIOD_MS);
    if (len == 0) {
        return ESP_ERR_TIMEOUT;
    }
    else if (len == 1) {
        ESP_LOGE(TAG, "Missing data, only 1 byte received");
        return ESP_ERR_INVALID_RESPONSE;
    }
    else if (len > max_len) {
        ESP_LOGE(TAG, "Buffer overflow: %d", len);
        return ESP_ERR_NO_MEM;
    }

    // Copy data (avoiding checksum)
    for (int i = 0 ; i < len-1 ; i++) {
        data_buffer[i] = rx_buffer[i];
        printf("%02X ", rx_buffer[i]);
    }
    printf("\n");

    // Check checksum
    uint8_t checksum = rx_buffer[len-1];
    if ( checksum != _lin_checksum(data_buffer, len-1) ) {
        ESP_LOGE(TAG, "Invalid response checksum: %d", checksum);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}


esp_err_t init_uart(void) {
    // Config extra Sync PIN
    gpio_reset_pin(LIN_SYNC_PIN);
    gpio_set_direction(LIN_SYNC_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LIN_SYNC_PIN, 0);

    // Config UART
    uart_config_t uart_config = {
        .baud_rate = MY_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(MY_UART_PORT_NUM, MY_UART_BUFFER_SIZE, MY_UART_BUFFER_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MY_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(MY_UART_PORT_NUM, 10, 9, -1, -1));
    uart_set_rx_timeout(MY_UART_PORT_NUM, 1);   // read timeout en simbolos !! (sino responde 10ms despues)
    uart_set_line_inverse(MY_UART_PORT_NUM, UART_SIGNAL_TXD_INV);

    return ESP_OK;
}


void vTaskUartTx(void *pvParameters) {
    int counter = 0;
    uint8_t data_buffer[LIN_MAX_DATA_SIZE];
    while(1) {
        for ( counter = 0 ; counter < 5 ; counter++ ) {
            ESP_LOGI(TAG, "Sending id: %d", counter);
            //ESP_ERROR_CHECK( lin_send(counter, data_buffer, LIN_MAX_DATA_SIZE) );
            esp_err_t res = lin_send(counter, data_buffer, LIN_MAX_DATA_SIZE);
            if (res == ESP_OK) {
                ESP_LOGI(TAG, "Response: %s", data_buffer);
            }
            else {
                ESP_LOGE(TAG, "Error: %d", res);
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}


esp_err_t init_tasks(void) {
    ESP_LOGI(TAG, "Starting tasks");
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
