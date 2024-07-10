#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"

static const char *TAG = "UART TEST";

#define TASKS_STACK_SIZE        2048

#define MY_UART_PORT_NUM       UART_NUM_1
#define MY_UART_BAUD_RATE      9600
#define MY_UART_BUFFER_SIZE    1024
#define MY_UART_TIMEOUT_MS     20

char rx_buffer[MY_UART_BUFFER_SIZE];
char tx_buffer[MY_UART_BUFFER_SIZE*2];


esp_err_t init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = MY_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(MY_UART_PORT_NUM, MY_UART_BUFFER_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MY_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(MY_UART_PORT_NUM, 10, 9, -1, -1));
    uart_set_rx_timeout(MY_UART_PORT_NUM, 1);   // read timeout en simbolos !! (sino responde 10ms despues)
    uart_set_line_inverse(MY_UART_PORT_NUM, UART_SIGNAL_TXD_INV);

    return ESP_OK;
}


// Check if PID is valid, returns ID if valid, -1 otherwise
int _lin_get_id(uint8_t pid) {
    // Ensure PID is within 8-bit range
    pid &= 0xFF;
    // Calculate P0: Parity of bits 0, 1, 2, and 4 (odd parity)
    uint8_t p0 = ((pid >> 0) & 1) ^ ((pid >> 1) & 1) ^ ((pid >> 2) & 1) ^ ((pid >> 4) & 1);
    // Calculate P1: Parity of bits 1, 3, 4, and 5 (odd parity)
    uint8_t p1 = ~(((pid >> 1) & 1) ^ ((pid >> 3) & 1) ^ ((pid >> 4) & 1) ^ ((pid >> 5) & 1)) & 1;
    // Check parity bits
    if ( (p0 == ((pid >> 6) & 1)) && (p1 == ((pid >> 7) & 1)) ) {
        return pid & 0x3F;
    }
    return -1;
}


int lin_receive() {
    size_t rx_len = 0;
    size_t data_len = 0;

    // Wait for something to be received
    while(1) {
        rx_len = uart_read_bytes(MY_UART_PORT_NUM, rx_buffer, 1, portMAX_DELAY);
        if (rx_len == 0) {
            continue;
        }
        data_len = 1;
        break;
    }

    // uart_get_buffered_data_len(MY_UART_PORT_NUM, &rx_len);
    // ESP_LOGI(TAG, "Received %d bytes", rx_len);

    // Get all rx buffer
    while (1) {
        rx_len = uart_read_bytes(MY_UART_PORT_NUM, &rx_buffer[data_len], 1, 0);
        if (rx_len == 0) {
            break;
        }
        data_len++;
    }

    // for (int i = 0; i < data_len; i++) {
    //     printf("%02X ", rx_buffer[i]);
    // }
    // printf("\n");

    // Check only last 2 bytes
    if (data_len < 2) {
        ESP_LOGE(TAG, "Not enough data received");
        return -1;
    }
    uint8_t word_sync = rx_buffer[data_len-2];
    uint8_t word_pid = rx_buffer[data_len-1];

    // Look for SyncField (0x55)
    if (word_sync != 0x55) {
        ESP_LOGE(TAG, "Invalid SyncField: 0x%02X", word_sync);
        return -1;
    }

    // Check if PID is valid
    int id = _lin_get_id(word_pid);
    if (id == -1) {
        ESP_LOGE(TAG, "Invalid PID: 0x%02X", word_pid);
        return -1;
    }
    return id;
}


// Calculate standard checksum (only data bytes)
uint8_t _lin_checksum(uint8_t *data, int len) {
    uint8_t checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum += data[i];
    }
    return ~checksum;
}


void lin_send_data(void *data, int len) {
    // Calculate checksum
    uint8_t checksum = _lin_checksum(data, len);
    // Send data
    uart_write_bytes(MY_UART_PORT_NUM, tx_buffer, len);
    uart_write_bytes(MY_UART_PORT_NUM, &checksum, 1);
    // clear rx buffer for echo
    uart_wait_tx_done(MY_UART_PORT_NUM, portMAX_DELAY);
    uart_flush(MY_UART_PORT_NUM);
}


void lin_handle_id(int id) {
    // Handle ID
    switch (id) {
        case 0x01:
            ESP_LOGI(TAG, "ID 0x01");
            sprintf(tx_buffer, "Hel");
            lin_send_data(tx_buffer, strlen(tx_buffer)+1);
            break;
        case 0x02:
            ESP_LOGI(TAG, "ID 0x02");
            sprintf(tx_buffer, "Goo");
            lin_send_data(tx_buffer, strlen(tx_buffer)+1);
            break;
        default:
            ESP_LOGW(TAG, "Unknown ID: 0x%02X", id);
            break;
    }
}


void vTaskUartRx(void *pvParameters) {
    while(1) {
        int id = lin_receive();
        if (id != -1) {
            lin_handle_id(id);
        }
    }
}


esp_err_t init_tasks(void) {
    ESP_LOGI(TAG, "Starting tasks");
    xTaskCreate( vTaskUartRx, "vTaskUartRx", TASKS_STACK_SIZE, NULL, configMAX_PRIORITIES-1, NULL );

    return ESP_OK;
}


void app_main(void) {
    ESP_ERROR_CHECK( init_uart() );
    
    ESP_ERROR_CHECK( init_tasks() );

    while(1) {
        vTaskSuspend(NULL); // Suspend main task
    }
}
