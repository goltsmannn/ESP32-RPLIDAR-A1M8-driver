#include "RPLidar.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log_level.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "hal/uart_types.h"
#include "esp_log.h"

#define UART_RX_PIN GPIO_NUM_18
#define UART_TX_PIN GPIO_NUM_19
#define LIDAR_MOTOR_PIN GPIO_NUM_20
QueueHandle_t uart_queue;



uint8_t rplidar_init () {
    // configure 8N1 no RTS/DTS UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));

    // set pins for uart this will configure gpios automatically, enable RX queue, disable TX queue (blocking sends)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 512, 0, 10, &uart_queue, 0));

    uint8_t retry_count = 5;
    uint8_t get_health_success = 0;
    const uint8_t get_health_req[] = {START_REQUEST, REQUEST_COMMANDS[GET_HEALTH]};

    while (retry_count > 0) {
        // send get_health command
        int sent = uart_write_bytes(UART_NUM_1, get_health_req, sizeof(get_health_req));
        ESP_LOGI("RPLidar", "Attempting init, %d attempts left.", retry_count);

        if (sent != sizeof(get_health_req)) {
            ESP_LOGE("RPLIDAR", "UART write failed: %d/%d bytes", sent, sizeof(get_health_req));
        }

        retry_count--;

        // process get health response
        uint8_t health_response[10];
        memset(health_response, 0, sizeof(uint8_t) * 10);
        int success = uart_read_bytes(UART_NUM_1, health_response, 10, pdMS_TO_TICKS(5000));
        // process communication timeout
        if (success != 10) {
            ESP_LOGE("RPLIDAR", "Communication error at init");
            ESP_LOG_BUFFER_HEXDUMP("RPLIDAR", health_response, sizeof(health_response), ESP_LOG_ERROR);
            continue;
        }
        if (health_response[0] != START_REQUEST || health_response[1] != 0x5A || health_response[6] != 0x06) {
            ESP_LOGE("RPLIDAR", "Invalid response by GET_HEALTH: ");
            ESP_LOG_BUFFER_HEXDUMP("RPLIDAR", health_response, sizeof(health_response), ESP_LOG_ERROR);
            continue;
        }

        // process bringup status
        uint8_t status = health_response[7];
        uint16_t error = health_response[8] | (health_response[9] << 8);
        if (status == HEALTH_WARNING) {
            ESP_LOGW("RPLidar", "Warning by GET_HEALTH request, error code: %d", error);
        } else if (status == HEALTH_ERROR) {
            ESP_LOGE("RPLidar", "Error by GET_HEALTH request, deinitializing, error code: %d", error);
        } else {
            get_health_success = 1;
            break;
        }
    }

    if (get_health_success == 0) {
        rplidar_deinit();
        return false;
    }

    // spin motor
    gpio_set_direction(LIDAR_MOTOR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LIDAR_MOTOR_PIN, 1);
    return true;
}


void rplidar_tick() {

}

void rplidar_deinit() {
    uart_driver_delete(UART_NUM_1);
}