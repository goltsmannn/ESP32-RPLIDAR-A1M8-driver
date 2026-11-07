#include "RPLidar.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log_level.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "hal/uart_types.h"
#include "esp_log.h"
#include "portmacro.h"

#define UART_RX_PIN GPIO_NUM_19
#define UART_TX_PIN GPIO_NUM_18
#define LIDAR_MOTOR_PIN GPIO_NUM_20

QueueHandle_t uart_queue;
static TaskHandle_t uart_scan_task_handle_ = NULL;
// this will be returned to user
static size_t prev_scan_begin_index = 0;
static size_t prev_scan_end_index = 0;
// this scan will be written to
static size_t cur_scan_begin_index = 0;
static size_t cur_scan_end_index = 0;
ScanResponse* uart_buffer;
size_t uart_buffer_size;


uint8_t rplidar_init() {
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
    ESP_LOGI("RPLidar", "Initialization succeeded; motor enabled");
    return true;
}


uint8_t rplidar_start_scanning(void* buffer, size_t size) {
    // check that buffer for scan periodic task is big enough
    if (size < MIN_SCAN_STACK_SIZE) {
        return false;
    }

    // send scan request
    const uint8_t scan_req[] = {START_REQUEST, REQUEST_COMMANDS[SCAN]};
    int sent = uart_write_bytes(UART_NUM_1, scan_req, sizeof(scan_req));
    if (sent != sizeof(scan_req)) {
        return false;
    }

    // process descriptor
    const uint8_t response_correct_descriptor[] = {START_REQUEST, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};
    uint8_t response_descriptor[7];
    int descriptor_success = uart_read_bytes(UART_NUM_1, response_descriptor, 7, pdMS_TO_TICKS(10));
    if (descriptor_success != 7 || memcmp(response_correct_descriptor, response_descriptor, 7) != 0) {
        ESP_LOGE("RPLidar", "Error sending scan request");
        ESP_LOG_BUFFER_HEXDUMP("RPLIDAR", response_descriptor, sizeof(response_descriptor), ESP_LOG_ERROR);
        return false;
    }
    
    // if descriptor is correct, start periodic scan processing
    uart_buffer = buffer;
    uart_buffer_size = size;
    xQueueReset(uart_queue); // drain the stale events from queue
    BaseType_t result = xTaskCreate(rplidar_scan_periodic, "RPLidar Periodic Scan Processing", MIN_SCAN_STACK_SIZE, NULL, 1, &uart_scan_task_handle_);
    if (result != pdPASS) {
        return false;
    }

    return true;
}

void rplidar_scan_periodic(void *arg) {
    (void) arg;

    uart_event_t uart_event;
    size_t uart_buffer_index = 0;
    ScanResponse response = {};
    ScanResponsePayload_e current_byte = BYTE1_QUALITY; // data is not guaranteed to come in one transaction
    uint8_t raw_buffer[20]; // for raw data

    while(true) {
        if(xQueueReceive(uart_queue, &uart_event, portMAX_DELAY) == pdTRUE) {
            switch (uart_event.type) {
                case UART_DATA:
                    int response_length = uart_read_bytes(UART_NUM_1, raw_buffer, sizeof(raw_buffer), 0);
                    for (int i = 0; i < response_length; ++i) {
                        switch (current_byte) {
                            // no descriptor so read straight from 0-index of the raw_buffer
                            case BYTE1_QUALITY:
                                response.quality = raw_buffer[i]; 
                                break;
                            case BYTE2_ANGLE_LSB:
                                response.angle = raw_buffer[i] & 0xff;
                                break;
                            case BYTE3_ANGLE_MSB:
                                response.angle |= (raw_buffer[i] << 8);
                                break;
                            case BYTE4_RANGE_LSB:
                                response.range = raw_buffer[i] & 0xff;
                                break;
                            case BYTE5_RANGE_MSG:
                                response.range |= (raw_buffer[i] << 8);
                                uart_buffer[uart_buffer_index] = response;
                                uart_buffer_index = (uart_buffer_index + 1) % uart_buffer_size;
                                memset(&response, 0, sizeof(response));
                                break;     
                            default:
                                break;
                        }
                    }
                    break;
                default:
                    ESP_LOGE("RPLidar", "Error in periodic scan receive. UART event type: %d ", (uint8_t) uart_event.type);
                    break;
            }
        }

    }
}


void rplidar_deinit() {
    if (uart_scan_task_handle_ != NULL) {
        vTaskDelete(uart_scan_task_handle_);
    }

    const uint8_t stop_req[] = {START_REQUEST, REQUEST_COMMANDS[STOP]};
    int retry_count = 3;

    while (retry_count > 0) {
        // Send STOP before removing the UART driver.
        int sent = uart_write_bytes(UART_NUM_1, stop_req, sizeof(stop_req));
        ESP_LOGI("RPLidar", "Attempting stop, %d attempts left.", retry_count);
        retry_count--;

        if (sent != sizeof(stop_req)) {
            ESP_LOGE("RPLidar", "UART write failed: %d/%d bytes", sent, sizeof(stop_req));
            continue;
        }
        esp_err_t err = uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(10));
        if (err != ESP_OK) {
            ESP_LOGE("RPLidar", "STOP transmission failed: %s", esp_err_to_name(err));
            continue;
        }
        break;
    }

    gpio_set_level(LIDAR_MOTOR_PIN, 0);
    ESP_ERROR_CHECK(uart_driver_delete(UART_NUM_1));
    uart_queue = NULL;
}
