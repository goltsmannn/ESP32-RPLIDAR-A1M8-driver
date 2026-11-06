#include "RPLidar.h"

void app_main(void) {
    uint8_t buffer[MIN_SCAN_STACK_SIZE];
    rplidar_init();
    rplidar_start_scanning(buffer, MIN_SCAN_STACK_SIZE);
}