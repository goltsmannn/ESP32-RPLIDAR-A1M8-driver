#pragma once

#include <stdint.h>

#define PARSE_ANGLE(angle) (angle / 64.0f)
#define PARSE_RANGE(range) (range / 4.0)
#define START_REQUEST (0xA5)
#define MAX_SAMPLING_FREQUENCY (4000)
#define MAX_SCANS_PER_REVOLUTION (MAX_SAMPLING_FREQUENCY)
#define MIN_SCAN_STACK_SIZE 2 * (MAX_SAMPLING_FREQUENCY * sizeof(ScanResponse) + 100) // multiply by 2 so we can store two full scans as we might need to return while the other one is not full yet


_Static_assert(sizeof(float) == 4,  "float must be 4 bytes");


typedef struct {
    uint8_t command;
    uint16_t payload;
} Request ;

typedef struct {
    uint64_t timestamp_ns;
    uint32_t scan_number;
    float angles[4000]; // 4khz express scan, budget for 1Hz spinning at most
    float ranges[4000];  // in mm
} Scan;

typedef struct {
    uint8_t quality;
    uint16_t angle;
    uint16_t range;
} ScanResponse;
typedef struct {
    uint8_t model;
    uint8_t firmware_minor;
    uint8_t firmware_major;
    uint8_t hardware;
    uint8_t serial_number[16];
} GetInfoResponse;

typedef struct {
    uint8_t status;
    uint16_t error_code;
} GetHealthResponse;

typedef struct {
    uint16_t standard;
    uint16_t express;
} GetSampleRateResponse;

typedef struct {
    uint32_t type;
    uint8_t payload[20]; // can be uint16_t, uint32_t, string, none
} GetLidarConfResponse;

typedef enum : uint8_t {
    STOP = 0,
    RESET,
    SCAN,
    // EXPRESS_SCAN,
    // FORCE_SCAN,
    GET_INFO,
    GET_HEALTH, 
    GET_SAMPLERATE,
    GET_LIDAR_CONF,
    TOTAL_REQUEST_TYPES
} RequestType_e;

typedef enum : uint8_t {
    BYTE1_QUALITY = 0,
    BYTE2_ANGLE_LSB,
    BYTE3_ANGLE_MSB,
    BYTE4_RANGE_LSB,
    BYTE5_RANGE_MSG,
    SCAN_RESPONSE_BYTES
} ScanResponsePayload_e;

typedef enum : uint8_t {
    SCAN_RESPONSE = 0,
    GET_INFO_RESPONSE,
    GET_HEALTH_RESPONSE,
    GET_SAMPLE_RATE_RESPONSE,
    GET_LIDAR_CONF_RESPONSE,
    TOTAL_RESPONSE_TYPES
} ResponseType_e;

typedef struct {
    ResponseType_e type;
    union {
        ScanResponse scan_response;
        GetInfoResponse get_info_response;
        GetHealthResponse get_health_response;
        GetSampleRateResponse get_sample_rate_response;
        GetLidarConfResponse get_lidar_conf_response;
    };
} Response;

typedef enum : uint8_t {
    IDLE = 0,
    INIT,
    REQUESTING,
    SCANNING,
    ERROR,
    TOTAL_STATE_TYPES
} StateType_e;

typedef enum : uint8_t {
    HEALTH_CLEAR = 0,
    HEALTH_WARNING = 1,
    HEALTH_ERROR = 2
} GetHealthStatus_e;

const static uint8_t REQUEST_COMMANDS[TOTAL_REQUEST_TYPES] = {
    [STOP] = 0x25,
    [RESET] = 0x40,
    [SCAN] = 0x20,
    [GET_INFO] = 0x50,
    [GET_HEALTH] = 0x52,   
    [GET_SAMPLERATE] = 0x59,
    [GET_LIDAR_CONF] = 0x84,
};

uint8_t rplidar_init();
void rplidar_deinit();
uint8_t rplidar_start_scanning(void* buffer, size_t size);
void rplidar_scan_periodic(void *arg);
void lidar_send_request(RequestType_e type, Request request);
void lidar_process_response(Response *response);
void lidar_parse_scan(ScanResponse *scan_response);

