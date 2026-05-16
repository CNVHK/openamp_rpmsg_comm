#ifndef PHYTIUM_BMI088_PORT_H
#define PHYTIUM_BMI088_PORT_H

#include <stdint.h>

typedef struct {
    int8_t init_ret;
    int8_t last_ret;
    uint8_t accel_chip_id;
    uint8_t gyro_chip_id;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    uint32_t read_count;
} PhytiumBmi088DebugState;

int phytium_bmi088_init(void);
int phytium_bmi088_read_sample(void);
const PhytiumBmi088DebugState *phytium_bmi088_get_debug_state(void);

#endif
