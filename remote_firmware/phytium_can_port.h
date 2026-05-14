#ifndef PHYTIUM_CAN_PORT_H
#define PHYTIUM_CAN_PORT_H

#include "jc4010_can.h"

typedef struct {
    int init_ret;
    int last_send_ret;
    uint32_t can_id;
    uint32_t baudrate;
    uint32_t send_count;
    uint32_t last_frame_id;
    uint8_t last_frame_dlc;
    uint8_t last_frame_data[8];
} PhytiumCanDebugState;

/* Platform adapter.
 *
 * The JC4010 protocol code is platform-independent. This file is the only
 * place that should call the Phytium Standalone SDK CAN driver.
 *
 * Return 0 on success, negative value on failure.
 */
int phytium_can_init(void);
int phytium_can_send(const Jc4010CanFrame *frame);
const PhytiumCanDebugState *phytium_can_get_debug_state(void);

#endif
