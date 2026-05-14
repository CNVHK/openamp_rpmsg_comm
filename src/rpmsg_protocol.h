#ifndef RPMSG_PROTOCOL_H
#define RPMSG_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define RPMSG_FRAME_MAGIC 0xA5
#define RPMSG_MAX_PAYLOAD 64

typedef enum {
    CMD_HEARTBEAT = 1,
    CMD_SET_MOTOR = 2,
    CMD_READ_IMU = 3,
    CMD_SAFE_STOP = 4,
    CMD_CAN_ENABLE = 10,
    CMD_CAN_ZERO_POSITION = 11,
    CMD_CAN_PVT = 12,
    CMD_CAN_SAFE_STOP = 13,
    CMD_CAN_SET_MODE = 14,
    CMD_CAN_INIT_MOTOR = 15
} CommandType;

typedef struct {
    uint8_t type;
    uint8_t seq;
    uint8_t length;
    uint8_t payload[RPMSG_MAX_PAYLOAD];
} RpmsgFrame;

size_t rpmsg_encode(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t length,
                    uint8_t *out, size_t out_size);
bool rpmsg_decode(const uint8_t *data, size_t size, RpmsgFrame *frame);
uint8_t rpmsg_checksum(const uint8_t *data, size_t size);

#endif
