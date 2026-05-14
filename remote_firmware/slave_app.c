/*
 * From-core baremetal firmware skeleton.
 *
 * Integration position:
 *   phytium-pi-os/output/build/phytium-standalone-openamp-v1.0/
 *   example/system/amp/openamp_for_linux/src/slaver_00_example.c
 *
 * This file is not a full Phytium SDK project by itself. It keeps the command
 * parsing and safety logic independent, so it can be copied into the real
 * OpenAMP rpmsg callback and compiled into openamp_core0.elf.
 */

#include "../src/rpmsg_protocol.h"
#include "jc4010_can.h"
#include "phytium_can_port.h"
#include <stdint.h>
#include <string.h>

typedef struct {
    int8_t motor_left;
    int8_t motor_right;
    uint8_t heartbeat_ok;
    uint32_t bad_frame_count;
} SlaveControlState;

static SlaveControlState g_state;

static void enter_safe_state(void)
{
    g_state.motor_left = 0;
    g_state.motor_right = 0;
}

static void apply_motor_output(int8_t left, int8_t right)
{
    /* TODO: replace with Phytium GPIO/PWM driver calls after motor board is ready. */
    g_state.motor_left = left;
    g_state.motor_right = right;
}

static int send_jc4010_frame(const Jc4010CanFrame *frame)
{
    return phytium_can_send(frame);
}

static int32_t read_be_i32(const uint8_t *p)
{
    return ((int32_t)p[0] << 24) |
           ((int32_t)p[1] << 16) |
           ((int32_t)p[2] << 8) |
           (int32_t)p[3];
}

static uint16_t read_be_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void handle_can_enable(uint8_t motor_id)
{
    Jc4010CanFrame can_frame;
    jc4010_build_enable(motor_id, &can_frame);
    send_jc4010_frame(&can_frame);
}

static void handle_can_zero(uint8_t motor_id)
{
    Jc4010CanFrame can_frame;
    jc4010_build_zero_position(motor_id, &can_frame);
    send_jc4010_frame(&can_frame);
}

static void handle_can_safe_stop(uint8_t motor_id)
{
    Jc4010CanFrame can_frame;
    jc4010_build_safe_stop(motor_id, &can_frame);
    send_jc4010_frame(&can_frame);
}

static void handle_can_pvt(const uint8_t *payload, uint8_t length)
{
    if (length < 8) {
        enter_safe_state();
        return;
    }

    uint8_t motor_id = payload[0];
    int32_t position_x100_deg = read_be_i32(&payload[1]);
    uint16_t speed_rpm = read_be_u16(&payload[5]);
    uint8_t torque_percent = payload[7];

    Jc4010CanFrame can_frame;
    jc4010_build_pvt(motor_id, position_x100_deg, speed_rpm, torque_percent, &can_frame);
    send_jc4010_frame(&can_frame);
}

static size_t build_ack(uint8_t seq, uint8_t *out, size_t out_size)
{
    uint8_t payload[3];
    payload[0] = (uint8_t)g_state.motor_left;
    payload[1] = (uint8_t)g_state.motor_right;
    payload[2] = g_state.heartbeat_ok;
    return rpmsg_encode(CMD_HEARTBEAT, seq, payload, sizeof(payload), out, out_size);
}

size_t slave_handle_frame(const uint8_t *data, unsigned int len, uint8_t *reply, size_t reply_size)
{
    RpmsgFrame frame;
    if (!rpmsg_decode(data, len, &frame)) {
        g_state.bad_frame_count++;
        enter_safe_state();
        return 0;
    }

    switch (frame.type) {
    case CMD_HEARTBEAT:
        g_state.heartbeat_ok = 1;
        return build_ack(frame.seq, reply, reply_size);
    case CMD_SET_MOTOR:
        if (frame.length >= 2) {
            apply_motor_output((int8_t)frame.payload[0], (int8_t)frame.payload[1]);
        }
        return build_ack(frame.seq, reply, reply_size);
    case CMD_SAFE_STOP:
        enter_safe_state();
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_ENABLE:
        if (frame.length >= 1) {
            handle_can_enable(frame.payload[0]);
        }
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_ZERO_POSITION:
        if (frame.length >= 1) {
            handle_can_zero(frame.payload[0]);
        }
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_PVT:
        handle_can_pvt(frame.payload, frame.length);
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_SAFE_STOP:
        if (frame.length >= 1) {
            handle_can_safe_stop(frame.payload[0]);
        } else {
            handle_can_safe_stop(1);
            handle_can_safe_stop(2);
        }
        enter_safe_state();
        return build_ack(frame.seq, reply, reply_size);
    default:
        enter_safe_state();
        return 0;
    }
}

/*
 * Real SDK callback shape reference:
 *
 * static int rpmsg_endpoint_cb(struct rpmsg_endpoint *ept, void *data,
 *                              size_t len, uint32_t src, void *priv)
 * {
 *     uint8_t reply[32];
 *     size_t reply_len = slave_handle_frame(data, len, reply, sizeof(reply));
 *     if (reply_len > 0) {
 *         rpmsg_send(ept, reply, reply_len);
 *     }
 *     return 0;
 * }
 */
