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
#include "phytium_bmi088_port.h"
#include "phytium_can_port.h"
#include "phytium_servo_port.h"
#include "fsleep.h"
#include <stdint.h>
#include <string.h>

typedef struct {
    int8_t motor_left;
    int8_t motor_right;
    uint8_t heartbeat_ok;
    int8_t last_can_ret;
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
    int ret = phytium_can_send(frame);
    g_state.last_can_ret = (int8_t)ret;
    return ret;
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

static void write_be_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)(value & 0xff);
}

static void write_be_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)(value & 0xff);
}

static void handle_servo_set4(const uint8_t *payload, uint8_t length)
{
    uint16_t angles[PHYTIUM_SERVO_NUM];

    if (length < PHYTIUM_SERVO_NUM * 2U) {
        return;
    }

    for (int i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        angles[i] = read_be_u16(&payload[i * 2]);
    }

    phytium_servo_set_all(angles);
}

static void handle_servo_center(void)
{
    const uint16_t angles[PHYTIUM_SERVO_NUM] = {90, 90, 90, 90};
    phytium_servo_set_all(angles);
}

static void handle_imu_init(void)
{
    (void)phytium_bmi088_init();
}

static void handle_imu_read(void)
{
    (void)phytium_bmi088_read_sample();
}

static void handle_can_enable(uint8_t motor_id)
{
    Jc4010CanFrame can_frame;
    jc4010_build_enable(motor_id, &can_frame);
    send_jc4010_frame(&can_frame);
}

static void handle_can_clear_fault(uint8_t motor_id)
{
    Jc4010CanFrame can_frame;
    jc4010_build_clear_fault(motor_id, &can_frame);
    send_jc4010_frame(&can_frame);
}

static void handle_can_set_mode(uint8_t motor_id, uint16_t mode)
{
    Jc4010CanFrame can_frame;
    jc4010_build_set_mode(motor_id, mode, &can_frame);
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

static void handle_can_init_motor(uint8_t motor_id)
{
    Jc4010CanFrame can_frame;

    /*
     * Same command order as G431_CAN/Core/Src/main.c:
     * 0x00A5=1, mode 0x0060=2, zero 0x00A7=1, enable 0x00A2=1.
     */
    jc4010_build_clear_fault(motor_id, &can_frame);
    if (send_jc4010_frame(&can_frame) != 0) {
        return;
    }
    fsleep_millisec(20);
    if (!phytium_can_bus_ok()) {
        return;
    }
    fsleep_millisec(2000);
    handle_can_set_mode(motor_id, 2);
    fsleep_millisec(10);
    handle_can_zero(motor_id);
    fsleep_millisec(10);
    handle_can_enable(motor_id);
    fsleep_millisec(10);
}

static void handle_can_g431_init(void)
{
    Jc4010CanFrame can_frame;

    /*
     * Match G431_CAN/Core/Src/main.c more closely:
     * send the same setup command to motor 1 and 2, then wait once.
     */
    jc4010_build_clear_fault(1, &can_frame);
    if (send_jc4010_frame(&can_frame) != 0) {
        return;
    }
    jc4010_build_clear_fault(2, &can_frame);
    if (send_jc4010_frame(&can_frame) != 0) {
        return;
    }
    fsleep_millisec(20);
    if (!phytium_can_bus_ok()) {
        return;
    }
    fsleep_millisec(2000);

    handle_can_set_mode(1, 2);
    handle_can_set_mode(2, 2);
    fsleep_millisec(10);

    handle_can_zero(1);
    handle_can_zero(2);
    fsleep_millisec(10);

    handle_can_enable(1);
    handle_can_enable(2);
    fsleep_millisec(10);
}

static void handle_can_g431_demo(uint8_t state)
{
    Jc4010CanFrame can_frame;

    if (state == 0) {
        jc4010_build_pvt(1, 36000, 200, 50, &can_frame);
        send_jc4010_frame(&can_frame);
        jc4010_build_pvt(2, 9000, 200, 50, &can_frame);
        send_jc4010_frame(&can_frame);
    } else {
        jc4010_build_pvt(1, 0, 200, 80, &can_frame);
        send_jc4010_frame(&can_frame);
        jc4010_build_pvt(2, 0, 200, 80, &can_frame);
        send_jc4010_frame(&can_frame);
    }
}

static size_t build_ack(uint8_t seq, uint8_t *out, size_t out_size)
{
    const PhytiumCanDebugState *can_dbg = phytium_can_get_debug_state();
    const PhytiumServoDebugState *servo_dbg = phytium_servo_get_debug_state();
    const PhytiumBmi088DebugState *imu_dbg = phytium_bmi088_get_debug_state();
    uint8_t payload[88];
    memset(payload, 0, sizeof(payload));

    payload[0] = (uint8_t)g_state.motor_left;
    payload[1] = (uint8_t)g_state.motor_right;
    payload[2] = g_state.heartbeat_ok;
    payload[3] = (uint8_t)g_state.last_can_ret;
    payload[4] = (uint8_t)can_dbg->init_ret;
    payload[5] = (uint8_t)can_dbg->last_send_ret;
    payload[6] = (uint8_t)can_dbg->can_id;
    payload[7] = (uint8_t)(can_dbg->baudrate >> 24);
    payload[8] = (uint8_t)(can_dbg->baudrate >> 16);
    payload[9] = (uint8_t)(can_dbg->baudrate >> 8);
    payload[10] = (uint8_t)(can_dbg->baudrate & 0xff);
    payload[11] = (uint8_t)(can_dbg->send_count >> 24);
    payload[12] = (uint8_t)(can_dbg->send_count >> 16);
    payload[13] = (uint8_t)(can_dbg->send_count >> 8);
    payload[14] = (uint8_t)(can_dbg->send_count & 0xff);
    payload[15] = (uint8_t)(can_dbg->last_frame_id >> 8);
    payload[16] = (uint8_t)(can_dbg->last_frame_id & 0xff);
    payload[17] = can_dbg->last_frame_dlc;
    for (int i = 0; i < 8; ++i) {
        payload[18 + i] = can_dbg->last_frame_data[i];
    }
    write_be_u32(&payload[26], can_dbg->reg_ctrl);
    write_be_u32(&payload[30], can_dbg->reg_intr);
    write_be_u32(&payload[34], can_dbg->reg_xfer_sts);
    write_be_u32(&payload[38], can_dbg->reg_err_cnt);
    write_be_u32(&payload[42], can_dbg->reg_fifo_cnt);
    write_be_u32(&payload[46], can_dbg->reg_xfer_en);
    payload[50] = (uint8_t)servo_dbg->init_ret;
    payload[51] = (uint8_t)servo_dbg->last_ret;
    for (int i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        write_be_u16(&payload[52 + i * 2], servo_dbg->angle_deg[i]);
    }
    for (int i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        write_be_u16(&payload[80 + i * 2], servo_dbg->pulse_us[i]);
    }
    payload[60] = (uint8_t)imu_dbg->init_ret;
    payload[61] = (uint8_t)imu_dbg->last_ret;
    payload[62] = imu_dbg->accel_chip_id;
    payload[63] = imu_dbg->gyro_chip_id;
    for (int i = 0; i < 3; ++i) {
        write_be_u16(&payload[64 + i * 2], (uint16_t)imu_dbg->accel_raw[i]);
        write_be_u16(&payload[70 + i * 2], (uint16_t)imu_dbg->gyro_raw[i]);
    }
    write_be_u32(&payload[76], imu_dbg->read_count);

    return rpmsg_encode(CMD_HEARTBEAT, seq, payload, 88, out, out_size);
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
    case CMD_CAN_SET_MODE:
        if (frame.length >= 3) {
            handle_can_set_mode(frame.payload[0], read_be_u16(&frame.payload[1]));
        }
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_INIT_MOTOR:
        if (frame.length >= 1) {
            handle_can_init_motor(frame.payload[0]);
        }
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_G431_INIT:
        handle_can_g431_init();
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_G431_DEMO:
        handle_can_g431_demo(frame.length >= 1 ? frame.payload[0] : 0);
        return build_ack(frame.seq, reply, reply_size);
    case CMD_SERVO_SET4:
        handle_servo_set4(frame.payload, frame.length);
        return build_ack(frame.seq, reply, reply_size);
    case CMD_SERVO_CENTER:
        handle_servo_center();
        return build_ack(frame.seq, reply, reply_size);
    case CMD_IMU_INIT:
        handle_imu_init();
        return build_ack(frame.seq, reply, reply_size);
    case CMD_IMU_READ:
        handle_imu_read();
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
