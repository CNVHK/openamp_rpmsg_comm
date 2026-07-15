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

#include "rpmsg_protocol.h"
#include "balance_controller.h"
#include "motor_can.h"
#include "phytium_bmi088_port.h"
#include "phytium_can_port.h"
#include "phytium_servo_port.h"
#include "fsleep.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    uint8_t heartbeat_ok;
    int8_t last_can_ret;
} SlaveControlState;

static SlaveControlState g_state;
static uint8_t g_last_command_type;

static int send_motor_frame(const MotorCanFrame *frame)
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

static void handle_servo_polarity(const uint8_t *payload, uint8_t length)
{
    if (length < 1U) {
        return;
    }

    phytium_servo_set_polarity(payload[0]);
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
    MotorCanFrame can_frame;
    motor_build_enable(motor_id, &can_frame);
    send_motor_frame(&can_frame);
}

static void handle_can_set_mode(uint8_t motor_id, uint16_t mode)
{
    MotorCanFrame can_frame;
    motor_build_set_mode(motor_id, mode, &can_frame);
    send_motor_frame(&can_frame);
}

static void handle_can_temporary_origin(uint8_t motor_id)
{
    MotorCanFrame can_frame;
    motor_build_set_temporary_origin(motor_id, &can_frame);
    send_motor_frame(&can_frame);
}

static void write_be_i32(uint8_t *p, int32_t value)
{
    write_be_u32(p, (uint32_t)value);
}

static int32_t float_to_i32(float value, float scale)
{
    float scaled;

    if (!isfinite(value)) {
        return 0;
    }
    scaled = value * scale;
    if (scaled > 2147483647.0f) {
        return INT32_MAX;
    }
    if (scaled < -2147483648.0f) {
        return INT32_MIN;
    }
    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static void handle_can_set_origin(uint8_t motor_id)
{
    MotorCanFrame can_frame;
    motor_build_set_origin(motor_id, &can_frame);
    send_motor_frame(&can_frame);
}

static void handle_can_safe_stop(uint8_t motor_id)
{
    MotorCanFrame can_frame;
    motor_build_safe_stop(motor_id, &can_frame);
    send_motor_frame(&can_frame);
}

static void handle_can_pvt(const uint8_t *payload, uint8_t length)
{
    if (length < 8) {
        return;
    }

    uint8_t motor_id = payload[0];
    int32_t position_x100_deg = read_be_i32(&payload[1]);
    uint16_t speed_rpm = read_be_u16(&payload[5]);
    uint8_t torque_percent = payload[7];

    MotorCanFrame can_frame;
    motor_build_pvt(motor_id, position_x100_deg, speed_rpm, torque_percent, &can_frame);
    send_motor_frame(&can_frame);
}

static void handle_can_init_motor(uint8_t motor_id)
{
    MotorCanFrame can_frame;

    /* Keep the saved origin intact: select position mode, then enter closed loop. */
    motor_build_set_mode(motor_id, 2, &can_frame);
    if (send_motor_frame(&can_frame) != 0) {
        return;
    }
    fsleep_millisec(10);
    motor_build_enable(motor_id, &can_frame);
    send_motor_frame(&can_frame);
    fsleep_millisec(10);
}

static int handle_motor_init_all(void)
{
    MotorCanFrame can_frame;

    motor_build_set_mode(1, 2, &can_frame);
    if (send_motor_frame(&can_frame) != 0) {
        return -1;
    }
    motor_build_set_mode(2, 2, &can_frame);
    if (send_motor_frame(&can_frame) != 0) {
        return -1;
    }
    fsleep_millisec(10);

    motor_build_enable(1, &can_frame);
    if (send_motor_frame(&can_frame) != 0) {
        return -1;
    }
    motor_build_enable(2, &can_frame);
    if (send_motor_frame(&can_frame) != 0) {
        return -1;
    }
    fsleep_millisec(10);
    return 0;
}

static void handle_motor_test(void)
{
    MotorCanFrame can_frame;

    if (handle_motor_init_all() != 0) {
        return;
    }
    motor_build_pvt(1, 36000, 200, 50, &can_frame);
    send_motor_frame(&can_frame);
    motor_build_pvt(2, 36000, 200, 50, &can_frame);
    send_motor_frame(&can_frame);
}

static size_t build_ack(uint8_t seq, uint8_t *out, size_t out_size)
{
    const PhytiumCanDebugState *can_dbg = phytium_can_get_debug_state();
    const PhytiumServoDebugState *servo_dbg = phytium_servo_get_debug_state();
    const PhytiumBmi088DebugState *imu_dbg = phytium_bmi088_get_debug_state();
    const BalanceTelemetry *balance = balance_control_get_telemetry();
    MotorFeedback left_feedback;
    MotorFeedback right_feedback;
    uint8_t payload[120];
    memset(payload, 0, sizeof(payload));

    payload[0] = (uint8_t)can_dbg->receive_count;
    payload[1] = (uint8_t)can_dbg->feedback_count;
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
    if (g_last_command_type >= CMD_BALANCE_ENABLE &&
        g_last_command_type <= CMD_BALANCE_STATUS) {
        if (phytium_can_get_motor_feedback(1U, &left_feedback) == 0) {
            write_be_u16(&payload[80], (uint16_t)left_feedback.current_x100_a);
            write_be_u16(&payload[84], (uint16_t)left_feedback.speed_rpm);
        }
        if (phytium_can_get_motor_feedback(2U, &right_feedback) == 0) {
            write_be_u16(&payload[82], (uint16_t)right_feedback.current_x100_a);
            write_be_u16(&payload[86], (uint16_t)right_feedback.speed_rpm);
        }
    } else {
        for (int i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
            write_be_u16(&payload[80 + i * 2], servo_dbg->pulse_us[i]);
        }
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

    payload[88] = balance->state;
    payload[89] = balance->fault;
    write_be_u16(&payload[90], balance->control_hz);
    write_be_i32(&payload[92], float_to_i32(balance->pitch_rad, 1000000.0f));
    write_be_i32(&payload[96], float_to_i32(balance->pitch_rate_rad_s, 1000000.0f));
    write_be_i32(&payload[100], float_to_i32(balance->wheel_position_m, 1000000.0f));
    write_be_i32(&payload[104], float_to_i32(balance->wheel_velocity_m_s, 1000000.0f));
    write_be_i32(&payload[108], float_to_i32(balance->left_torque_nm, 1000000.0f));
    write_be_i32(&payload[112], float_to_i32(balance->right_torque_nm, 1000000.0f));
    write_be_u32(&payload[116], balance->loop_count);

    return rpmsg_encode(CMD_HEARTBEAT, seq, payload, sizeof(payload), out, out_size);
}

int slave_app_init(void)
{
    return balance_control_init();
}

void slave_app_poll(void)
{
    balance_control_poll();
}

void slave_app_shutdown(void)
{
    balance_control_disable();
}

size_t slave_handle_frame(const uint8_t *data, unsigned int len, uint8_t *reply, size_t reply_size)
{
    RpmsgFrame frame;
    if (!rpmsg_decode(data, len, &frame)) {
        return 0;
    }
    g_last_command_type = frame.type;

    switch (frame.type) {
    case CMD_HEARTBEAT:
        g_state.heartbeat_ok = 1;
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
    case CMD_MOTOR_TEST:
        handle_motor_test();
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_SET_ORIGIN:
        if (frame.length >= 1) {
            handle_can_set_origin(frame.payload[0]);
        }
        return build_ack(frame.seq, reply, reply_size);
    case CMD_SERVO_SET4:
        handle_servo_set4(frame.payload, frame.length);
        return build_ack(frame.seq, reply, reply_size);
    case CMD_SERVO_CENTER:
        handle_servo_center();
        return build_ack(frame.seq, reply, reply_size);
    case CMD_SERVO_POLARITY:
        handle_servo_polarity(frame.payload, frame.length);
        return build_ack(frame.seq, reply, reply_size);
    case CMD_IMU_INIT:
        handle_imu_init();
        return build_ack(frame.seq, reply, reply_size);
    case CMD_IMU_READ:
        handle_imu_read();
        return build_ack(frame.seq, reply, reply_size);
    case CMD_BALANCE_ENABLE:
        (void)balance_control_enable();
        return build_ack(frame.seq, reply, reply_size);
    case CMD_BALANCE_DISABLE:
        balance_control_disable();
        return build_ack(frame.seq, reply, reply_size);
    case CMD_BALANCE_STATUS:
        return build_ack(frame.seq, reply, reply_size);
    case CMD_CAN_ZERO_POSITION:
        if (frame.length >= 1) {
            handle_can_temporary_origin(frame.payload[0]);
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
        return build_ack(frame.seq, reply, reply_size);
    default:
        return 0;
    }
}
