#include "jc4010_can.h"

static void jc4010_clear_frame(uint8_t motor_id, Jc4010CanFrame *frame)
{
    frame->id = 0x600u + motor_id;
    frame->dlc = JC4010_CAN_DLC;
    for (int i = 0; i < JC4010_CAN_DLC; ++i) {
        frame->data[i] = 0;
    }
}

void jc4010_build_cmd(uint8_t motor_id, uint8_t cmd_word, uint16_t reg_addr,
                      int32_t value, Jc4010CanFrame *frame)
{
    jc4010_clear_frame(motor_id, frame);

    frame->data[0] = cmd_word;
    frame->data[1] = (uint8_t)(reg_addr >> 8);
    frame->data[2] = (uint8_t)(reg_addr & 0xff);
    frame->data[3] = 0x00;

    if (cmd_word == 0x2B) {
        frame->data[4] = (uint8_t)(value >> 8);
        frame->data[5] = (uint8_t)(value & 0xff);
    } else if (cmd_word == 0x23) {
        frame->data[4] = (uint8_t)(value >> 24);
        frame->data[5] = (uint8_t)(value >> 16);
        frame->data[6] = (uint8_t)(value >> 8);
        frame->data[7] = (uint8_t)(value & 0xff);
    }
}

void jc4010_build_pvt(uint8_t motor_id, int32_t position_x100_deg,
                      uint16_t speed_rpm, uint8_t torque_percent,
                      Jc4010CanFrame *frame)
{
    jc4010_clear_frame(motor_id, frame);

    frame->data[0] = 0x25;
    frame->data[1] = (uint8_t)(position_x100_deg >> 24);
    frame->data[2] = (uint8_t)(position_x100_deg >> 16);
    frame->data[3] = (uint8_t)(position_x100_deg >> 8);
    frame->data[4] = (uint8_t)(position_x100_deg & 0xff);
    frame->data[5] = (uint8_t)(speed_rpm >> 8);
    frame->data[6] = (uint8_t)(speed_rpm & 0xff);
    frame->data[7] = torque_percent;
}

void jc4010_build_enable(uint8_t motor_id, Jc4010CanFrame *frame)
{
    jc4010_build_cmd(motor_id, 0x2B, 0x00A2, 1, frame);
}

void jc4010_build_set_mode(uint8_t motor_id, uint16_t mode, Jc4010CanFrame *frame)
{
    jc4010_build_cmd(motor_id, 0x2B, 0x0060, mode, frame);
}

void jc4010_build_zero_position(uint8_t motor_id, Jc4010CanFrame *frame)
{
    jc4010_build_cmd(motor_id, 0x2B, 0x00A7, 1, frame);
}

void jc4010_build_safe_stop(uint8_t motor_id, Jc4010CanFrame *frame)
{
    jc4010_build_pvt(motor_id, 0, 0, 0, frame);
}

int jc4010_parse_status(uint32_t can_id, const uint8_t data[8],
                        Jc4010MotorState *motor1, Jc4010MotorState *motor2)
{
    Jc4010MotorState *target = 0;
    if (can_id == 0x581u) {
        target = motor1;
    } else if (can_id == 0x582u) {
        target = motor2;
    } else {
        return 0;
    }

    if (!target || data[0] != 0x2A) {
        return 0;
    }

    int32_t raw_pos = ((int32_t)data[1] << 16) | ((int32_t)data[2] << 8) | data[3];
    if (raw_pos & 0x00800000) {
        raw_pos |= (int32_t)0xFF000000;
    }

    int16_t raw_speed = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    int16_t raw_cur = (int16_t)(((uint16_t)data[6] << 8) | data[7]);

    target->position_deg = (float)raw_pos / 100.0f;
    target->speed_rpm = (float)raw_speed / 100.0f;
    target->current_a = (float)raw_cur / 100.0f;
    return 1;
}

