#include "motor_can.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    MotorCanFrame frame;
    MotorFeedback feedback = {0};

    motor_build_torque(2U, -25, &frame);
    assert(frame.id == 0x602U);
    assert(frame.data[0] == 0x2bU);
    assert(frame.data[1] == 0x00U && frame.data[2] == 0x20U);
    assert(frame.data[4] == 0xffU && frame.data[5] == 0xe7U);

    frame.id = 0x581U;
    frame.dlc = 8U;
    frame.data[0] = 0x2aU;
    frame.data[1] = 0xffU;
    frame.data[2] = 0xffU;
    frame.data[3] = 0x9cU;
    frame.data[4] = 0xffU;
    frame.data[5] = 0x9cU;
    frame.data[6] = 0x00U;
    frame.data[7] = 0x7bU;
    assert(motor_parse_feedback(&frame, &feedback) == 0);
    assert(feedback.motor_id == 1U);
    assert(feedback.position_x100_deg == -100);
    assert(feedback.speed_rpm == -100);
    assert(feedback.current_x100_a == 123);
    assert(feedback.valid == 1U);

    puts("motor_balance_protocol_test: PASS");
    return 0;
}
