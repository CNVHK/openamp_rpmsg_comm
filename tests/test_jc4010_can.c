#include "jc4010_can.h"

#include <assert.h>
#include <stdio.h>

static void test_pvt_frame(void)
{
    Jc4010CanFrame frame;
    jc4010_build_pvt(1, 36000, 200, 50, &frame);

    assert(frame.id == 0x601);
    assert(frame.dlc == 8);
    assert(frame.data[0] == 0x25);
    assert(frame.data[1] == 0x00);
    assert(frame.data[2] == 0x00);
    assert(frame.data[3] == 0x8C);
    assert(frame.data[4] == 0xA0);
    assert(frame.data[5] == 0x00);
    assert(frame.data[6] == 0xC8);
    assert(frame.data[7] == 50);
}

static void test_known_linux_cansend_frame(void)
{
    Jc4010CanFrame frame;
    jc4010_build_pvt(1, 500, 50, 10, &frame);

    assert(frame.id == 0x601);
    assert(frame.dlc == 8);
    assert(frame.data[0] == 0x25);
    assert(frame.data[1] == 0x00);
    assert(frame.data[2] == 0x00);
    assert(frame.data[3] == 0x01);
    assert(frame.data[4] == 0xF4);
    assert(frame.data[5] == 0x00);
    assert(frame.data[6] == 0x32);
    assert(frame.data[7] == 0x0A);
}

static void test_status_parse(void)
{
    Jc4010MotorState m1 = {0};
    Jc4010MotorState m2 = {0};
    uint8_t data[8] = {0x2A, 0x00, 0x8C, 0xA0, 0x00, 0xC8, 0x00, 0x64};

    assert(jc4010_parse_status(0x581, data, &m1, &m2) == 1);
    assert((int)m1.position_deg == 360);
    assert((int)m1.speed_rpm == 2);
    assert((int)m1.current_a == 1);
}

int main(void)
{
    test_pvt_frame();
    test_known_linux_cansend_frame();
    test_status_parse();
    printf("jc4010 can protocol tests passed\n");
    return 0;
}
