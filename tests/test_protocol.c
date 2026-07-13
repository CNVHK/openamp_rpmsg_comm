#include "rpmsg_protocol.h"
#include <assert.h>
#include <stdio.h>

static void test_encode_decode(void)
{
    uint8_t payload[1] = {1};
    uint8_t buffer[32];
    RpmsgFrame frame;

    size_t size = rpmsg_encode(CMD_CAN_ENABLE, 7, payload, 1, buffer, sizeof(buffer));
    assert(size == 6);
    assert(rpmsg_decode(buffer, size, &frame));
    assert(frame.type == CMD_CAN_ENABLE);
    assert(frame.seq == 7);
    assert(frame.length == 1);
    assert(frame.payload[0] == 1);
}

static void test_bad_checksum(void)
{
    uint8_t payload[1] = {1};
    uint8_t buffer[32];
    RpmsgFrame frame;

    size_t size = rpmsg_encode(CMD_HEARTBEAT, 1, payload, 1, buffer, sizeof(buffer));
    buffer[size - 1] ^= 0x55;
    assert(!rpmsg_decode(buffer, size, &frame));
}

int main(void)
{
    test_encode_decode();
    test_bad_checksum();
    printf("rpmsg protocol tests passed\n");
    return 0;
}

