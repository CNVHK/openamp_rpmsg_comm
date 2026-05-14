#include "../src/rpmsg_protocol.h"

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

static int wait_readable(int fd, int timeout_ms)
{
    fd_set rfds;
    struct timeval tv;

    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    return select(fd + 1, &rfds, NULL, NULL, &tv);
}

static void put_be_i32(uint8_t *p, int32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)(value & 0xff);
}

static void put_be_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)(value & 0xff);
}

static void usage(const char *prog)
{
    printf("Usage:\n");
    printf("  %s <rpmsg_dev> heartbeat\n", prog);
    printf("  %s <rpmsg_dev> enable <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> zero <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> mode <motor_id> <mode>\n", prog);
    printf("  %s <rpmsg_dev> init <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> pvt <motor_id> <pos_x100_deg> <speed_rpm> <torque_percent>\n", prog);
    printf("  %s <rpmsg_dev> stop [motor_id]\n", prog);
    printf("\nExamples:\n");
    printf("  %s /dev/rpmsg0 heartbeat\n", prog);
    printf("  %s /dev/rpmsg0 init 1\n", prog);
    printf("  %s /dev/rpmsg0 enable 1\n", prog);
    printf("  %s /dev/rpmsg0 pvt 1 1000 100 20\n", prog);
    printf("  %s /dev/rpmsg0 stop 1\n", prog);
}

static int build_command(int argc, char **argv, uint8_t *type, uint8_t *payload, uint8_t *payload_len)
{
    const char *cmd = argc > 2 ? argv[2] : "heartbeat";

    *payload_len = 0;
    if (strcmp(cmd, "heartbeat") == 0) {
        *type = CMD_HEARTBEAT;
        payload[0] = 0;
        *payload_len = 1;
        return 0;
    }

    if (strcmp(cmd, "enable") == 0) {
        if (argc < 4) return -1;
        *type = CMD_CAN_ENABLE;
        payload[0] = (uint8_t)strtoul(argv[3], NULL, 0);
        *payload_len = 1;
        return 0;
    }

    if (strcmp(cmd, "zero") == 0) {
        if (argc < 4) return -1;
        *type = CMD_CAN_ZERO_POSITION;
        payload[0] = (uint8_t)strtoul(argv[3], NULL, 0);
        *payload_len = 1;
        return 0;
    }

    if (strcmp(cmd, "mode") == 0) {
        if (argc < 5) return -1;
        *type = CMD_CAN_SET_MODE;
        payload[0] = (uint8_t)strtoul(argv[3], NULL, 0);
        put_be_u16(&payload[1], (uint16_t)strtoul(argv[4], NULL, 0));
        *payload_len = 3;
        return 0;
    }

    if (strcmp(cmd, "init") == 0) {
        if (argc < 4) return -1;
        *type = CMD_CAN_INIT_MOTOR;
        payload[0] = (uint8_t)strtoul(argv[3], NULL, 0);
        *payload_len = 1;
        return 0;
    }

    if (strcmp(cmd, "pvt") == 0) {
        if (argc < 7) return -1;
        *type = CMD_CAN_PVT;
        payload[0] = (uint8_t)strtoul(argv[3], NULL, 0);
        put_be_i32(&payload[1], (int32_t)strtol(argv[4], NULL, 0));
        put_be_u16(&payload[5], (uint16_t)strtoul(argv[5], NULL, 0));
        payload[7] = (uint8_t)strtoul(argv[6], NULL, 0);
        *payload_len = 8;
        return 0;
    }

    if (strcmp(cmd, "stop") == 0) {
        *type = CMD_CAN_SAFE_STOP;
        if (argc >= 4) {
            payload[0] = (uint8_t)strtoul(argv[3], NULL, 0);
            *payload_len = 1;
        } else {
            *payload_len = 0;
        }
        return 0;
    }

    return -1;
}

int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "/dev/rpmsg0";
    uint8_t type;
    uint8_t payload[RPMSG_MAX_PAYLOAD];
    uint8_t payload_len;
    uint8_t tx_frame[96];
    uint8_t rx_frame[96];
    RpmsgFrame ack;

    if (build_command(argc, argv, &type, payload, &payload_len) != 0) {
        usage(argv[0]);
        return 1;
    }

    int fd = open(dev, O_RDWR);
    if (fd < 0) {
        perror("open rpmsg device");
        return 1;
    }

    size_t size = rpmsg_encode(type, 1, payload, payload_len, tx_frame, sizeof(tx_frame));
    if (size == 0) {
        fprintf(stderr, "encode command failed\n");
        close(fd);
        return 1;
    }

    if (write(fd, tx_frame, size) != (ssize_t)size) {
        perror("write");
        close(fd);
        return 1;
    }

    printf("command type=%u sent to %s, len=%zu\n", type, dev, size);

    int ready = wait_readable(fd, 1500);
    if (ready < 0) {
        perror("select");
        close(fd);
        return 1;
    }
    if (ready == 0) {
        printf("timeout: no reply from remote core\n");
        close(fd);
        return 2;
    }

    ssize_t rx_size = read(fd, rx_frame, sizeof(rx_frame));
    if (rx_size < 0) {
        perror("read");
        close(fd);
        return 1;
    }

    printf("received raw reply, len=%zd\n", rx_size);
    if (!rpmsg_decode(rx_frame, (size_t)rx_size, &ack)) {
        printf("decode reply failed\n");
        close(fd);
        return 3;
    }

    printf("ack type=%u seq=%u payload_len=%u\n", ack.type, ack.seq, ack.length);
    if (ack.length >= 3) {
        printf("remote state: motor_left=%d motor_right=%d heartbeat_ok=%u",
               (int8_t)ack.payload[0],
               (int8_t)ack.payload[1],
               ack.payload[2]);
        if (ack.length >= 4) {
            printf(" last_can_ret=%d", (int8_t)ack.payload[3]);
        }
        printf("\n");
    }

    close(fd);
    return 0;
}
