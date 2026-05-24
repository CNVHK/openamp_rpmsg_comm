#include "../src/rpmsg_protocol.h"

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#define RPMSG_CLIENT_VERSION "0.8.0-bmi088-spi"

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

static uint32_t read_be_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           (uint32_t)p[3];
}

static uint16_t read_be_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void usage(const char *prog)
{
    printf("Usage:\n");
    printf("  %s <rpmsg_dev> heartbeat\n", prog);
    printf("  %s <rpmsg_dev> enable <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> zero <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> mode <motor_id> <mode>\n", prog);
    printf("  %s <rpmsg_dev> init <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> g431init\n", prog);
    printf("  %s <rpmsg_dev> g431demo <0|1>\n", prog);
    printf("  %s <rpmsg_dev> servo <s0_deg> <s1_deg> <s2_deg> <s3_deg>\n", prog);
    printf("  %s <rpmsg_dev> servocenter\n", prog);
    printf("  %s <rpmsg_dev> imuinit\n", prog);
    printf("  %s <rpmsg_dev> imuread\n", prog);
    printf("  %s <rpmsg_dev> pvt <motor_id> <pos_x100_deg> <speed_rpm> <torque_percent>\n", prog);
    printf("  %s <rpmsg_dev> stop [motor_id]\n", prog);
    printf("\nExamples:\n");
    printf("  %s /dev/rpmsg0 heartbeat\n", prog);
    printf("  %s /dev/rpmsg0 init 1\n", prog);
    printf("  %s /dev/rpmsg0 g431init\n", prog);
    printf("  %s /dev/rpmsg0 g431demo 0\n", prog);
    printf("  %s /dev/rpmsg0 g431demo 1\n", prog);
    printf("  %s /dev/rpmsg0 servo 90 90 90 90\n", prog);
    printf("  %s /dev/rpmsg0 servocenter\n", prog);
    printf("  %s /dev/rpmsg0 imuinit\n", prog);
    printf("  %s /dev/rpmsg0 imuread\n", prog);
    printf("  watch -n 0.1 '%s /dev/rpmsg0 imuread'\n", prog);
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

    if (strcmp(cmd, "g431init") == 0) {
        *type = CMD_CAN_G431_INIT;
        *payload_len = 0;
        return 0;
    }

    if (strcmp(cmd, "g431demo") == 0) {
        *type = CMD_CAN_G431_DEMO;
        payload[0] = argc >= 4 ? (uint8_t)strtoul(argv[3], NULL, 0) : 0;
        *payload_len = 1;
        return 0;
    }

    if (strcmp(cmd, "servo") == 0) {
        if (argc < 7) return -1;
        *type = CMD_SERVO_SET4;
        for (int i = 0; i < 4; ++i) {
            unsigned long angle = strtoul(argv[3 + i], NULL, 0);
            if (angle > 180) angle = 180;
            put_be_u16(&payload[i * 2], (uint16_t)angle);
        }
        *payload_len = 8;
        return 0;
    }

    if (strcmp(cmd, "servocenter") == 0) {
        *type = CMD_SERVO_CENTER;
        *payload_len = 0;
        return 0;
    }

    if (strcmp(cmd, "imuinit") == 0) {
        *type = CMD_IMU_INIT;
        *payload_len = 0;
        return 0;
    }

    if (strcmp(cmd, "imuread") == 0) {
        *type = CMD_IMU_READ;
        *payload_len = 0;
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
    uint8_t tx_frame[128];
    uint8_t rx_frame[128];
    RpmsgFrame ack;

    printf("rpmsg_client version: %s\n", RPMSG_CLIENT_VERSION);

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

    int ready = wait_readable(fd, 5000);
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

    if (ack.length >= 26) {
        uint32_t baudrate = read_be_u32(&ack.payload[7]);
        uint32_t send_count = read_be_u32(&ack.payload[11]);
        uint16_t frame_id = (uint16_t)(((uint16_t)ack.payload[15] << 8) | ack.payload[16]);

        printf("can debug: init_ret=%d send_ret=%d can_id=%u baudrate=%u send_count=%u\n",
               (int8_t)ack.payload[4],
               (int8_t)ack.payload[5],
               ack.payload[6],
               baudrate,
               send_count);

        printf("last can frame: id=0x%03x dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
               frame_id,
               ack.payload[17],
               ack.payload[18],
               ack.payload[19],
               ack.payload[20],
               ack.payload[21],
               ack.payload[22],
               ack.payload[23],
               ack.payload[24],
               ack.payload[25]);
    }

    if (ack.length >= 50) {
        uint32_t reg_ctrl = read_be_u32(&ack.payload[26]);
        uint32_t reg_intr = read_be_u32(&ack.payload[30]);
        uint32_t reg_xfer_sts = read_be_u32(&ack.payload[34]);
        uint32_t reg_err_cnt = read_be_u32(&ack.payload[38]);
        uint32_t reg_fifo_cnt = read_be_u32(&ack.payload[42]);
        uint32_t reg_xfer_en = read_be_u32(&ack.payload[46]);
        uint32_t tx_err = (reg_err_cnt >> 16) & 0x1ff;
        uint32_t rx_err = reg_err_cnt & 0x1ff;
        uint32_t tx_fifo = (reg_fifo_cnt >> 16) & 0x7f;
        uint32_t rx_fifo = reg_fifo_cnt & 0x7f;
        uint32_t ctrl_enable = reg_ctrl & 0x1;
        uint32_t xfer_en_bit = reg_xfer_en & 0x1;

        printf("can regs: CTRL=0x%08X INTR=0x%08X XFER_STS=0x%08X ERR_CNT=0x%08X FIFO_CNT=0x%08X XFER_EN=0x%08X\n",
               reg_ctrl, reg_intr, reg_xfer_sts, reg_err_cnt, reg_fifo_cnt, reg_xfer_en);
        printf("can decoded: tx_err=%u rx_err=%u tx_fifo=%u rx_fifo=%u ctrl_enable=%u xfer_en_bit=%u\n",
               tx_err, rx_err, tx_fifo, rx_fifo, ctrl_enable, xfer_en_bit);
    }

    if (ack.length >= 60) {
        uint16_t servo_angle[4];
        for (int i = 0; i < 4; ++i) {
            servo_angle[i] = (uint16_t)(((uint16_t)ack.payload[52 + i * 2] << 8) |
                                        ack.payload[53 + i * 2]);
        }

        printf("servo debug: init_ret=%d last_ret=%d angles=%u,%u,%u,%u\n",
               (int8_t)ack.payload[50],
               (int8_t)ack.payload[51],
               servo_angle[0],
               servo_angle[1],
               servo_angle[2],
               servo_angle[3]);
        if (ack.length >= 87) {
            uint16_t servo_pulse[4];
            for (int i = 0; i < 4; ++i) {
                servo_pulse[i] = read_be_u16(&ack.payload[80 + i * 2]);
            }
            printf("servo pwm: pulse_us=%u,%u,%u,%u\n",
                   servo_pulse[0],
                   servo_pulse[1],
                   servo_pulse[2],
                   servo_pulse[3]);
        }
    }

    if (ack.length >= 80) {
        int16_t acc[3];
        int16_t gyro[3];
        for (int i = 0; i < 3; ++i) {
            acc[i] = (int16_t)(((uint16_t)ack.payload[64 + i * 2] << 8) |
                               ack.payload[65 + i * 2]);
            gyro[i] = (int16_t)(((uint16_t)ack.payload[70 + i * 2] << 8) |
                                ack.payload[71 + i * 2]);
        }

        printf("imu debug: init_ret=%d last_ret=%d accel_id=0x%02X gyro_id=0x%02X read_count=%u\n",
               (int8_t)ack.payload[60],
               (int8_t)ack.payload[61],
               ack.payload[62],
               ack.payload[63],
               read_be_u32(&ack.payload[76]));
        printf("imu raw: acc=%d,%d,%d gyro=%d,%d,%d\n",
               acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
    }

    close(fd);
    return 0;
}
