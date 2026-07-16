#include "../src/rpmsg_protocol.h"

#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#define RPMSG_CLIENT_VERSION "0.17.1-lqr-speed-verified"
#define RAD_PER_DEG 0.017453292519943295f

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

static int32_t scaled_i32(float value, float scale)
{
    float scaled = value * scale;
    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static uint32_t read_be_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           (uint32_t)p[3];
}

static int32_t read_be_i32(const uint8_t *p)
{
    return (int32_t)read_be_u32(p);
}

static uint16_t read_be_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void print_motor_fault_bits(uint32_t code)
{
    static const struct {
        uint32_t bit;
        const char *name;
    } faults[] = {
        {0x00000001U, "power/calibration-current"},
        {0x00000002U, "phase-resistance"},
        {0x00000008U, "calibration-current-ripple"},
        {0x00000010U, "phase-inductance"},
        {0x00000020U, "encoder-bandwidth"},
        {0x00000040U, "encoder-spi"},
        {0x00000080U, "encoder-type"},
        {0x00000100U, "hall-not-calibrated"},
        {0x00000200U, "encoder-no-data"},
        {0x00000400U, "encoder-cpr"},
        {0x00000800U, "run-state"},
        {0x00008000U, "hall-signal"},
        {0x00020000U, "secondary-encoder"},
        {0x00080000U, "gate-driver"},
        {0x00100000U, "mos-overtemperature"},
        {0x00200000U, "motor-overtemperature"},
        {0x00400000U, "undervoltage"},
        {0x00800000U, "overvoltage"},
        {0x01000000U, "overcurrent"},
    };
    int found = 0;

    if (code == 0U) {
        printf("motor fault decoded: none\n");
        return;
    }
    printf("motor fault decoded:");
    for (size_t i = 0; i < sizeof(faults) / sizeof(faults[0]); ++i) {
        if ((code & faults[i].bit) != 0U) {
            printf(" %s", faults[i].name);
            found = 1;
        }
    }
    if (!found) {
        printf(" unknown-bits");
    }
    printf("\n");
}

static void usage(const char *prog)
{
    printf("Usage:\n");
    printf("  %s <rpmsg_dev> heartbeat\n", prog);
    printf("  %s <rpmsg_dev> enable <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> zero <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> setorigin <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> mode <motor_id> <mode>\n", prog);
    printf("  %s <rpmsg_dev> init <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> test\n", prog);
    printf("  %s <rpmsg_dev> torque-test <motor_id> <torque_nm> [duration_ms]\n", prog);
    printf("  %s <rpmsg_dev> motor-fault <motor_id>\n", prog);
    printf("  %s <rpmsg_dev> motor-speed-diag <motor_id> <torque_nm> [duration_ms]\n", prog);
    printf("  %s <rpmsg_dev> servo <s0_deg> <s1_deg> <s2_deg> <s3_deg>\n", prog);
    printf("  %s <rpmsg_dev> servopol <0..7>\n", prog);
    printf("  %s <rpmsg_dev> servocenter\n", prog);
    printf("  %s <rpmsg_dev> imuinit\n", prog);
    printf("  %s <rpmsg_dev> imuread\n", prog);
    printf("  %s <rpmsg_dev> balance-enable\n", prog);
    printf("  %s <rpmsg_dev> balance-disable\n", prog);
    printf("  %s <rpmsg_dev> balance-status\n", prog);
    printf("  %s <rpmsg_dev> balance-trim <upright_pitch_deg>\n", prog);
    printf("  %s <rpmsg_dev> balance-gains <k_theta> <k_theta_rate> <k_position> <k_velocity>\n", prog);
    printf("  %s <rpmsg_dev> balance-config\n", prog);
    printf("  %s <rpmsg_dev> balance-reset-config\n", prog);
    printf("  %s <rpmsg_dev> balance-speed-limit <m_s>\n", prog);
    printf("  %s <rpmsg_dev> pvt <motor_id> <pos_x100_deg> <speed_rpm> <torque_percent>\n", prog);
    printf("  %s <rpmsg_dev> stop [motor_id]\n", prog);
    printf("\nExamples:\n");
    printf("  %s /dev/rpmsg0 heartbeat\n", prog);
    printf("  %s /dev/rpmsg0 init 1\n", prog);
    printf("  %s /dev/rpmsg0 test\n", prog);
    printf("  %s /dev/rpmsg0 servo 90 90 90 90\n", prog);
    printf("  %s /dev/rpmsg0 servopol 4\n", prog);
    printf("  %s /dev/rpmsg0 servocenter\n", prog);
    printf("  %s /dev/rpmsg0 imuinit\n", prog);
    printf("  %s /dev/rpmsg0 imuread\n", prog);
    printf("  watch -n 0.1 '%s /dev/rpmsg0 imuread'\n", prog);
    printf("  %s /dev/rpmsg0 balance-enable\n", prog);
    printf("  watch -n 0.1 '%s /dev/rpmsg0 balance-status'\n", prog);
    printf("  %s /dev/rpmsg0 balance-disable\n", prog);
    printf("  %s /dev/rpmsg0 balance-trim 1.0\n", prog);
    printf("  %s /dev/rpmsg0 balance-config\n", prog);
    printf("  %s /dev/rpmsg0 balance-speed-limit 1.2\n", prog);
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

    if (strcmp(cmd, "setorigin") == 0) {
        if (argc < 4) return -1;
        *type = CMD_CAN_SET_ORIGIN;
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

    if (strcmp(cmd, "test") == 0) {
        *type = CMD_MOTOR_TEST;
        *payload_len = 0;
        return 0;
    }

    if (strcmp(cmd, "torque-test") == 0) {
        char *end = NULL;
        unsigned long motor_id;
        unsigned long duration_ms = 200U;
        float torque_nm;
        int16_t torque_x100;

        if (argc < 5) return -1;
        motor_id = strtoul(argv[3], &end, 0);
        if (end == argv[3] || *end != '\0' || motor_id < 1U || motor_id > 2U) {
            return -1;
        }
        end = NULL;
        torque_nm = strtof(argv[4], &end);
        if (end == argv[4] || *end != '\0' || !isfinite(torque_nm) ||
            torque_nm < -0.22f || torque_nm > 0.22f) {
            return -1;
        }
        if (argc >= 6) {
            end = NULL;
            duration_ms = strtoul(argv[5], &end, 0);
            if (end == argv[5] || *end != '\0' ||
                duration_ms < 20U || duration_ms > 2000U) {
                return -1;
            }
        }
        torque_x100 = (int16_t)(torque_nm >= 0.0f ?
            torque_nm * 100.0f + 0.5f : torque_nm * 100.0f - 0.5f);
        *type = CMD_CAN_TORQUE_TEST;
        payload[0] = (uint8_t)motor_id;
        put_be_u16(&payload[1], (uint16_t)torque_x100);
        put_be_u16(&payload[3], (uint16_t)duration_ms);
        *payload_len = 5U;
        return 0;
    }

    if (strcmp(cmd, "motor-fault") == 0) {
        char *end = NULL;
        unsigned long motor_id;

        if (argc < 4) return -1;
        motor_id = strtoul(argv[3], &end, 0);
        if (end == argv[3] || *end != '\0' || motor_id < 1U || motor_id > 2U) {
            return -1;
        }
        *type = CMD_CAN_MOTOR_FAULT;
        payload[0] = (uint8_t)motor_id;
        *payload_len = 1U;
        return 0;
    }

    if (strcmp(cmd, "motor-speed-diag") == 0) {
        char *end = NULL;
        unsigned long motor_id;
        unsigned long duration_ms = 1000U;
        float torque_nm;
        int16_t torque_x100;

        if (argc < 5) return -1;
        motor_id = strtoul(argv[3], &end, 0);
        if (end == argv[3] || *end != '\0' || motor_id < 1U || motor_id > 2U) {
            return -1;
        }
        end = NULL;
        torque_nm = strtof(argv[4], &end);
        if (end == argv[4] || *end != '\0' || !isfinite(torque_nm) ||
            torque_nm < -0.10f || torque_nm > 0.10f) {
            return -1;
        }
        if (argc >= 6) {
            end = NULL;
            duration_ms = strtoul(argv[5], &end, 0);
            if (end == argv[5] || *end != '\0' ||
                duration_ms < 200U || duration_ms > 2000U) {
                return -1;
            }
        }
        torque_x100 = (int16_t)(torque_nm >= 0.0f ?
            torque_nm * 100.0f + 0.5f : torque_nm * 100.0f - 0.5f);
        *type = CMD_CAN_SPEED_DIAG;
        payload[0] = (uint8_t)motor_id;
        put_be_u16(&payload[1], (uint16_t)torque_x100);
        put_be_u16(&payload[3], (uint16_t)duration_ms);
        *payload_len = 5U;
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

    if (strcmp(cmd, "servopol") == 0) {
        if (argc < 4) {
            return -1;
        }
        *type = CMD_SERVO_POLARITY;
        payload[0] = (uint8_t)strtoul(argv[3], NULL, 0);
        *payload_len = 1;
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

    if (strcmp(cmd, "balance-enable") == 0) {
        *type = CMD_BALANCE_ENABLE;
        *payload_len = 0;
        return 0;
    }

    if (strcmp(cmd, "balance-disable") == 0) {
        *type = CMD_BALANCE_DISABLE;
        *payload_len = 0;
        return 0;
    }

    if (strcmp(cmd, "balance-status") == 0) {
        *type = CMD_BALANCE_STATUS;
        *payload_len = 0;
        return 0;
    }

    if (strcmp(cmd, "balance-trim") == 0) {
        char *end = NULL;
        float trim_deg;

        if (argc < 4) return -1;
        trim_deg = strtof(argv[3], &end);
        if (end == argv[3] || *end != '\0' || !isfinite(trim_deg) ||
            trim_deg < -5.0f || trim_deg > 5.0f) {
            return -1;
        }
        *type = CMD_BALANCE_SET_TRIM;
        put_be_i32(payload, scaled_i32(trim_deg * RAD_PER_DEG, 1000000.0f));
        *payload_len = 4U;
        return 0;
    }

    if (strcmp(cmd, "balance-gains") == 0) {
        float gains[4];
        static const float minimum[4] = {-10.0f, -5.0f, -2.0f, -2.0f};
        static const float maximum[4] = {-0.1f, 0.0f, 0.0f, 0.0f};

        if (argc < 7) return -1;
        for (int i = 0; i < 4; ++i) {
            char *end = NULL;
            gains[i] = strtof(argv[3 + i], &end);
            if (end == argv[3 + i] || *end != '\0' || !isfinite(gains[i]) ||
                gains[i] < minimum[i] || gains[i] > maximum[i]) {
                return -1;
            }
            put_be_i32(&payload[i * 4], scaled_i32(gains[i], 1000000.0f));
        }
        *type = CMD_BALANCE_SET_GAINS;
        *payload_len = 16U;
        return 0;
    }

    if (strcmp(cmd, "balance-config") == 0) {
        *type = CMD_BALANCE_CONFIG;
        *payload_len = 0U;
        return 0;
    }

    if (strcmp(cmd, "balance-reset-config") == 0) {
        *type = CMD_BALANCE_RESET_CONFIG;
        *payload_len = 0U;
        return 0;
    }

    if (strcmp(cmd, "balance-speed-limit") == 0) {
        char *end = NULL;
        float speed_limit;

        if (argc < 4) return -1;
        speed_limit = strtof(argv[3], &end);
        if (end == argv[3] || *end != '\0' || !isfinite(speed_limit) ||
            speed_limit < 0.5f || speed_limit > 1.5f) {
            return -1;
        }
        *type = CMD_BALANCE_SET_SPEED_LIMIT;
        put_be_i32(payload, scaled_i32(speed_limit, 1000000.0f));
        *payload_len = 4U;
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
    if (type == CMD_CAN_SPEED_DIAG) {
        static const char *const status_names[] = {
            "ok", "invalid-or-busy", "can-error", "incomplete"
        };
        int32_t periodic;
        int32_t register_speed;
        int32_t position_speed;
        uint8_t status;

        if (ack.type != CMD_CAN_SPEED_DIAG || ack.length < 20U ||
            ack.payload[0] != 1U) {
            printf("invalid motor speed diagnostic reply\n");
            close(fd);
            return 3;
        }
        periodic = read_be_i32(&ack.payload[4]);
        register_speed = read_be_i32(&ack.payload[8]);
        position_speed = read_be_i32(&ack.payload[12]);
        status = ack.payload[1];
        printf("motor speed diagnostic: status=%s(%u) motor_id=%u valid=0x%02X samples=%u\n",
               status < 4U ? status_names[status] : "unknown", status,
               ack.payload[2], ack.payload[3],
               read_be_u32(&ack.payload[16]));
        printf("speed comparison: periodic_0x2A=%.2f rpm register_0x0006=%.2f rpm position_delta=%.2f rpm\n",
               (double)periodic / 100.0,
               (double)register_speed / 100.0,
               (double)position_speed / 100.0);
        if (periodic != 0) {
            printf("speed ratios: register/periodic=%.4f position/periodic=%.4f\n",
                   (double)register_speed / (double)periodic,
                   (double)position_speed / (double)periodic);
        }
        close(fd);
        return status == 0U ? 0 : 4;
    }
    if (type >= CMD_BALANCE_SET_TRIM &&
        type <= CMD_BALANCE_SET_SPEED_LIMIT) {
        static const char *const status_names[] = {
            "ok", "invalid", "busy"
        };
        uint8_t status;
        const char *status_name;

        if (ack.length < 32U || ack.payload[0] < 2U) {
            printf("invalid balance config reply\n");
            close(fd);
            return 3;
        }
        status = ack.payload[1];
        status_name = status < 3U ? status_names[status] : "unknown";
        printf("balance config: status=%s(%u) state=%u\n",
               status_name, status, ack.payload[2]);
        printf("balance trim: %.6f deg (%.6f rad)\n",
               (double)read_be_i32(&ack.payload[4]) / 1000000.0 /
                   RAD_PER_DEG,
               (double)read_be_i32(&ack.payload[4]) / 1000000.0);
        printf("balance gains: K1=%.6f K2=%.6f K3=%.6f K4=%.6f\n",
               (double)read_be_i32(&ack.payload[8]) / 1000000.0,
               (double)read_be_i32(&ack.payload[12]) / 1000000.0,
               (double)read_be_i32(&ack.payload[16]) / 1000000.0,
               (double)read_be_i32(&ack.payload[20]) / 1000000.0);
        printf("posture priority angle: %.6f deg\n",
               (double)read_be_i32(&ack.payload[24]) / 1000000.0 /
                   RAD_PER_DEG);
        printf("wheel speed limit: %.6f m/s (%.1f rpm)\n",
               (double)read_be_i32(&ack.payload[28]) / 1000000.0,
               (double)read_be_i32(&ack.payload[28]) / 1000000.0 /
                   0.03225 * 60.0 / (2.0 * 3.14159265358979323846));
        if (ack.length >= 40U && ack.payload[0] >= 3U) {
            printf("motor feedback speed scale: %.6f\n",
                   (double)read_be_i32(&ack.payload[32]) / 1000000.0);
            printf("pitch-rate low-pass cutoff: %.3f Hz\n",
                   (double)read_be_i32(&ack.payload[36]) / 1000000.0);
        }
        close(fd);
        return status == 0U ? 0 : 4;
    }
    if (ack.length >= 4) {
        printf("remote state: heartbeat_ok=%u last_can_ret=%d can_rx=%u feedback=%u\n",
               ack.payload[2], (int8_t)ack.payload[3],
               ack.payload[0], ack.payload[1]);
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
        if (ack.length >= 86 && type == CMD_CAN_MOTOR_FAULT) {
            uint32_t fault_code = read_be_u32(&ack.payload[80]);
            uint8_t motor_id = ack.payload[84];
            int8_t read_ret = (int8_t)ack.payload[85];

            printf("motor fault: id=%u read_ret=%d code=0x%08X\n",
                   motor_id, read_ret, fault_code);
            if (read_ret == 0) {
                print_motor_fault_bits(fault_code);
            }
        } else if (ack.length >= 88 &&
            (type == CMD_CAN_TORQUE_TEST ||
             (type >= CMD_BALANCE_ENABLE && type <= CMD_BALANCE_STATUS))) {
            int16_t left_current_x100 = (int16_t)read_be_u16(&ack.payload[80]);
            int16_t right_current_x100 = (int16_t)read_be_u16(&ack.payload[82]);
            int16_t left_speed_rpm = (int16_t)read_be_u16(&ack.payload[84]);
            int16_t right_speed_rpm = (int16_t)read_be_u16(&ack.payload[86]);

            printf("motor feedback%s: current=%.2f,%.2f A speed=%d,%d rpm\n",
                   type == CMD_CAN_TORQUE_TEST ? " peak" : "",
                   (double)left_current_x100 / 100.0,
                   (double)right_current_x100 / 100.0,
                   left_speed_rpm, right_speed_rpm);
        } else if (ack.length >= 88) {
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

    if (ack.length >= 120) {
        static const char *const state_names[] = {
            "disabled", "arming", "active", "fault"
        };
        uint8_t state = ack.payload[88];
        uint8_t fault = ack.payload[89];
        const char *state_name = state < 4U ? state_names[state] : "unknown";

        printf("balance: state=%s(%u) fault=0x%02X control_hz=%u loop_count=%u\n",
               state_name, state, fault, read_be_u16(&ack.payload[90]),
               read_be_u32(&ack.payload[116]));
        printf("balance state: pitch=%.6f rad pitch_rate=%.6f rad/s position=%.6f m velocity=%.6f m/s\n",
               (double)read_be_i32(&ack.payload[92]) / 1000000.0,
               (double)read_be_i32(&ack.payload[96]) / 1000000.0,
               (double)read_be_i32(&ack.payload[100]) / 1000000.0,
               (double)read_be_i32(&ack.payload[104]) / 1000000.0);
        printf("balance output: left=%.6f Nm right=%.6f Nm\n",
               (double)read_be_i32(&ack.payload[108]) / 1000000.0,
               (double)read_be_i32(&ack.payload[112]) / 1000000.0);
        if (fault != 0U) {
            printf("balance fault bits: imu=%u left_motor=%u right_motor=%u can=%u fall=%u overrun=%u arm_timeout=%u arm_speed_or_config=%u\n",
                   !!(fault & 0x01U), !!(fault & 0x02U),
                   !!(fault & 0x04U), !!(fault & 0x08U),
                   !!(fault & 0x10U), !!(fault & 0x20U),
                   !!(fault & 0x40U), !!(fault & 0x80U));
        }
    }

    close(fd);
    return 0;
}
