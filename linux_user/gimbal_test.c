#define _POSIX_C_SOURCE 200809L

#include "../src/rpmsg_protocol.h"

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#define GIMBAL_YAW_MOTOR_ID 3U
#define GIMBAL_PITCH_MOTOR_ID 4U

#define DEFAULT_SPEED_RPM 20U
#define DEFAULT_TORQUE_PERCENT 10U
#define DEFAULT_SWEEP_DEG 10.0
#define DEFAULT_TEST_DEG 5.0
#define DEFAULT_SWEEP_DELAY_MS 800U
#define LIMIT_FEEDBACK_RETRY_MS 100U
#define LIMIT_FEEDBACK_RETRIES 20U
#define GIMBAL_REPLY_NO_FEEDBACK 3U

static volatile sig_atomic_t g_stop_requested;
static uint8_t g_seq;

static int sleep_ms(unsigned int delay_ms);

static void request_stop(int signo)
{
    (void)signo;
    g_stop_requested = 1;
}

static void put_be_i32(uint8_t *p, int32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)value;
}

static void put_be_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

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

static int send_command_reply(int fd, uint8_t type, const uint8_t *payload,
                              uint8_t payload_len, RpmsgFrame *response)
{
    uint8_t tx[128];
    uint8_t rx[128];
    RpmsgFrame ack;
    size_t tx_len;
    ssize_t rx_len;
    int ready;

    ++g_seq;
    tx_len = rpmsg_encode(type, g_seq, payload, payload_len, tx, sizeof(tx));
    if (tx_len == 0U) {
        fprintf(stderr, "encode command failed\n");
        return -1;
    }

    if (write(fd, tx, tx_len) != (ssize_t)tx_len) {
        perror("write rpmsg");
        return -1;
    }

    ready = wait_readable(fd, 5000);
    if (ready <= 0) {
        if (ready < 0) {
            perror("select rpmsg");
        } else {
            fprintf(stderr, "remote reply timeout\n");
        }
        return -1;
    }

    rx_len = read(fd, rx, sizeof(rx));
    if (rx_len < 0) {
        perror("read rpmsg");
        return -1;
    }
    if (!rpmsg_decode(rx, (size_t)rx_len, &ack) || ack.seq != g_seq) {
        fprintf(stderr, "invalid remote reply\n");
        return -1;
    }

    if (response != NULL) {
        *response = ack;
    }

    if (ack.type == CMD_HEARTBEAT && ack.length >= 18U) {
        uint16_t frame_id = (uint16_t)(((uint16_t)ack.payload[15] << 8) |
                                       ack.payload[16]);
        printf("ack: can_init=%d send=%d count=%u last_id=0x%03x\n",
               (int8_t)ack.payload[4], (int8_t)ack.payload[5],
               ((uint32_t)ack.payload[11] << 24) |
               ((uint32_t)ack.payload[12] << 16) |
               ((uint32_t)ack.payload[13] << 8) |
               ack.payload[14], frame_id);
    }
    return 0;
}

static int send_command(int fd, uint8_t type, const uint8_t *payload,
                        uint8_t payload_len)
{
    return send_command_reply(fd, type, payload, payload_len, NULL);
}

static int send_set_origin(int fd, uint8_t motor_id)
{
    return send_command(fd, CMD_CAN_SET_ORIGIN, &motor_id, 1);
}

static int32_t read_be_i32(const uint8_t *p)
{
    return ((int32_t)p[0] << 24) | ((int32_t)p[1] << 16) |
           ((int32_t)p[2] << 8) | (int32_t)p[3];
}

static uint32_t read_be_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static uint16_t read_be_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static const char *gimbal_state_name(uint8_t state)
{
    static const char *const names[] = {
        "disabled", "starting", "homing", "active", "stopping", "fault"
    };
    return state < sizeof(names) / sizeof(names[0]) ? names[state] : "unknown";
}

static const char *gimbal_status_name(uint8_t status)
{
    static const char *const names[] = {
        "ok", "invalid", "busy", "no-feedback", "limits-not-ready",
        "can-error"
    };
    return status < sizeof(names) / sizeof(names[0]) ? names[status] : "unknown";
}

static int print_gimbal_reply(const RpmsgFrame *reply)
{
    const uint8_t *p;
    uint8_t status;

    if (reply->length < GIMBAL_TELEMETRY_PAYLOAD_SIZE ||
        reply->payload[0] != GIMBAL_TELEMETRY_VERSION) {
        fprintf(stderr, "invalid gimbal reply\n");
        return -1;
    }
    p = reply->payload;
    status = p[1];
    printf("gimbal: status=%s(%u) state=%s(%u) fault=0x%02x\n",
           gimbal_status_name(status), status, gimbal_state_name(p[2]), p[2],
           p[3]);
    printf("feedback: valid=0x%02x yaw=%.2f deg %.0f rpm %.2f A age=%u ms; "
           "pitch=%.2f deg %.0f rpm %.2f A age=%u ms\n",
           p[5], (double)read_be_i32(&p[8]) / 100.0,
           (double)(int16_t)read_be_u16(&p[16]),
           (double)(int16_t)read_be_u16(&p[20]) / 100.0,
           read_be_u32(&p[52]),
           (double)read_be_i32(&p[12]) / 100.0,
           (double)(int16_t)read_be_u16(&p[18]),
           (double)(int16_t)read_be_u16(&p[22]) / 100.0,
           read_be_u32(&p[56]));
    printf("target: yaw=%.2f deg pitch=%.2f deg speed=%u rpm torque=%u%% "
           "timeout_remaining=%u ms\n",
           (double)read_be_i32(&p[24]) / 100.0,
           (double)read_be_i32(&p[28]) / 100.0,
           read_be_u16(&p[48]), p[6], read_be_u32(&p[60]));
    printf("limits: valid=0x%02x yaw=[%.2f, %.2f] deg "
           "pitch=[%.2f, %.2f] deg\n",
           p[4], (double)read_be_i32(&p[32]) / 100.0,
           (double)read_be_i32(&p[36]) / 100.0,
           (double)read_be_i32(&p[40]) / 100.0,
           (double)read_be_i32(&p[44]) / 100.0);
    return status == 0U ? 0 : -1;
}

static int send_gimbal_command(int fd, uint8_t type, const uint8_t *payload,
                               uint8_t payload_len)
{
    RpmsgFrame reply;
    if (send_command_reply(fd, type, payload, payload_len, &reply) != 0 ||
        reply.type != type) {
        return -1;
    }
    return print_gimbal_reply(&reply);
}

static int send_gimbal_command_with_feedback_retry(
    int fd, uint8_t type, const uint8_t *payload, uint8_t payload_len,
    const char *action)
{
    RpmsgFrame reply;
    uint8_t status = 0xffU;
    int ret;

    for (unsigned int attempt = 0U; attempt <= LIMIT_FEEDBACK_RETRIES;
         ++attempt) {
        if (attempt != 0U && sleep_ms(LIMIT_FEEDBACK_RETRY_MS) != 0) {
            return -1;
        }
        if (send_command_reply(fd, type, payload, payload_len, &reply) != 0 ||
            reply.type != type ||
            reply.length < GIMBAL_TELEMETRY_PAYLOAD_SIZE) {
            return -1;
        }
        status = reply.payload[1];
        if (status != GIMBAL_REPLY_NO_FEEDBACK) {
            ret = print_gimbal_reply(&reply);
            return ret;
        }
        if (attempt == 0U) {
            (void)print_gimbal_reply(&reply);
            printf("Waking both axes in zero-torque mode for %s; "
                   "support the camera because holding torque is zero...\n",
                   action);
        }
        if (attempt == LIMIT_FEEDBACK_RETRIES) {
            fprintf(stderr, "motor feedback did not start within %u ms\n",
                    LIMIT_FEEDBACK_RETRIES * LIMIT_FEEDBACK_RETRY_MS);
            return -1;
        }
    }
    return -1;
}

static int calibrate_gimbal_limit(int fd, const uint8_t *payload)
{
    return send_gimbal_command_with_feedback_retry(
        fd, CMD_GIMBAL_CALIBRATE_LIMIT, payload, 2U, "limit calibration");
}

static int degrees_to_x100(double degrees, int32_t *result)
{
    double scaled = degrees * 100.0;
    if (scaled < -2147483648.0 || scaled > 2147483647.0) {
        return -1;
    }
    *result = (int32_t)(scaled + (scaled >= 0.0 ? 0.5 : -0.5));
    return 0;
}

static int set_gimbal(int fd, double yaw_deg, double pitch_deg,
                      uint16_t speed_rpm, uint8_t torque_percent)
{
    uint8_t payload[13];
    int32_t yaw_x100;
    int32_t pitch_x100;

    printf("set yaw(id=3)=%.2f deg pitch(id=4)=%.2f deg speed=%u torque=%u%%\n",
           yaw_deg, pitch_deg, speed_rpm, torque_percent);
    if (degrees_to_x100(yaw_deg, &yaw_x100) != 0 ||
        degrees_to_x100(pitch_deg, &pitch_x100) != 0) {
        return -1;
    }
    put_be_i32(&payload[0], yaw_x100);
    put_be_i32(&payload[4], pitch_x100);
    put_be_u16(&payload[8], speed_rpm);
    payload[10] = torque_percent;
    put_be_u16(&payload[11], 0U);
    return send_gimbal_command(fd, CMD_GIMBAL_SET_TARGET, payload,
                               sizeof(payload));
}

static int sleep_ms(unsigned int delay_ms)
{
    struct timespec delay = {
        .tv_sec = delay_ms / 1000U,
        .tv_nsec = (long)(delay_ms % 1000U) * 1000000L,
    };

    while (nanosleep(&delay, &delay) != 0) {
        if (errno != EINTR || g_stop_requested) {
            return -1;
        }
    }
    return 0;
}

static int run_sweep(int fd, double angle_deg, unsigned int cycles)
{
    static const double path[][2] = {
        {1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}, {0.0, -1.0}, {0.0, 0.0},
    };

    if (angle_deg <= 0.0 || angle_deg > 20.0 || cycles == 0U || cycles > 20U) {
        fprintf(stderr, "sweep requires angle 0..20 and cycles 1..20\n");
        return -1;
    }

    for (unsigned int cycle = 0; cycle < cycles && !g_stop_requested; ++cycle) {
        for (size_t i = 0; i < sizeof(path) / sizeof(path[0]); ++i) {
            if (set_gimbal(fd, path[i][0] * angle_deg, path[i][1] * angle_deg,
                           DEFAULT_SPEED_RPM, DEFAULT_TORQUE_PERCENT) != 0) {
                return -1;
            }
            if (sleep_ms(DEFAULT_SWEEP_DELAY_MS) != 0) {
                break;
            }
        }
    }
    return g_stop_requested ? -1 : 0;
}

static int run_continuous_test(int fd, double angle_deg)
{
    if (angle_deg <= 0.0 || angle_deg > 10.0) {
        fprintf(stderr, "test requires angle 0..10\n");
        return -1;
    }

    printf("continuous gimbal test: +/-%.2f deg; press Ctrl+C to stop\n",
           angle_deg);
    while (!g_stop_requested) {
        if (run_sweep(fd, angle_deg, 1U) != 0) {
            return g_stop_requested ? 0 : -1;
        }
    }
    return 0;
}

static int parse_double(const char *text, double *value)
{
    char *end;
    errno = 0;
    *value = strtod(text, &end);
    return errno == 0 && end != text && *end == '\0' ? 0 : -1;
}

static int parse_uint(const char *text, unsigned long max, unsigned long *value)
{
    char *end;
    errno = 0;
    *value = strtoul(text, &end, 0);
    return errno == 0 && end != text && *end == '\0' && *value <= max ? 0 : -1;
}

static void usage(const char *program)
{
    printf("Usage:\n");
    printf("  %s <rpmsg_dev> setzero CONFIRM\n", program);
    printf("  %s <rpmsg_dev> enable|init\n", program);
    printf("  %s <rpmsg_dev> disable|stop\n", program);
    printf("  %s <rpmsg_dev> estop\n", program);
    printf("  %s <rpmsg_dev> set <yaw_deg> <pitch_deg> [speed_rpm] [torque_pct]\n", program);
    printf("  %s <rpmsg_dev> center\n", program);
    printf("  %s <rpmsg_dev> sweep [angle_deg] [cycles]\n", program);
    printf("  %s <rpmsg_dev> test [angle_deg]\n", program);
    printf("  %s <rpmsg_dev> status\n", program);
    printf("  %s <rpmsg_dev> limit <yaw|pitch> <min|max> CONFIRM\n", program);
    printf("  %s <rpmsg_dev> limits <yaw_min> <yaw_max> <pitch_min> <pitch_max> CONFIRM\n", program);
    printf("  %s <rpmsg_dev> reset-limits CONFIRM\n", program);
}

int main(int argc, char **argv)
{
    const char *device;
    const char *command;
    int fd;
    int ret = -1;

    if (argc < 3) {
        usage(argv[0]);
        return 1;
    }
    device = argv[1];
    command = argv[2];
    signal(SIGINT, request_stop);
    signal(SIGTERM, request_stop);

    fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("open rpmsg device");
        return 1;
    }

    if (strcmp(command, "setzero") == 0) {
        if (argc < 4 || strcmp(argv[3], "CONFIRM") != 0) {
            fprintf(stderr, "setzero permanently saves both current positions; use setzero CONFIRM\n");
        } else {
            printf("Saving yaw(id=3) and pitch(id=4) current positions as permanent zero.\n");
            ret = send_set_origin(fd, GIMBAL_YAW_MOTOR_ID);
            if (ret == 0) {
                ret = send_set_origin(fd, GIMBAL_PITCH_MOTOR_ID);
            }
            if (ret == 0) {
                printf("Waiting for both motor drivers to restart...\n");
                ret = sleep_ms(2000U);
            }
        }
    } else if (strcmp(command, "init") == 0 || strcmp(command, "enable") == 0) {
        printf("Starting gimbal and slowly homing both axes to zero.\n");
        ret = send_gimbal_command_with_feedback_retry(
            fd, CMD_GIMBAL_ENABLE, NULL, 0U, "safe startup");
    } else if (strcmp(command, "set") == 0 && argc >= 5) {
        double yaw;
        double pitch;
        unsigned long speed = DEFAULT_SPEED_RPM;
        unsigned long torque = DEFAULT_TORQUE_PERCENT;
        if (parse_double(argv[3], &yaw) != 0 || parse_double(argv[4], &pitch) != 0 ||
            (argc >= 6 && parse_uint(argv[5], 1000U, &speed) != 0) ||
            (argc >= 7 && parse_uint(argv[6], 100U, &torque) != 0)) {
            fprintf(stderr, "invalid set argument\n");
        } else {
            ret = set_gimbal(fd, yaw, pitch, (uint16_t)speed, (uint8_t)torque);
        }
    } else if (strcmp(command, "center") == 0) {
        ret = set_gimbal(fd, 0.0, 0.0, DEFAULT_SPEED_RPM,
                         DEFAULT_TORQUE_PERCENT);
    } else if (strcmp(command, "sweep") == 0) {
        double angle = DEFAULT_SWEEP_DEG;
        unsigned long cycles = 1;
        if ((argc >= 4 && parse_double(argv[3], &angle) != 0) ||
            (argc >= 5 && parse_uint(argv[4], 20U, &cycles) != 0)) {
            fprintf(stderr, "invalid sweep argument\n");
        } else {
            ret = run_sweep(fd, angle, (unsigned int)cycles);
        }
    } else if (strcmp(command, "test") == 0) {
        double angle = DEFAULT_TEST_DEG;
        if (argc >= 4 && parse_double(argv[3], &angle) != 0) {
            fprintf(stderr, "invalid test angle\n");
        } else {
            ret = run_continuous_test(fd, angle);
        }
    } else if (strcmp(command, "stop") == 0 ||
               strcmp(command, "disable") == 0) {
        printf("Holding current pose, ramping torque down, then entering idle.\n");
        ret = send_gimbal_command(fd, CMD_GIMBAL_DISABLE, NULL, 0U);
    } else if (strcmp(command, "estop") == 0) {
        ret = send_gimbal_command(fd, CMD_GIMBAL_EMERGENCY_STOP, NULL, 0U);
    } else if (strcmp(command, "status") == 0) {
        ret = send_gimbal_command(fd, CMD_GIMBAL_STATUS, NULL, 0U);
    } else if (strcmp(command, "limit") == 0) {
        uint8_t payload[2];
        if (argc != 6 || strcmp(argv[5], "CONFIRM") != 0 ||
            (strcmp(argv[3], "yaw") != 0 && strcmp(argv[3], "pitch") != 0) ||
            (strcmp(argv[4], "min") != 0 && strcmp(argv[4], "max") != 0)) {
            fprintf(stderr, "use: limit <yaw|pitch> <min|max> CONFIRM\n");
        } else {
            payload[0] = strcmp(argv[3], "pitch") == 0 ? 1U : 0U;
            payload[1] = strcmp(argv[4], "max") == 0 ? 1U : 0U;
            ret = calibrate_gimbal_limit(fd, payload);
        }
    } else if (strcmp(command, "limits") == 0) {
        uint8_t payload[16];
        double values[4];
        int32_t scaled[4];
        int valid = argc == 8 && strcmp(argv[7], "CONFIRM") == 0;
        for (int i = 0; i < 4 && valid; ++i) {
            valid = parse_double(argv[3 + i], &values[i]) == 0 &&
                    degrees_to_x100(values[i], &scaled[i]) == 0;
        }
        if (!valid) {
            fprintf(stderr, "use: limits <yaw_min> <yaw_max> <pitch_min> <pitch_max> CONFIRM\n");
        } else {
            for (int i = 0; i < 4; ++i) {
                put_be_i32(&payload[i * 4], scaled[i]);
            }
            ret = send_gimbal_command(fd, CMD_GIMBAL_SET_LIMITS,
                                      payload, sizeof(payload));
        }
    } else if (strcmp(command, "reset-limits") == 0) {
        if (argc != 4 || strcmp(argv[3], "CONFIRM") != 0) {
            fprintf(stderr, "use: reset-limits CONFIRM\n");
        } else {
            ret = send_gimbal_command(fd, CMD_GIMBAL_RESET_LIMITS, NULL, 0U);
        }
    } else {
        usage(argv[0]);
    }

    if (g_stop_requested) {
        fprintf(stderr, "requesting controlled gimbal shutdown\n");
        (void)send_gimbal_command(fd, CMD_GIMBAL_DISABLE, NULL, 0U);
    }
    close(fd);
    return ret == 0 ? 0 : 1;
}
