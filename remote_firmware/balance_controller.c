#include "balance_controller.h"

#include "fgeneric_timer.h"
#include "fparameters.h"
#include "fsleep.h"
#include "lqr_controller.h"
#include "motor_can.h"
#include "phytium_bmi088_port.h"
#include "phytium_can_port.h"

#include <math.h>
#include <string.h>

#define BALANCE_CONTROL_HZ 100U
#define BALANCE_LEFT_MOTOR_ID 1U
#define BALANCE_RIGHT_MOTOR_ID 2U
#define BALANCE_IMU_CALIBRATION_SAMPLES 100U
#define BALANCE_FEEDBACK_TIMEOUT_MS 100U
#define BALANCE_ARM_TIMEOUT_MS 2000U
#define BALANCE_ARM_MAX_WHEEL_SPEED_M_S 0.10f
#define BALANCE_ARM_MAX_PITCH_RAD (5.0f * BALANCE_PI / 180.0f)
#define BALANCE_ARM_MAX_PITCH_RATE_RAD_S 0.15f
#define BALANCE_MAX_WHEEL_SPEED_M_S 1.0f
#define BALANCE_PI 3.14159265358979323846f
#define BALANCE_IMU_MOUNT_PITCH_RAD 0.0f

static LqrController g_lqr;
static BalanceTelemetry g_telemetry;
static uint64_t g_timer_frequency;
static uint64_t g_period_ticks;
static uint64_t g_next_tick;
static uint64_t g_arm_tick;
static uint8_t g_initialized;

static uint64_t ms_to_ticks(uint32_t milliseconds)
{
    return g_timer_frequency * milliseconds / 1000U;
}

static int send_frame(const MotorCanFrame *frame)
{
    return phytium_can_send(frame);
}

static int send_torque(uint8_t motor_id, float torque_nm)
{
    MotorCanFrame frame;
    float scaled = torque_nm * 100.0f;
    int16_t torque_x100;

    if (scaled > 32767.0f) {
        scaled = 32767.0f;
    } else if (scaled < -32768.0f) {
        scaled = -32768.0f;
    }
    torque_x100 = (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
    motor_build_torque(motor_id, torque_x100, &frame);
    return send_frame(&frame);
}

static void stop_motors(uint8_t enter_idle)
{
    MotorCanFrame frame;

    (void)send_torque(BALANCE_LEFT_MOTOR_ID, 0.0f);
    (void)send_torque(BALANCE_RIGHT_MOTOR_ID, 0.0f);
    if (enter_idle) {
        motor_build_idle(BALANCE_LEFT_MOTOR_ID, &frame);
        (void)send_frame(&frame);
        motor_build_idle(BALANCE_RIGHT_MOTOR_ID, &frame);
        (void)send_frame(&frame);
    }
}

static void enter_fault(uint8_t fault)
{
    lqr_disable(&g_lqr);
    g_telemetry.fault |= fault;
    g_telemetry.state = BALANCE_STATE_FAULT;
    g_telemetry.left_torque_nm = 0.0f;
    g_telemetry.right_torque_nm = 0.0f;
    stop_motors(1U);
}

static int feedback_is_fresh(const MotorFeedback *feedback, uint64_t now)
{
    return feedback->valid &&
           now - feedback->update_tick <= ms_to_ticks(BALANCE_FEEDBACK_TIMEOUT_MS);
}

static int read_lqr_sensor(uint64_t now, LqrSensorData *sensor,
                           uint8_t *fault)
{
    PhytiumBmi088Sample imu;
    MotorFeedback left;
    MotorFeedback right;

    memset(sensor, 0, sizeof(*sensor));
    *fault = BALANCE_FAULT_NONE;
    if (phytium_bmi088_get_sample(&imu) != 0 || !imu.valid ||
        now - imu.update_tick > ms_to_ticks(30U)) {
        *fault |= BALANCE_FAULT_IMU;
    }
    if (phytium_can_get_motor_feedback(BALANCE_LEFT_MOTOR_ID, &left) != 0 ||
        !feedback_is_fresh(&left, now)) {
        *fault |= BALANCE_FAULT_LEFT_MOTOR;
    }
    if (phytium_can_get_motor_feedback(BALANCE_RIGHT_MOTOR_ID, &right) != 0 ||
        !feedback_is_fresh(&right, now)) {
        *fault |= BALANCE_FAULT_RIGHT_MOTOR;
    }
    if (*fault != BALANCE_FAULT_NONE) {
        return -1;
    }

    sensor->pitch_rad = imu.pitch_rad;
    sensor->pitch_rate_rad_s = imu.pitch_rate_rad_s;
    sensor->left_position_rad = (float)left.position_x100_deg *
                                BALANCE_PI / 18000.0f;
    sensor->right_position_rad = (float)right.position_x100_deg *
                                 BALANCE_PI / 18000.0f;
    sensor->left_velocity_rad_s = (float)left.speed_rpm *
                                  2.0f * BALANCE_PI / 60.0f;
    sensor->right_velocity_rad_s = (float)right.speed_rpm *
                                   2.0f * BALANCE_PI / 60.0f;
    sensor->valid = 1U;
    return 0;
}

int balance_control_init(void)
{
    const LqrConfig config = {
        .k_theta = -152.765302998f,
        .k_theta_rate = -13.260563186f,
        .k_position = -1.414213562f,
        .k_velocity = -5.829463511f,
        .wheel_radius_m = 0.03225f,
        .torque_limit_nm = 0.22f,
        .fall_angle_rad = 15.0f * BALANCE_PI / 180.0f,
        .pitch_offset_rad = BALANCE_IMU_MOUNT_PITCH_RAD,
        .left_motor_direction = 1.0f,
        .right_motor_direction = -1.0f,
    };

    memset(&g_telemetry, 0, sizeof(g_telemetry));
    g_telemetry.state = BALANCE_STATE_DISABLED;
    g_telemetry.control_hz = BALANCE_CONTROL_HZ;
    g_timer_frequency = GenericTimerFrequecy();
    if (g_timer_frequency == 0U || lqr_init(&g_lqr, &config) != 0) {
        g_telemetry.fault = BALANCE_FAULT_CONFIG;
        g_telemetry.state = BALANCE_STATE_FAULT;
        return -1;
    }
    g_period_ticks = g_timer_frequency / BALANCE_CONTROL_HZ;
    g_next_tick = GenericTimerRead(GENERIC_TIMER_ID0) + g_period_ticks;

    if (phytium_can_init() != 0 || phytium_bmi088_init() != 0) {
        g_telemetry.fault = BALANCE_FAULT_CAN | BALANCE_FAULT_IMU;
        g_telemetry.state = BALANCE_STATE_FAULT;
        return -1;
    }

    g_initialized = 1U;
    return 0;
}

int balance_control_enable(void)
{
    MotorCanFrame frame;
    int ret = 0;

    if (!g_initialized || g_telemetry.state == BALANCE_STATE_ACTIVE ||
        g_telemetry.state == BALANCE_STATE_ARMING) {
        return -1;
    }

    g_telemetry.fault = BALANCE_FAULT_NONE;
    lqr_disable(&g_lqr);
    if (phytium_bmi088_calibrate_gyro(BALANCE_IMU_CALIBRATION_SAMPLES) != 0) {
        enter_fault(BALANCE_FAULT_IMU);
        return -1;
    }
    motor_build_set_mode(BALANCE_LEFT_MOTOR_ID, 0U, &frame);
    ret |= send_frame(&frame);
    motor_build_set_mode(BALANCE_RIGHT_MOTOR_ID, 0U, &frame);
    ret |= send_frame(&frame);
    fsleep_millisec(5U);
    motor_build_enable(BALANCE_LEFT_MOTOR_ID, &frame);
    ret |= send_frame(&frame);
    motor_build_enable(BALANCE_RIGHT_MOTOR_ID, &frame);
    ret |= send_frame(&frame);
    fsleep_millisec(5U);
    ret |= send_torque(BALANCE_LEFT_MOTOR_ID, 0.0f);
    ret |= send_torque(BALANCE_RIGHT_MOTOR_ID, 0.0f);
    if (ret != 0) {
        enter_fault(BALANCE_FAULT_CAN);
        return -1;
    }

    g_arm_tick = GenericTimerRead(GENERIC_TIMER_ID0);
    g_telemetry.state = BALANCE_STATE_ARMING;
    return 0;
}

void balance_control_disable(void)
{
    if (!g_initialized) {
        return;
    }
    lqr_disable(&g_lqr);
    stop_motors(1U);
    g_telemetry.state = BALANCE_STATE_DISABLED;
    g_telemetry.fault = BALANCE_FAULT_NONE;
    g_telemetry.left_torque_nm = 0.0f;
    g_telemetry.right_torque_nm = 0.0f;
}

void balance_control_poll(void)
{
    uint64_t now;
    LqrSensorData sensor;
    LqrOutput output;
    uint8_t fault;

    if (!g_initialized) {
        return;
    }
    (void)phytium_can_poll();
    now = GenericTimerRead(GENERIC_TIMER_ID0);
    if ((int64_t)(now - g_next_tick) < 0) {
        return;
    }
    if (g_telemetry.state == BALANCE_STATE_ACTIVE &&
        now - g_next_tick > 2U * g_period_ticks) {
        g_next_tick = now + g_period_ticks;
        enter_fault(BALANCE_FAULT_OVERRUN);
        return;
    }
    g_next_tick += g_period_ticks;
    if (now - g_next_tick > g_period_ticks) {
        g_next_tick = now + g_period_ticks;
    }

    if (phytium_bmi088_update(1.0f / (float)BALANCE_CONTROL_HZ) != 0) {
        if (g_telemetry.state == BALANCE_STATE_ACTIVE) {
            enter_fault(BALANCE_FAULT_IMU);
        }
        return;
    }
    now = GenericTimerRead(GENERIC_TIMER_ID0);
    if (g_telemetry.state == BALANCE_STATE_DISABLED) {
        PhytiumBmi088Sample imu;
        if (phytium_bmi088_get_sample(&imu) == 0 && imu.valid) {
            g_telemetry.pitch_rad =
                imu.pitch_rad - g_lqr.config.pitch_offset_rad;
            g_telemetry.pitch_rate_rad_s = imu.pitch_rate_rad_s;
        }
        return;
    }
    if (g_telemetry.state == BALANCE_STATE_FAULT) {
        return;
    }
    if (!phytium_can_bus_ok()) {
        enter_fault(BALANCE_FAULT_CAN);
        return;
    }

    if (g_telemetry.state == BALANCE_STATE_ARMING) {
        (void)send_torque(BALANCE_LEFT_MOTOR_ID, 0.0f);
        (void)send_torque(BALANCE_RIGHT_MOTOR_ID, 0.0f);
        if (read_lqr_sensor(now, &sensor, &fault) == 0) {
            if (fabsf(sensor.pitch_rad - g_lqr.config.pitch_offset_rad) >
                    BALANCE_ARM_MAX_PITCH_RAD ||
                fabsf(sensor.pitch_rate_rad_s) >
                    BALANCE_ARM_MAX_PITCH_RATE_RAD_S ||
                fabsf(sensor.left_velocity_rad_s * g_lqr.config.wheel_radius_m) >
                    BALANCE_ARM_MAX_WHEEL_SPEED_M_S ||
                fabsf(sensor.right_velocity_rad_s * g_lqr.config.wheel_radius_m) >
                    BALANCE_ARM_MAX_WHEEL_SPEED_M_S) {
                enter_fault(BALANCE_FAULT_ARM_CONDITION);
                return;
            }
            if (lqr_enable(&g_lqr, &sensor) == 0) {
                g_telemetry.state = BALANCE_STATE_ACTIVE;
                return;
            }
            enter_fault(BALANCE_FAULT_CONFIG);
        } else if (now - g_arm_tick > ms_to_ticks(BALANCE_ARM_TIMEOUT_MS)) {
            enter_fault((uint8_t)(fault | BALANCE_FAULT_ARM_TIMEOUT));
        }
        return;
    }

    if (read_lqr_sensor(now, &sensor, &fault) != 0) {
        enter_fault(fault);
        return;
    }
    output = lqr_update(&g_lqr, &sensor);
    if (output.fault || !output.enabled) {
        enter_fault(BALANCE_FAULT_FALL);
        return;
    }
    if (fabsf(output.wheel_velocity_m_s) > BALANCE_MAX_WHEEL_SPEED_M_S) {
        enter_fault(BALANCE_FAULT_SPEED);
        return;
    }
    if (send_torque(BALANCE_LEFT_MOTOR_ID, output.left_torque_nm) != 0 ||
        send_torque(BALANCE_RIGHT_MOTOR_ID, output.right_torque_nm) != 0) {
        enter_fault(BALANCE_FAULT_CAN);
        return;
    }

    g_telemetry.pitch_rad = sensor.pitch_rad - g_lqr.config.pitch_offset_rad;
    g_telemetry.pitch_rate_rad_s = sensor.pitch_rate_rad_s;
    g_telemetry.wheel_position_m = output.wheel_position_m;
    g_telemetry.wheel_velocity_m_s = output.wheel_velocity_m_s;
    g_telemetry.left_torque_nm = output.left_torque_nm;
    g_telemetry.right_torque_nm = output.right_torque_nm;
    g_telemetry.loop_count++;
}

const BalanceTelemetry *balance_control_get_telemetry(void)
{
    return &g_telemetry;
}
