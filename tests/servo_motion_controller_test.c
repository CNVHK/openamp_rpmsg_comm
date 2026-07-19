#include "servo_motion_controller.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define TEST_TIMER_HZ 1000000U

static uint64_t g_now;
static unsigned int g_write_count;
static PhytiumServoDebugState g_debug;

uint64_t GenericTimerRead(uint32_t timer_id)
{
    (void)timer_id;
    return g_now;
}

uint64_t GenericTimerFrequecy(void)
{
    return TEST_TIMER_HZ;
}

const PhytiumServoDebugState *phytium_servo_get_debug_state(void)
{
    return &g_debug;
}

int phytium_servo_set_all_x10(
    const uint16_t angle_x10_deg[PHYTIUM_SERVO_NUM])
{
    ++g_write_count;
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        g_debug.angle_x10_deg[i] = angle_x10_deg[i];
        g_debug.angle_deg[i] = angle_x10_deg[i] / 10U;
        g_debug.pulse_us[i] = (uint16_t)(500U +
            (2000U * angle_x10_deg[i]) / 1800U);
    }
    return 0;
}

static void advance_ms(uint32_t milliseconds)
{
    g_now += (uint64_t)milliseconds * TEST_TIMER_HZ / 1000U;
}

static void reset_fixture(void)
{
    memset(&g_debug, 0, sizeof(g_debug));
    g_now = 1000U;
    g_write_count = 0U;
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        g_debug.angle_deg[i] = 90U;
        g_debug.angle_x10_deg[i] = 900U;
        g_debug.pulse_us[i] = 1500U;
    }
    assert(servo_motion_init() == 0);
}

static void test_synchronized_interpolation(void)
{
    const uint16_t target[PHYTIUM_SERVO_NUM] = {1000U, 800U, 1100U, 700U};
    const ServoMotionTelemetry *telemetry;

    reset_fixture();
    assert(servo_motion_start(target, 1000U) == SERVO_MOTION_OK);
    assert(servo_motion_start(target, 1000U) == SERVO_MOTION_BUSY);
    servo_motion_poll();
    assert(g_write_count == 1U);

    advance_ms(500U);
    servo_motion_poll();
    telemetry = servo_motion_get_telemetry();
    assert(telemetry->current_angle_x10_deg[0] == 950U);
    assert(telemetry->current_angle_x10_deg[1] == 850U);
    assert(telemetry->current_angle_x10_deg[2] == 1000U);
    assert(telemetry->current_angle_x10_deg[3] == 800U);
    assert(telemetry->remaining_ms == 500U);

    advance_ms(500U);
    servo_motion_poll();
    telemetry = servo_motion_get_telemetry();
    assert(telemetry->state == SERVO_MOTION_IDLE);
    assert(telemetry->remaining_ms == 0U);
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        assert(telemetry->current_angle_x10_deg[i] == target[i]);
    }
}

static void test_validation_and_stop(void)
{
    const uint16_t valid[PHYTIUM_SERVO_NUM] = {900U, 900U, 900U, 900U};
    const uint16_t invalid[PHYTIUM_SERVO_NUM] = {900U, 1801U, 900U, 900U};

    reset_fixture();
    assert(servo_motion_start(valid, 99U) == SERVO_MOTION_INVALID);
    assert(servo_motion_start(invalid, 1000U) == SERVO_MOTION_INVALID);
    assert(servo_motion_start(valid, 1000U) == SERVO_MOTION_OK);
    servo_motion_stop();
    assert(servo_motion_get_telemetry()->state == SERVO_MOTION_IDLE);
    assert(servo_motion_get_telemetry()->remaining_ms == 0U);
}

int main(void)
{
    test_synchronized_interpolation();
    test_validation_and_stop();
    puts("servo_motion_controller_test: PASS");
    return 0;
}
