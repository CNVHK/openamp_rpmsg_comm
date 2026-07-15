#include "lqr_controller.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static LqrConfig test_config(void)
{
    LqrConfig config = {
        .k_theta = -100.0f,
        .k_theta_rate = -10.0f,
        .k_position = -1.0f,
        .k_velocity = -2.0f,
        .wheel_radius_m = 0.05f,
        .torque_limit_nm = 0.10f,
        .fall_angle_rad = 0.30f,
        .pitch_offset_rad = 0.0f,
        .left_motor_direction = 1.0f,
        .right_motor_direction = -1.0f,
    };
    return config;
}

static LqrSensorData upright_sensor(void)
{
    LqrSensorData sensor = {
        .left_position_rad = 10.0f,
        .right_position_rad = -10.0f,
        .valid = 1U,
    };
    return sensor;
}

int main(void)
{
    LqrController controller;
    LqrConfig config = test_config();
    LqrSensorData sensor = upright_sensor();
    LqrOutput output;

    assert(lqr_init(&controller, &config) == 0);
    assert(lqr_enable(&controller, &sensor) == 0);
    output = lqr_update(&controller, &sensor);
    assert(output.enabled && !output.fault);
    assert(fabsf(output.wheel_position_m) < 1.0e-6f);
    assert(fabsf(output.left_torque_nm) < 1.0e-6f);

    sensor.pitch_rad = 0.10f;
    output = lqr_update(&controller, &sensor);
    assert(output.left_torque_nm > 0.0f);
    assert(output.right_torque_nm < 0.0f);
    assert(fabsf(output.left_torque_nm - config.torque_limit_nm) < 1.0e-6f);

    sensor.pitch_rad = 0.31f;
    output = lqr_update(&controller, &sensor);
    assert(output.fault && !output.enabled);

    sensor = upright_sensor();
    assert(lqr_enable(&controller, &sensor) == 0);
    sensor.left_position_rad += 1.0f;
    sensor.right_position_rad -= 1.0f;
    output = lqr_update(&controller, &sensor);
    assert(output.wheel_position_m > 0.0f);

    sensor.pitch_rad = NAN;
    output = lqr_update(&controller, &sensor);
    assert(output.fault);

    puts("lqr_controller_test: PASS");
    return 0;
}
