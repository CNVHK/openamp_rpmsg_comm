#include "lqr_controller.h"

#include <math.h>
#include <string.h>

static float clampf(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static int lqr_sensor_is_finite(const LqrSensorData *sensor)
{
    return isfinite(sensor->pitch_rad) &&
           isfinite(sensor->pitch_rate_rad_s) &&
           isfinite(sensor->left_position_rad) &&
           isfinite(sensor->right_position_rad) &&
           isfinite(sensor->left_velocity_rad_s) &&
           isfinite(sensor->right_velocity_rad_s);
}

static float wheel_position(const LqrConfig *cfg,
                            const LqrSensorData *sensor)
{
    return 0.5f * cfg->wheel_radius_m *
           (cfg->left_motor_direction * sensor->left_position_rad +
            cfg->right_motor_direction * sensor->right_position_rad);
}

int lqr_init(LqrController *controller, const LqrConfig *config)
{
    if (controller == NULL || config == NULL ||
        !(config->wheel_radius_m > 0.0f) ||
        !(config->torque_limit_nm > 0.0f) ||
        !(config->fall_angle_rad > 0.0f)) {
        return -1;
    }

    memset(controller, 0, sizeof(*controller));
    controller->config = *config;
    controller->initialized = 1U;
    return 0;
}

int lqr_enable(LqrController *controller, const LqrSensorData *sensor)
{
    if (controller == NULL || sensor == NULL || !controller->initialized ||
        !sensor->valid || !lqr_sensor_is_finite(sensor)) {
        return -1;
    }

    controller->wheel_zero_position_m =
        wheel_position(&controller->config, sensor);
    controller->pitch_target_rad = 0.0f;
    controller->position_target_m = 0.0f;
    controller->velocity_target_m_s = 0.0f;
    controller->enabled = 1U;
    return 0;
}

void lqr_disable(LqrController *controller)
{
    if (controller != NULL) {
        controller->enabled = 0U;
    }
}

LqrOutput lqr_update(LqrController *controller, const LqrSensorData *sensor)
{
    LqrOutput output;
    const LqrConfig *cfg;
    float pitch;
    float wheel_pos;
    float wheel_vel;
    float force;
    float torque;

    memset(&output, 0, sizeof(output));
    if (controller == NULL || sensor == NULL || !controller->initialized ||
        !sensor->valid || !lqr_sensor_is_finite(sensor)) {
        output.fault = 1U;
        return output;
    }
    if (!controller->enabled) {
        return output;
    }

    cfg = &controller->config;
    pitch = sensor->pitch_rad - cfg->pitch_offset_rad;
    if (fabsf(pitch) > cfg->fall_angle_rad) {
        controller->enabled = 0U;
        output.fault = 1U;
        return output;
    }

    wheel_pos = wheel_position(cfg, sensor) - controller->wheel_zero_position_m;
    wheel_vel = 0.5f * cfg->wheel_radius_m *
                (cfg->left_motor_direction * sensor->left_velocity_rad_s +
                 cfg->right_motor_direction * sensor->right_velocity_rad_s);
    force = -(cfg->k_theta * (pitch - controller->pitch_target_rad) +
              cfg->k_theta_rate * sensor->pitch_rate_rad_s +
              cfg->k_position * (wheel_pos - controller->position_target_m) +
              cfg->k_velocity * (wheel_vel - controller->velocity_target_m_s));
    torque = clampf(0.5f * force * cfg->wheel_radius_m,
                    cfg->torque_limit_nm);

    output.left_torque_nm = cfg->left_motor_direction * torque;
    output.right_torque_nm = cfg->right_motor_direction * torque;
    output.wheel_position_m = wheel_pos;
    output.wheel_velocity_m_s = wheel_vel;
    output.force_command_n = force;
    output.enabled = 1U;
    return output;
}
