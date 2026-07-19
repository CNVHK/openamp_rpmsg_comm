#include "leg_kinematics.h"
#include "leg_robot_geometry.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_inverse_forward_round_trip(void)
{
    const LegKinematicsConfig config = {
        .base_spacing_m = 0.10f,
        .upper_link_m = 0.10f,
        .lower_link_m = 0.15f,
        .front_elbow_branch = 1,
        .rear_elbow_branch = -1,
        .foot_branch = -1,
    };
    LegKinematicsAngles angles;
    float x;
    float height;

    assert(leg_inverse_kinematics(&config, 0.0f, 0.18f, &angles) == 0);
    assert(leg_forward_kinematics(&config, &angles, &x, &height) == 0);
    assert(fabsf(x) < 1.0e-5f);
    assert(fabsf(height - 0.18f) < 1.0e-5f);

    assert(leg_inverse_kinematics(&config, 0.02f, 0.17f, &angles) == 0);
    assert(leg_forward_kinematics(&config, &angles, &x, &height) == 0);
    assert(fabsf(x - 0.02f) < 1.0e-5f);
    assert(fabsf(height - 0.17f) < 1.0e-5f);
}

static void test_unreachable_and_invalid_geometry(void)
{
    LegKinematicsConfig config = {
        .base_spacing_m = 0.10f,
        .upper_link_m = 0.10f,
        .lower_link_m = 0.15f,
        .front_elbow_branch = 1,
        .rear_elbow_branch = -1,
        .foot_branch = -1,
    };
    LegKinematicsAngles angles;

    assert(leg_inverse_kinematics(&config, 0.0f, 0.50f, &angles) != 0);
    config.front_elbow_branch = 0;
    assert(!leg_kinematics_config_valid(&config));
}

static void test_measured_reference_calibration(void)
{
    const float rad_to_deg = 57.29577951308232f;
    LegKinematicsCalibration calibration;
    LegKinematicsAngles effective;

    leg_robot_get_measured_calibration(&calibration);
    assert(leg_kinematics_calibration_valid(&calibration));
    assert(leg_calibrated_inverse_kinematics(
        &calibration, 0.0f, 0.140f, &effective) == 0);
    assert(fabsf(effective.front_angle_rad * rad_to_deg - 45.0f) < 1.0e-4f);
    assert(fabsf(effective.rear_angle_rad * rad_to_deg - 45.0f) < 1.0e-4f);

    assert(leg_calibrated_inverse_kinematics(
        &calibration, 0.0f, 0.150f, &effective) == 0);
    assert(fabsf(effective.front_angle_rad * rad_to_deg - 49.40f) < 0.02f);
    assert(fabsf(effective.rear_angle_rad * rad_to_deg - 49.40f) < 0.02f);

    assert(leg_calibrated_inverse_kinematics(
        &calibration, 0.005f, 0.140f, &effective) == 0);
    assert(effective.front_angle_rad < 45.0f / rad_to_deg);
    assert(effective.rear_angle_rad > 45.0f / rad_to_deg);
}

int main(void)
{
    test_inverse_forward_round_trip();
    test_unreachable_and_invalid_geometry();
    test_measured_reference_calibration();
    puts("leg_kinematics_test: PASS");
    return 0;
}
