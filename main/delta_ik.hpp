#pragma once
#include <cmath>

struct DeltaConfig {
    float r_base;
    float r_effector;
    float L1;
    float L2;
};

constexpr float DEG_TO_RAD = M_PI / 180.0f;
constexpr int32_t DEFAULT_MICROSTEPS_PER_STEP = 4;
constexpr int32_t STEPS_PER_REVOLUTION = 200;

constexpr float steps_per_rad_for_microsteps(int32_t microsteps_per_step)
{
    return static_cast<float>(STEPS_PER_REVOLUTION) * static_cast<float>(microsteps_per_step) / (2.f * M_PI);
}

extern const DeltaConfig kDeltaConfig;

float calc_arm_angle(const DeltaConfig& cfg,
                     float x, float y, float z,
                     int arm_index);

bool delta_ik(const DeltaConfig& cfg,
              float x, float y, float z,
              float angles[3]);
