//
// Created by maciej on 15.03.2026.
//

#pragma once
#include <cmath>

struct DeltaConfig {
    float r_base;
    float r_effector;
    float L1;
    float L2;
};

constexpr float DEG_TO_RAD = M_PI / 180.0f;
constexpr int32_t MICROSTEPS_PER_STEP  = 8;
constexpr int32_t STEPS_PER_REVOLUTION = 200;
constexpr int32_t STEPS_PER_RAD = STEPS_PER_REVOLUTION * MICROSTEPS_PER_STEP / (2.f * M_PI);

float calc_arm_angle(const DeltaConfig& cfg,
                     float x, float y, float z,
                     int arm_index);

bool delta_ik(const DeltaConfig& cfg,
              float x, float y, float z,
              float angles[3]);

int32_t angle_to_steps(float angle_rad);