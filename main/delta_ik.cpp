//
// Created by maciej on 15.03.2026.
//

#include "delta_ik.hpp"

#include <cmath>
#include <cstdio>

#include "esp_log.h"

static const char* TAG = "delta_ik";

float calc_arm_angle(const DeltaConfig& cfg,
                     float x, float y, float z,
                     int arm_index)
{
    float angle_i = (float)arm_index * 120.f * DEG_TO_RAD;
    float x_proj = cos(angle_i) * x + sin(angle_i) * y; //rzut punktu na płaszczyznę ramienia

    float E_x = x_proj - (cfg.r_base - cfg.r_effector);
    float E_y = z;

    float d2 = E_x * E_x + E_y * E_y;
    float d = sqrt(d2); //długość wektora między B (oś obrotu wału silnika) a Q - efektorem

    float cos_alpha = (cfg.L1 * cfg.L1 + d2 - cfg.L2 * cfg.L2) / (2 * cfg.L1 * d);
    if (cos_alpha < -1.0f || cos_alpha > 1.0f) return NAN;

    float alpha = acos(cos_alpha);
    float cos_beta = (cfg.r_effector - cfg.r_base) / d;
    if (cos_beta < -1.0f || cos_beta > 1.0f) return NAN;

    float beta = acos(cos_beta);

    return (beta - alpha);
}

bool delta_ik(const DeltaConfig& cfg, float x, float y, float z, float angles[3])
{
    for (int i = 0; i < 3; i++)
    {
        angles[i] = calc_arm_angle(cfg, x, y, z, i);
        if (std::isnan(angles[i]))
        {
            ESP_LOGE(TAG, "Angle out of range for arm %d", i);
            return false;
        }
    }
    return true;
}

int32_t angle_to_steps(float angle_rad)
{
    return (int32_t)(angle_rad * STEPS_PER_RAD);
}
