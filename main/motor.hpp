#pragma once

#include <cstdint>

#include "delta_ik.hpp"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

constexpr uint64_t gpio_mask(gpio_num_t pin)
{
    return pin == GPIO_NUM_NC ? 0ULL : (1ULL << static_cast<int>(pin));
}

constexpr gpio_num_t MOTOR1_DIR = GPIO_NUM_4;
constexpr gpio_num_t MOTOR1_STEP = GPIO_NUM_5;
constexpr gpio_num_t MOTOR2_DIR = GPIO_NUM_6;
constexpr gpio_num_t MOTOR2_STEP = GPIO_NUM_7;
constexpr gpio_num_t MOTOR3_DIR = GPIO_NUM_15;
constexpr gpio_num_t MOTOR3_STEP = GPIO_NUM_16;

// DRV8825 control pins shared by all three axes.
constexpr gpio_num_t DRV8825_EN = GPIO_NUM_9;
constexpr gpio_num_t DRV8825_MODE0 = GPIO_NUM_12;
constexpr gpio_num_t DRV8825_MODE1 = GPIO_NUM_11;
constexpr gpio_num_t DRV8825_MODE2 = GPIO_NUM_10;

constexpr int NUM_MOTORS = 3;

constexpr float HOME_ANGLE_DEG = -44.f;
constexpr float HOME_ANGLE_RAD = HOME_ANGLE_DEG * DEG_TO_RAD;
constexpr float MAX_ANGLE_DEG = 70.0f;
constexpr float MAX_ANGLE_RAD = MAX_ANGLE_DEG * DEG_TO_RAD;
constexpr uint32_t STEP_PULSE_HIGH_US = 10;
constexpr uint32_t MOTION_TICK_US = 50;
constexpr uint32_t STEP_PULSE_HIGH_TICKS =
        (STEP_PULSE_HIGH_US + MOTION_TICK_US - 1) / MOTION_TICK_US;
constexpr uint32_t DIR_SETUP_US = 50;
constexpr TickType_t COORDINATOR_PERIOD_TICKS = pdMS_TO_TICKS(10);

void motor_configure_microstep_pins();
bool motor_set_microsteps(int microsteps_per_step);
int motor_get_microsteps();
void motor_init_motion_engine();
void motor_set_target_angles(const float target_angles[NUM_MOTORS]);
void motor_get_current_angles(float current_angles[NUM_MOTORS]);
