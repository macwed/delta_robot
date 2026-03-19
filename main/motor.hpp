#pragma once
#include <cstdint>

#include "delta_ik.hpp"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#define MOTOR1_EN   GPIO_NUM_6
#define MOTOR1_STEP GPIO_NUM_15
#define MOTOR1_DIR  GPIO_NUM_7
#define MOTOR2_EN   GPIO_NUM_5
#define MOTOR2_STEP GPIO_NUM_21
#define MOTOR2_DIR  GPIO_NUM_47
#define MOTOR3_EN   GPIO_NUM_12
#define MOTOR3_STEP GPIO_NUM_10
#define MOTOR3_DIR  GPIO_NUM_11

#define MOTOR1_MS1  GPIO_NUM_9
#define MOTOR1_MS2  GPIO_NUM_3

#define MOTOR2_MS1  GPIO_NUM_2
#define MOTOR2_MS2  GPIO_NUM_1

#define MOTOR3_MS1  GPIO_NUM_41
#define MOTOR3_MS2  GPIO_NUM_42


#define NUM_MOTORS 3

constexpr float HOME_ANGLE_DEG = -44.5f;
constexpr float HOME_ANGLE_RAD = HOME_ANGLE_DEG * DEG_TO_RAD;
constexpr float MAX_ANGLE_DEG = 70.0f;
constexpr float MAX_ANGLE_RAD = MAX_ANGLE_DEG * DEG_TO_RAD;
constexpr int kMicrostepCfgMs1 =
        (MICROSTEPS_PER_STEP == 8)  ? 0 :
        (MICROSTEPS_PER_STEP == 16) ? 1 :
        (MICROSTEPS_PER_STEP == 32) ? 1 :
        (MICROSTEPS_PER_STEP == 64) ? 0 : -1;
constexpr int kMicrostepCfgMs2 =
        (MICROSTEPS_PER_STEP == 8)  ? 0 :
        (MICROSTEPS_PER_STEP == 16) ? 1 :
        (MICROSTEPS_PER_STEP == 32) ? 0 :
        (MICROSTEPS_PER_STEP == 64) ? 1 : -1;
static_assert(kMicrostepCfgMs1 >= 0 && kMicrostepCfgMs2 >= 0,
              "Unsupported standalone microstep configuration");
constexpr uint32_t STEP_PULSE_HIGH_US = 10;
constexpr uint32_t MOTION_TICK_US = 50;
constexpr uint32_t STEP_PULSE_HIGH_TICKS =
        (STEP_PULSE_HIGH_US + MOTION_TICK_US - 1) / MOTION_TICK_US;
constexpr uint32_t DIR_SETUP_US = 50;
constexpr int32_t STEP_RAMP_COUNT = 48;
constexpr int32_t HOME_RESEAT_STEPS = 32;
constexpr uint32_t HOME_RESEAT_DELAY_US = 15000;
constexpr int32_t FINE_MOVE_MAX_STEPS = 24;
constexpr uint32_t FINE_MOVE_DELAY_US = 2200;
constexpr uint32_t FINE_MOVE_START_DELAY_US = 3500;
constexpr int32_t MEDIUM_MOVE_MAX_STEPS = 96;
constexpr uint32_t MEDIUM_MOVE_DELAY_US = 3500;
constexpr uint32_t MEDIUM_MOVE_START_DELAY_US = 7000;
constexpr uint32_t LARGE_MOVE_DELAY_US = 6000;
constexpr uint32_t LARGE_MOVE_START_DELAY_US = 12000;
constexpr TickType_t COORDINATOR_PERIOD_TICKS = pdMS_TO_TICKS(10);

void motor_configure_microstep_pins();
void motor_home_reseat_all();
void motor_init_motion_engine();
void motor_set_target_angles(const float target_angles[NUM_MOTORS]);
void motor_get_current_angles(float current_angles[NUM_MOTORS]);
