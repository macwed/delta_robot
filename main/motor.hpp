#pragma once
#include <cstdint>

#include "delta_ik.hpp"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define MOTOR1_EN   GPIO_NUM_6
#define MOTOR1_STEP GPIO_NUM_15
#define MOTOR1_DIR  GPIO_NUM_7
#define MOTOR2_EN   GPIO_NUM_5
#define MOTOR2_STEP GPIO_NUM_21
#define MOTOR2_DIR  GPIO_NUM_47
#define MOTOR3_EN   GPIO_NUM_12
#define MOTOR3_STEP GPIO_NUM_10
#define MOTOR3_DIR  GPIO_NUM_11

#define STEPS_10DEG    44
#define STEPS_DELAY_US 2000
#define NUM_MOTORS     3
#define BIT_ALL_READY  ((1 << NUM_MOTORS) - 1)

constexpr float HOME_ANGLE_DEG = -44.5f;
constexpr float HOME_ANGLE_RAD = HOME_ANGLE_DEG * DEG_TO_RAD;
constexpr int32_t microsteps_per_step = 8;
constexpr int32_t steps_per_revolution = 200;

struct MotorTarget {
    float angle_rad;
};

extern QueueHandle_t motor_queue[3];

extern EventGroupHandle_t sync_event;
extern uint8_t motor_id[3];

void step_motor(gpio_num_t step_pin, int steps, uint32_t delay_us);
void motor_task(void *p);