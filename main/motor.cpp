//
// Created by maciej on 13.03.2026.
//

#include "motor.hpp"


EventGroupHandle_t sync_event;
uint8_t motor_id[3] = {1, 2, 3};
QueueHandle_t motor_queue[3];

void step_motor(gpio_num_t step_pin, int steps, uint32_t delay_us)
{
    for (int i = 0; i < steps; i++)
    {
        gpio_set_level(step_pin, 1);
        esp_rom_delay_us(10);
        gpio_set_level(step_pin, 0);
        esp_rom_delay_us(delay_us);
    }
}

void motor_task(void *p)
{
    uint8_t motor_id = *(uint8_t*)p;
    static uint8_t motor_dir_invert[3] = {1, 1, 1};
    gpio_num_t motor_step, motor_dir;

    switch (motor_id)
    {
        case 1:
            motor_step = MOTOR1_STEP;
            motor_dir = MOTOR1_DIR;
            break;
        case 2:
            motor_step = MOTOR2_STEP;
            motor_dir = MOTOR2_DIR;
            break;
        case 3:
            motor_step = MOTOR3_STEP;
            motor_dir = MOTOR3_DIR;
            break;
        default:
            printf("ERROR: unknown motor_id %d\n", motor_id);
            vTaskDelete(NULL);
            return;
    }

    uint8_t invert = motor_dir_invert[motor_id - 1];

    float current_angle = HOME_ANGLE_RAD;

    MotorTarget target;

    while (true)
    {
        xEventGroupSetBits(sync_event, 1 << (motor_id - 1));
        xEventGroupWaitBits(sync_event, BIT_ALL_READY,
                    pdFALSE, pdTRUE, portMAX_DELAY);
        xEventGroupClearBits(sync_event, 1 << (motor_id - 1));

        xQueueReceive(motor_queue[motor_id - 1], &target, portMAX_DELAY);

        int32_t steps = angle_to_steps(target.angle_rad - current_angle);
        if (steps < 0)
        {
            steps = -steps;
            gpio_set_level(motor_dir, 1 ^ invert);
        } else
        {
            gpio_set_level(motor_dir, 0 ^ invert);
        }

        vTaskDelay(pdMS_TO_TICKS(50));

        step_motor(motor_step, steps, STEPS_DELAY_US);
        current_angle = target.angle_rad;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}