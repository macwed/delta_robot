//
// Created by maciej on 16.03.2026.
//

#include "coordinator.hpp"

#include "delta_ik.hpp"
#include "motor.hpp"
#include "freertos/queue.h"

const DeltaConfig DELTA = {
    .r_base     = 49.63f,
    .r_effector = 120.51f,
    .L1         = 110.95f,
    .L2         = 164.76f
};

void coordinator_task(void *p)
{
    float angles[3];

    while (true)
    {
        bool ok = delta_ik(DELTA, 0, 0, 150.f, angles);
        if (ok)
        {
            for (int i = 0; i < NUM_MOTORS; i++)
            {
                MotorTarget target = {angles[i]};
                xQueueOverwrite(motor_queue[i], &target);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
