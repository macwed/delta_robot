#include "motor.hpp"
#include "delta_ik.hpp"

extern "C" void app_main(void)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask =  (1ULL << MOTOR1_EN) | (1ULL << MOTOR1_STEP) | (1ULL << MOTOR1_DIR) |
                            (1ULL << MOTOR2_EN) | (1ULL << MOTOR2_STEP) | (1ULL << MOTOR2_DIR) |
                            (1ULL << MOTOR3_EN) | (1ULL << MOTOR3_STEP) | (1ULL << MOTOR3_DIR);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(MOTOR1_EN, 0);
    gpio_set_level(MOTOR2_EN, 0);
    gpio_set_level(MOTOR3_EN, 0);

    vTaskDelay(pdMS_TO_TICKS(100));

    sync_event = xEventGroupCreate();

    if (sync_event == NULL)
    {
        printf("create event group failed\n");
        return;
    } else
    {
        xTaskCreate(motor_task, "motor1", 2048, &motor_id[0], 1, NULL);
        xTaskCreate(motor_task, "motor2", 2048, &motor_id[1], 1, NULL);
        xTaskCreate(motor_task, "motor3", 2048, &motor_id[2], 1, NULL);
    }
}