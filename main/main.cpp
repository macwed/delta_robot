#include "coordinator.hpp"
#include "motor.hpp"
#include "delta_ik.hpp"

extern "C" void app_main(void)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask =  (1ULL << MOTOR1_EN) | (1ULL << MOTOR1_STEP) | (1ULL << MOTOR1_DIR) |
                            (1ULL << MOTOR2_EN) | (1ULL << MOTOR2_STEP) | (1ULL << MOTOR2_DIR) |
                            (1ULL << MOTOR3_EN) | (1ULL << MOTOR3_STEP) | (1ULL << MOTOR3_DIR) |
                            (1ULL << MOTOR1_MS1) | (1ULL << MOTOR1_MS2) |
                            (1ULL << MOTOR2_MS1) | (1ULL << MOTOR2_MS2) |
                            (1ULL << MOTOR3_MS1) | (1ULL << MOTOR3_MS2);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    motor_configure_microstep_pins();

    gpio_set_level(MOTOR1_STEP, 0);
    gpio_set_level(MOTOR1_DIR, 0);
    gpio_set_level(MOTOR2_STEP, 0);
    gpio_set_level(MOTOR2_DIR, 0);
    gpio_set_level(MOTOR3_STEP, 0);
    gpio_set_level(MOTOR3_DIR, 0);

    // Let the mechanism settle into the passive power-off position before enabling torque.
    gpio_set_level(MOTOR1_EN, 1);
    gpio_set_level(MOTOR2_EN, 1);
    gpio_set_level(MOTOR3_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(800));

    gpio_set_level(MOTOR1_EN, 0);
    gpio_set_level(MOTOR2_EN, 0);
    gpio_set_level(MOTOR3_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    motor_home_reseat_all();
    vTaskDelay(pdMS_TO_TICKS(100));
    motor_init_motion_engine();
    xTaskCreate(coordinator_task, "coordinator", 4096, NULL, 2, NULL);
    xTaskCreate(coordinator_console_task, "console", 4096, NULL, 1, NULL);
}
