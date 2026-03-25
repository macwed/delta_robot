#include "coordinator.hpp"
#include "motor.hpp"
#include "delta_ik.hpp"

extern "C" void app_main(void)
{
    constexpr TickType_t kStartupSettlingDelay = pdMS_TO_TICKS(2500);

    // Preload the output latch so EN goes high as soon as the pin becomes an output.
    gpio_set_level(DRV8825_EN, 1);

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = gpio_mask(DRV8825_EN) |
                           gpio_mask(MOTOR1_STEP) | gpio_mask(MOTOR1_DIR) |
                           gpio_mask(MOTOR2_STEP) | gpio_mask(MOTOR2_DIR) |
                           gpio_mask(MOTOR3_STEP) | gpio_mask(MOTOR3_DIR) |
                           gpio_mask(DRV8825_MODE0) |
                           gpio_mask(DRV8825_MODE1) |
                           gpio_mask(DRV8825_MODE2);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    gpio_input_enable(DRV8825_EN);
    gpio_input_enable(DRV8825_MODE0);
    gpio_input_enable(DRV8825_MODE1);
    gpio_input_enable(DRV8825_MODE2);

    // Keep all drivers disabled long enough for the arms to fall back to HOME_ANGLE.
    gpio_set_level(DRV8825_EN, 1);
    motor_configure_microstep_pins();

    gpio_set_level(MOTOR1_STEP, 0);
    gpio_set_level(MOTOR1_DIR, 0);
    gpio_set_level(MOTOR2_STEP, 0);
    gpio_set_level(MOTOR2_DIR, 0);
    gpio_set_level(MOTOR3_STEP, 0);
    gpio_set_level(MOTOR3_DIR, 0);

    vTaskDelay(kStartupSettlingDelay);
    gpio_set_level(DRV8825_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    motor_init_motion_engine();
    xTaskCreate(coordinator_task, "coordinator", 4096, NULL, 2, NULL);
    xTaskCreate(coordinator_console_task, "console", 4096, NULL, 1, NULL);
}
