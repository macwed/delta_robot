//
// Created by maciej on 13.03.2026.
//

#include "motor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"

namespace {
constexpr char kTag[] = "motor";
constexpr uint32_t kMotionNotifyTargetUpdate = 1U << 0;
constexpr uint32_t kMotionNotifySegmentDone  = 1U << 1;

constexpr uint8_t kMotorDirInvert[NUM_MOTORS] = {1, 1, 1};
constexpr gpio_num_t kMotorStepPins[NUM_MOTORS] = {
        MOTOR1_STEP, MOTOR2_STEP, MOTOR3_STEP
};
constexpr gpio_num_t kMotorDirPins[NUM_MOTORS] = {
        MOTOR1_DIR, MOTOR2_DIR, MOTOR3_DIR
};
constexpr gpio_num_t kMotorMs1Pins[NUM_MOTORS] = {
        MOTOR1_MS1, MOTOR2_MS1, MOTOR3_MS1
};
constexpr gpio_num_t kMotorMs2Pins[NUM_MOTORS] = {
        MOTOR1_MS2, MOTOR2_MS2, MOTOR3_MS2
};

struct MotionPlan {
    int32_t abs_steps[NUM_MOTORS];
    int32_t accumulators[NUM_MOTORS];
    int8_t dir_sign[NUM_MOTORS];
    int32_t max_steps;
    int32_t step_index;
    uint32_t start_delay_ticks;
    uint32_t cruise_delay_ticks;
    uint32_t wait_ticks;
    uint8_t active_step_mask;
    bool active;
    bool pulse_high;
    bool replan_requested;
};

struct MotionState {
    int32_t current_steps[NUM_MOTORS];
    int32_t target_steps[NUM_MOTORS];
    MotionPlan plan;
    bool timer_running;
};

static_assert(STEP_PULSE_HIGH_TICKS >= 1, "STEP_PULSE_HIGH_TICKS must be at least 1");

MotionState s_motion_state = {};
portMUX_TYPE s_motion_mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t s_motion_task = nullptr;
gptimer_handle_t s_motion_timer = nullptr;
gpio_dev_t *const s_gpio_hw = GPIO_LL_GET_HW(0);

uint32_t us_to_ticks_ceil(uint32_t delay_us)
{
    return (delay_us + MOTION_TICK_US - 1) / MOTION_TICK_US;
}

uint32_t wait_counter_from_ticks(uint32_t delay_ticks)
{
    return delay_ticks > 0 ? delay_ticks - 1 : 0;
}

uint32_t smoothstep_q16(uint32_t numerator, uint32_t denominator)
{
    if (denominator == 0) {
        return 65536U;
    }

    const uint32_t u = numerator >= denominator
                       ? 65536U
                       : static_cast<uint32_t>((static_cast<uint64_t>(numerator) << 16) / denominator);
    const uint64_t u2 = (static_cast<uint64_t>(u) * u) >> 16;
    const uint64_t u3 = (u2 * u) >> 16;
    const uint64_t eased = 3 * u2 - 2 * u3;
    return static_cast<uint32_t>(std::min<uint64_t>(eased, 65536U));
}

uint32_t interpolate_delay(uint32_t start_delay_ticks, uint32_t cruise_delay_ticks, uint32_t ease_q16)
{
    if (start_delay_ticks <= cruise_delay_ticks) {
        return cruise_delay_ticks;
    }

    const uint32_t delta = start_delay_ticks - cruise_delay_ticks;
    return start_delay_ticks - static_cast<uint32_t>((static_cast<uint64_t>(delta) * ease_q16) >> 16);
}

float steps_to_angle_rad(int32_t steps_from_home)
{
    return HOME_ANGLE_RAD + static_cast<float>(steps_from_home) / static_cast<float>(STEPS_PER_RAD);
}

uint32_t step_delay_for_index(int32_t completed_steps, int32_t total_steps,
                              uint32_t start_delay_ticks, uint32_t cruise_delay_ticks)
{
    if (total_steps <= 1) {
        return start_delay_ticks;
    }

    const int32_t ramp_steps = std::max<int32_t>(1, std::min<int32_t>(STEP_RAMP_COUNT, total_steps / 2));
    const int32_t phase_steps = std::min(completed_steps, total_steps - completed_steps);
    if (phase_steps >= ramp_steps) {
        return cruise_delay_ticks;
    }

    return interpolate_delay(start_delay_ticks, cruise_delay_ticks,
                             smoothstep_q16(static_cast<uint32_t>(phase_steps),
                                            static_cast<uint32_t>(ramp_steps)));
}

void pulse_step_pins_blocking(const bool step_mask[NUM_MOTORS], uint32_t delay_us)
{
    bool any_step = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (!step_mask[i]) {
            continue;
        }
        gpio_set_level(kMotorStepPins[i], 1);
        any_step = true;
    }

    if (!any_step) {
        esp_rom_delay_us(delay_us);
        return;
    }

    esp_rom_delay_us(STEP_PULSE_HIGH_US);
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (step_mask[i]) {
            gpio_set_level(kMotorStepPins[i], 0);
        }
    }
    esp_rom_delay_us(delay_us);
}

void move_motor_group_blocking(const int32_t signed_steps[NUM_MOTORS], uint32_t delay_us)
{
    int32_t abs_steps[NUM_MOTORS] = {0};
    int32_t max_steps = 0;

    for (int i = 0; i < NUM_MOTORS; i++) {
        abs_steps[i] = std::abs(signed_steps[i]);
        max_steps = std::max(max_steps, abs_steps[i]);

        const int dir_level = (signed_steps[i] < 0 ? 1 : 0) ^ kMotorDirInvert[i];
        gpio_set_level(kMotorDirPins[i], dir_level);
    }

    if (max_steps == 0) {
        return;
    }

    esp_rom_delay_us(DIR_SETUP_US);

    int32_t accumulators[NUM_MOTORS] = {0};
    for (int32_t step_index = 0; step_index < max_steps; step_index++) {
        bool step_mask[NUM_MOTORS] = {false, false, false};
        for (int i = 0; i < NUM_MOTORS; i++) {
            accumulators[i] += abs_steps[i];
            if (accumulators[i] >= max_steps) {
                accumulators[i] -= max_steps;
                step_mask[i] = true;
            }
        }

        pulse_step_pins_blocking(step_mask, delay_us);
    }
}

void set_step_mask_level_ll(uint8_t step_mask, int level)
{
    for (int i = 0; i < NUM_MOTORS; i++) {
        if ((step_mask & (1U << i)) == 0) {
            continue;
        }
        gpio_ll_set_level(s_gpio_hw, kMotorStepPins[i], level);
    }
}

void set_dir_pins(const int8_t dir_sign[NUM_MOTORS])
{
    for (int i = 0; i < NUM_MOTORS; i++) {
        const int dir_level = (dir_sign[i] < 0 ? 1 : 0) ^ kMotorDirInvert[i];
        gpio_set_level(kMotorDirPins[i], dir_level);
    }
}

bool build_motion_plan(const int32_t current_steps[NUM_MOTORS],
                       const int32_t target_steps[NUM_MOTORS],
                       MotionPlan *plan,
                       int32_t signed_steps[NUM_MOTORS],
                       float current_angles[NUM_MOTORS],
                       float target_angles[NUM_MOTORS])
{
    int32_t max_steps = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        signed_steps[i] = target_steps[i] - current_steps[i];
        max_steps = std::max(max_steps, std::abs(signed_steps[i]));
        current_angles[i] = steps_to_angle_rad(current_steps[i]);
        target_angles[i] = steps_to_angle_rad(target_steps[i]);
    }

    if (max_steps == 0) {
        return false;
    }

    uint32_t start_delay_us = LARGE_MOVE_START_DELAY_US;
    uint32_t cruise_delay_us = LARGE_MOVE_DELAY_US;
    if (max_steps <= FINE_MOVE_MAX_STEPS) {
        start_delay_us = FINE_MOVE_START_DELAY_US;
        cruise_delay_us = FINE_MOVE_DELAY_US;
    } else if (max_steps <= MEDIUM_MOVE_MAX_STEPS) {
        start_delay_us = MEDIUM_MOVE_START_DELAY_US;
        cruise_delay_us = MEDIUM_MOVE_DELAY_US;
    }

    std::memset(plan, 0, sizeof(*plan));
    for (int i = 0; i < NUM_MOTORS; i++) {
        plan->abs_steps[i] = std::abs(signed_steps[i]);
        plan->dir_sign[i] = (signed_steps[i] > 0) ? 1 : (signed_steps[i] < 0 ? -1 : 0);
    }
    plan->max_steps = max_steps;
    plan->start_delay_ticks = us_to_ticks_ceil(start_delay_us);
    plan->cruise_delay_ticks = us_to_ticks_ceil(cruise_delay_us);
    plan->wait_ticks = wait_counter_from_ticks(us_to_ticks_ceil(DIR_SETUP_US) + plan->start_delay_ticks);
    plan->active = true;
    return true;
}

void stop_motion_timer_if_running()
{
    bool should_stop = false;
    portENTER_CRITICAL(&s_motion_mux);
    if (s_motion_state.timer_running && !s_motion_state.plan.active) {
        s_motion_state.timer_running = false;
        should_stop = true;
    }
    portEXIT_CRITICAL(&s_motion_mux);

    if (should_stop) {
        ESP_ERROR_CHECK(gptimer_stop(s_motion_timer));
    }
}

bool IRAM_ATTR motion_timer_on_alarm(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;

    BaseType_t high_task_woken = pdFALSE;

    portENTER_CRITICAL_ISR(&s_motion_mux);
    MotionPlan &plan = s_motion_state.plan;
    if (!plan.active) {
        portEXIT_CRITICAL_ISR(&s_motion_mux);
        return false;
    }

    if (plan.wait_ticks > 0) {
        plan.wait_ticks--;
        portEXIT_CRITICAL_ISR(&s_motion_mux);
        return false;
    }

    if (plan.pulse_high) {
        set_step_mask_level_ll(plan.active_step_mask, 0);
        plan.active_step_mask = 0;
        plan.pulse_high = false;
        plan.step_index++;

        if (plan.step_index >= plan.max_steps || plan.replan_requested) {
            plan.active = false;
            xTaskNotifyFromISR(s_motion_task, kMotionNotifySegmentDone, eSetBits, &high_task_woken);
        } else {
            const uint32_t delay_ticks = step_delay_for_index(
                    plan.step_index, plan.max_steps, plan.start_delay_ticks, plan.cruise_delay_ticks);
            plan.wait_ticks = wait_counter_from_ticks(delay_ticks);
        }

        portEXIT_CRITICAL_ISR(&s_motion_mux);
        return high_task_woken == pdTRUE;
    }

    uint8_t step_mask = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        plan.accumulators[i] += plan.abs_steps[i];
        if (plan.accumulators[i] >= plan.max_steps) {
            plan.accumulators[i] -= plan.max_steps;
            step_mask |= (1U << i);
            s_motion_state.current_steps[i] += plan.dir_sign[i];
        }
    }

    if (step_mask != 0) {
        set_step_mask_level_ll(step_mask, 1);
        plan.active_step_mask = step_mask;
        plan.pulse_high = true;
        plan.wait_ticks = wait_counter_from_ticks(STEP_PULSE_HIGH_TICKS);
    } else {
        plan.wait_ticks = 0;
    }

    portEXIT_CRITICAL_ISR(&s_motion_mux);
    return false;
}

void motion_task(void *p)
{
    (void)p;

    while (true) {
        uint32_t notify_value = 0;
        xTaskNotifyWait(0, UINT32_MAX, &notify_value, portMAX_DELAY);

        while (true) {
            int32_t current_steps[NUM_MOTORS] = {0};
            int32_t target_steps[NUM_MOTORS] = {0};
            bool plan_active = false;

            portENTER_CRITICAL(&s_motion_mux);
            plan_active = s_motion_state.plan.active;
            for (int i = 0; i < NUM_MOTORS; i++) {
                current_steps[i] = s_motion_state.current_steps[i];
                target_steps[i] = s_motion_state.target_steps[i];
            }
            portEXIT_CRITICAL(&s_motion_mux);

            if (plan_active) {
                break;
            }

            MotionPlan next_plan = {};
            int32_t signed_steps[NUM_MOTORS] = {0};
            float current_angles[NUM_MOTORS] = {0.f};
            float target_angles[NUM_MOTORS] = {0.f};
            if (!build_motion_plan(current_steps, target_steps, &next_plan,
                                   signed_steps, current_angles, target_angles)) {
                stop_motion_timer_if_running();
                break;
            }

            bool needs_rebuild = false;
            portENTER_CRITICAL(&s_motion_mux);
            needs_rebuild = s_motion_state.plan.active;
            for (int i = 0; i < NUM_MOTORS && !needs_rebuild; i++) {
                needs_rebuild = (s_motion_state.target_steps[i] != target_steps[i]);
            }
            portEXIT_CRITICAL(&s_motion_mux);
            if (needs_rebuild) {
                continue;
            }

            set_dir_pins(next_plan.dir_sign);

            ESP_LOGI(kTag,
                     "Plan: M1 %.2f->%.2f (%ld), M2 %.2f->%.2f (%ld), M3 %.2f->%.2f (%ld)",
                     current_angles[0] * 180.f / M_PI, target_angles[0] * 180.f / M_PI, (long)signed_steps[0],
                     current_angles[1] * 180.f / M_PI, target_angles[1] * 180.f / M_PI, (long)signed_steps[1],
                     current_angles[2] * 180.f / M_PI, target_angles[2] * 180.f / M_PI, (long)signed_steps[2]);

            bool should_start_timer = false;
            portENTER_CRITICAL(&s_motion_mux);
            s_motion_state.plan = next_plan;
            if (!s_motion_state.timer_running) {
                s_motion_state.timer_running = true;
                should_start_timer = true;
            }
            portEXIT_CRITICAL(&s_motion_mux);

            if (should_start_timer) {
                ESP_ERROR_CHECK(gptimer_set_raw_count(s_motion_timer, 0));
                ESP_ERROR_CHECK(gptimer_start(s_motion_timer));
            }

            break;
        }
    }
}
} // namespace

void motor_configure_microstep_pins()
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask =
            (1ULL << MOTOR1_MS1) | (1ULL << MOTOR1_MS2) |
            (1ULL << MOTOR2_MS1) | (1ULL << MOTOR2_MS2) |
            (1ULL << MOTOR3_MS1) | (1ULL << MOTOR3_MS2);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_set_level(kMotorMs1Pins[i], kMicrostepCfgMs1);
        gpio_set_level(kMotorMs2Pins[i], kMicrostepCfgMs2);
    }

    printf("Microstep config: %ld microsteps, MS1=%d MS2=%d\n",
           (long)MICROSTEPS_PER_STEP, kMicrostepCfgMs1, kMicrostepCfgMs2);
}

void motor_home_reseat_all()
{
    const int32_t home_steps[NUM_MOTORS] = {
            -HOME_RESEAT_STEPS,
            -HOME_RESEAT_STEPS,
            -HOME_RESEAT_STEPS
    };
    printf("Home reseat: %ld steps\n", (long)HOME_RESEAT_STEPS);
    move_motor_group_blocking(home_steps, HOME_RESEAT_DELAY_US);
}

void motor_init_motion_engine()
{
    portENTER_CRITICAL(&s_motion_mux);
    std::memset(&s_motion_state, 0, sizeof(s_motion_state));
    portEXIT_CRITICAL(&s_motion_mux);

    if (s_motion_task == nullptr) {
        xTaskCreate(motion_task, "motion", 4096, nullptr, 4, &s_motion_task);
    }

    if (s_motion_timer == nullptr) {
        const gptimer_config_t timer_config = {
                .clk_src = GPTIMER_CLK_SRC_DEFAULT,
                .direction = GPTIMER_COUNT_UP,
                .resolution_hz = 1000000,
                .intr_priority = 0,
                .flags = {
                        .intr_shared = 0,
                        .allow_pd = 0,
                        .backup_before_sleep = 0,
                }
        };
        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &s_motion_timer));

        const gptimer_event_callbacks_t callbacks = {
                .on_alarm = motion_timer_on_alarm
        };
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_motion_timer, &callbacks, nullptr));
        ESP_ERROR_CHECK(gptimer_enable(s_motion_timer));

        static gptimer_alarm_config_t alarm_config = {
                .alarm_count = MOTION_TICK_US,
                .reload_count = 0,
                .flags = {
                        .auto_reload_on_alarm = true
                }
        };
        ESP_ERROR_CHECK(gptimer_set_alarm_action(s_motion_timer, &alarm_config));
    }
}

void motor_set_target_angles(const float target_angles[NUM_MOTORS])
{
    int32_t target_steps[NUM_MOTORS] = {0};
    for (int i = 0; i < NUM_MOTORS; i++) {
        const float clamped_angle = std::clamp(target_angles[i], HOME_ANGLE_RAD, MAX_ANGLE_RAD);
        target_steps[i] = angle_to_steps(clamped_angle - HOME_ANGLE_RAD);
    }

    bool changed = false;
    portENTER_CRITICAL(&s_motion_mux);
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (s_motion_state.target_steps[i] != target_steps[i]) {
            changed = true;
            s_motion_state.target_steps[i] = target_steps[i];
        }
    }
    if (changed && s_motion_state.plan.active) {
        s_motion_state.plan.replan_requested = true;
    }
    portEXIT_CRITICAL(&s_motion_mux);

    if (changed && s_motion_task != nullptr) {
        xTaskNotify(s_motion_task, kMotionNotifyTargetUpdate, eSetBits);
    }
}

void motor_get_current_angles(float current_angles[NUM_MOTORS])
{
    int32_t current_steps[NUM_MOTORS] = {0};
    portENTER_CRITICAL(&s_motion_mux);
    for (int i = 0; i < NUM_MOTORS; i++) {
        current_steps[i] = s_motion_state.current_steps[i];
    }
    portEXIT_CRITICAL(&s_motion_mux);

    for (int i = 0; i < NUM_MOTORS; i++) {
        current_angles[i] = steps_to_angle_rad(current_steps[i]);
    }
}
