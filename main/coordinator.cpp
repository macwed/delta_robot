#include "coordinator.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "delta_ik.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.hpp"

namespace {
constexpr float kDefaultManualTarget[3] = {0.f, 0.f, 0.f};
constexpr float kLegacyLowPoseZMm = 140.f;
constexpr float kLegacyHighPoseZMm = 210.f;

struct CoordinatorControl {
    float manual_xyz[3];
    bool demo_mode;
    bool paused;  // set by console diag commands to suppress coordinator output
};

CoordinatorControl s_control = {
    .manual_xyz = {kDefaultManualTarget[0], kDefaultManualTarget[1], kDefaultManualTarget[2]},
    .demo_mode = false,
};
portMUX_TYPE s_control_mux = portMUX_INITIALIZER_UNLOCKED;

float center_angle_for_z(float z_mm)
{
    return calc_arm_angle(kDeltaConfig, 0.f, 0.f, z_mm, 0);
}

float compute_home_height_mm()
{
    constexpr float kTargetAngle = HOME_ANGLE_RAD;
    constexpr float kSearchMinZ = 0.f;
    constexpr float kSearchMaxZ = 400.f;
    constexpr float kSearchStepZ = 1.f;

    float prev_z = kSearchMinZ;
    float prev_angle = center_angle_for_z(prev_z);
    if (!std::isfinite(prev_angle)) {
        prev_angle = NAN;
    }

    for (float z = kSearchMinZ + kSearchStepZ; z <= kSearchMaxZ; z += kSearchStepZ) {
        const float angle = center_angle_for_z(z);
        if (!std::isfinite(angle) || !std::isfinite(prev_angle)) {
            prev_z = z;
            prev_angle = angle;
            continue;
        }

        const float prev_error = prev_angle - kTargetAngle;
        const float error = angle - kTargetAngle;
        if ((prev_error <= 0.f && error >= 0.f) || (prev_error >= 0.f && error <= 0.f)) {
            float lo = prev_z;
            float hi = z;
            float lo_error = prev_error;

            for (int i = 0; i < 24; i++) {
                const float mid = 0.5f * (lo + hi);
                const float mid_angle = center_angle_for_z(mid);
                if (!std::isfinite(mid_angle)) {
                    break;
                }

                const float mid_error = mid_angle - kTargetAngle;
                if ((lo_error <= 0.f && mid_error >= 0.f) || (lo_error >= 0.f && mid_error <= 0.f)) {
                    hi = mid;
                } else {
                    lo = mid;
                    lo_error = mid_error;
                }
            }

            return 0.5f * (lo + hi);
        }

        prev_z = z;
        prev_angle = angle;
    }

    return 80.f;
}

float home_height_mm()
{
    static const float kHomeHeightMm = compute_home_height_mm();
    return kHomeHeightMm;
}

float machine_z_from_home_relative(float z_from_home_mm)
{
    return home_height_mm() + z_from_home_mm;
}

void set_manual_xyz(float x, float y, float z)
{
    portENTER_CRITICAL(&s_control_mux);
    s_control.manual_xyz[0] = x;
    s_control.manual_xyz[1] = y;
    s_control.manual_xyz[2] = z;
    s_control.demo_mode = false;
    portEXIT_CRITICAL(&s_control_mux);
}

void set_demo_mode(bool enabled)
{
    portENTER_CRITICAL(&s_control_mux);
    s_control.demo_mode = enabled;
    portEXIT_CRITICAL(&s_control_mux);
}

CoordinatorControl snapshot_control()
{
    CoordinatorControl snapshot = {};
    portENTER_CRITICAL(&s_control_mux);
    snapshot = s_control;
    portEXIT_CRITICAL(&s_control_mux);
    return snapshot;
}

bool validate_xyz(float x, float y, float z, float angles[NUM_MOTORS])
{
    return delta_ik(kDeltaConfig, x, y, machine_z_from_home_relative(z), angles);
}

// Block until motion engine is idle or timeout_ms elapses.
// Adds a short settle delay at the end.
void wait_for_idle(uint32_t timeout_ms)
{
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (!motor_is_idle()) {
        if (xTaskGetTickCount() >= deadline) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(150));
}

void set_paused(bool paused)
{
    portENTER_CRITICAL(&s_control_mux);
    s_control.paused = paused;
    portEXIT_CRITICAL(&s_control_mux);
}

// Move each motor individually by +20° and back so the user can identify
// physical arm order.  Coordinator is paused during the test.
void run_diag()
{
    set_paused(true);
    vTaskDelay(pdMS_TO_TICKS(30));  // let coordinator_task see the flag
    wait_for_idle(3000);

    float base_angles[NUM_MOTORS];
    motor_get_current_angles(base_angles);
    const float delta_rad = 20.f * DEG_TO_RAD;

    printf("\n--- DIAG: individual motor test ---\n");
    printf("Watch which physical arm moves and note its number.\n\n");
    fflush(stdout);

    for (int i = 0; i < NUM_MOTORS; i++) {
        printf("Motor %d moving UP\n", i + 1);
        fflush(stdout);

        float up[NUM_MOTORS];
        for (int j = 0; j < NUM_MOTORS; j++) up[j] = base_angles[j];
        up[i] = base_angles[i] + delta_rad;
        motor_set_target_angles(up);
        wait_for_idle(3000);

        printf("Motor %d moving DOWN\n", i + 1);
        fflush(stdout);
        motor_set_target_angles(base_angles);
        wait_for_idle(3000);

        if (i < NUM_MOTORS - 1) {
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    set_paused(false);
    printf("--- DIAG complete ---\n\n");
    fflush(stdout);
}

// Move effector along ±X and ±Y axes at z=45mm so the user can verify
// direction mapping.
void run_dirtest()
{
    constexpr float kZ = 45.f;
    constexpr float kOffset = 20.f;

    printf("\n--- DIRTEST: axis direction test (z=%.0fmm) ---\n\n", kZ);
    fflush(stdout);

    struct { float x; float y; const char *label; } steps[] = {
        {  kOffset, 0.f,      "Moving X positive (+20, 0)" },
        { -kOffset, 0.f,      "Moving X negative (-20, 0)" },
        { 0.f,  kOffset,      "Moving Y positive (0, +20)" },
        { 0.f, -kOffset,      "Moving Y negative (0, -20)" },
        { 0.f,  0.f,          "Returning to center (0, 0)" },
    };

    for (const auto &step : steps) {
        printf("%s\n", step.label);
        fflush(stdout);
        set_manual_xyz(step.x, step.y, kZ);
        wait_for_idle(4000);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }

    printf("--- DIRTEST complete ---\n\n");
    fflush(stdout);
}

void print_help()
{
    printf(
        "\nCommands:\n"
        "  xyz <x> <y> <z>  set manual target in mm relative to HOME\n"
        "  ms <1|2|4|8|16|32>  change DRV8825 microstep mode\n"
        "  tune             print current motion profile (ramp thresholds and delays)\n"
        "  reset            disable drivers 2s (arms fall to HOME), re-enable\n"
        "  diag             move each motor +20deg individually to identify arm order\n"
        "  dirtest          move effector ±X and ±Y to verify axis directions\n"
        "  demo on          enable demo motion\n"
        "  demo off         disable demo motion and hold manual target\n"
        "  status           print current mode and arm angles\n"
        "  help             print this help\n\n");
    fflush(stdout);
}

void print_status()
{
    const CoordinatorControl control = snapshot_control();
    float current_angles[NUM_MOTORS] = {0.f};
    motor_get_current_angles(current_angles);

    printf("Mode: %s\n", control.demo_mode ? "demo" : "manual");
    printf("Manual XYZ (home-relative): %.1f %.1f %.1f mm\n",
           control.manual_xyz[0], control.manual_xyz[1], control.manual_xyz[2]);
    printf("Home height: %.1f mm above motor plane\n", home_height_mm());
    printf("Microsteps: %d\n", motor_get_microsteps());
    printf("Arm angles: %.2f %.2f %.2f deg\n",
           current_angles[0] * 180.f / M_PI,
           current_angles[1] * 180.f / M_PI,
           current_angles[2] * 180.f / M_PI);
    fflush(stdout);
}

void handle_console_line(char *line)
{
    size_t len = std::strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
        line[len - 1] = '\0';
        len--;
    }

    if (line[0] == '\0') {
        return;
    }

    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    int microsteps = 0;
    char extra = '\0';
    if (std::sscanf(line, "xyz %f %f %f %c", &x, &y, &z, &extra) == 3) {
        float angles[NUM_MOTORS] = {0.f};
        if (!validate_xyz(x, y, z, angles)) {
            printf("Target out of range: %.1f %.1f %.1f mm\n", x, y, z);
            fflush(stdout);
            return;
        }

        set_manual_xyz(x, y, z);
        printf("Manual target set (home-relative): %.1f %.1f %.1f mm\n", x, y, z);
        printf("Machine Z: %.1f mm\n", machine_z_from_home_relative(z));
        printf("Arm target: %.2f %.2f %.2f deg\n",
               angles[0] * 180.f / M_PI,
               angles[1] * 180.f / M_PI,
               angles[2] * 180.f / M_PI);
        fflush(stdout);
        return;
    }

    if (std::sscanf(line, "ms %d %c", &microsteps, &extra) == 1) {
        if (!motor_set_microsteps(microsteps)) {
            printf("Microstep change failed. Use 1, 2, 4, 8, 16 or 32 while the robot is idle.\n");
            fflush(stdout);
            return;
        }

        printf("Microstep mode set to %d\n", microsteps);
        fflush(stdout);
        return;
    }

    if (std::strcmp(line, "demo on") == 0) {
        set_demo_mode(true);
        printf("Demo enabled\n");
        fflush(stdout);
        return;
    }

    if (std::strcmp(line, "demo off") == 0) {
        set_demo_mode(false);
        printf("Demo disabled\n");
        fflush(stdout);
        return;
    }

    if (std::strcmp(line, "tune") == 0) {
        motor_print_motion_profile();
        return;
    }

    if (std::strcmp(line, "reset") == 0) {
        printf("Resetting: disabling drivers for 2s, arms return to HOME.\n");
        fflush(stdout);
        set_paused(true);
        motor_reset();
        portENTER_CRITICAL(&s_control_mux);
        s_control.manual_xyz[0] = 0.f;
        s_control.manual_xyz[1] = 0.f;
        s_control.manual_xyz[2] = 0.f;
        portEXIT_CRITICAL(&s_control_mux);
        set_paused(false);
        printf("Reset complete. At HOME.\n");
        fflush(stdout);
        return;
    }

    if (std::strcmp(line, "diag") == 0) {
        run_diag();
        return;
    }

    if (std::strcmp(line, "dirtest") == 0) {
        run_dirtest();
        return;
    }

    if (std::strcmp(line, "status") == 0) {
        print_status();
        return;
    }

    if (std::strcmp(line, "help") == 0) {
        print_help();
        return;
    }

    printf("Unknown command: %s\n", line);
    print_help();
}
} // namespace

bool coordinator_set_manual_xyz(float x, float y, float z)
{
    float angles[NUM_MOTORS];
    if (!validate_xyz(x, y, z, angles)) {
        return false;
    }
    set_manual_xyz(x, y, z);
    return true;
}

void coordinator_task(void *p)
{
    (void)p;

    float angles[NUM_MOTORS] = {0.f};
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t phase_started = last_wake;
    bool high_pose = true;

    while (true)
    {
        const CoordinatorControl control = snapshot_control();
        bool ok = false;

        if (control.demo_mode) {
            const TickType_t now = xTaskGetTickCount();
            if ((now - phase_started) >= pdMS_TO_TICKS(2000)) {
                high_pose = !high_pose;
                phase_started = now;
            }

            ok = high_pose
                 ? delta_ik(kDeltaConfig, 20, 0, machine_z_from_home_relative(kLegacyHighPoseZMm - home_height_mm()), angles)
                 : delta_ik(kDeltaConfig, 0, 0, machine_z_from_home_relative(kLegacyLowPoseZMm - home_height_mm()), angles);
        } else {
            ok = delta_ik(kDeltaConfig,
                          control.manual_xyz[0],
                          control.manual_xyz[1],
                          machine_z_from_home_relative(control.manual_xyz[2]),
                          angles);
        }

        // motor_set_target_angles internally checks whether the target
        // actually changed, so calling it every cycle is safe — replanning
        // only happens when the XYZ target differs from the previous one.
        if (ok && !control.paused) {
            motor_set_target_angles(angles);
        }

        vTaskDelayUntil(&last_wake, COORDINATOR_PERIOD_TICKS);
    }
}

void coordinator_console_task(void *p)
{
    (void)p;

    setvbuf(stdin, nullptr, _IONBF, 0);
    setvbuf(stdout, nullptr, _IONBF, 0);

    printf("\nConsole ready. Manual mode is active.\n");
    print_help();
    print_status();
    printf("> ");
    fflush(stdout);

    char line[128];
    size_t line_len = 0;
    while (true) {
        const int ch = std::fgetc(stdin);
        if (ch == EOF) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            if (line_len == 0) {
                continue;
            }

            line[line_len] = '\0';
            printf("\n");
            handle_console_line(line);
            line_len = 0;
            printf("> ");
            fflush(stdout);
            continue;
        }

        if ((ch == '\b' || ch == 127) && line_len > 0) {
            line_len--;
            continue;
        }

        if (line_len < sizeof(line) - 1) {
            line[line_len++] = static_cast<char>(ch);
        }
    }
}
