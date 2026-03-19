#include "coordinator.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "delta_ik.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.hpp"

namespace {
struct CoordinatorControl {
    float manual_xyz[3];
    bool demo_mode;
};

CoordinatorControl s_control = {
    .manual_xyz = {0.f, 0.f, 140.f},
    .demo_mode = false,
};
portMUX_TYPE s_control_mux = portMUX_INITIALIZER_UNLOCKED;

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
    return delta_ik(kDeltaConfig, x, y, z, angles);
}

void print_help()
{
    printf(
        "\nCommands:\n"
        "  xyz <x> <y> <z>  set manual target in mm\n"
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
    printf("Manual XYZ: %.1f %.1f %.1f mm\n",
           control.manual_xyz[0], control.manual_xyz[1], control.manual_xyz[2]);
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
    char extra = '\0';
    if (std::sscanf(line, "xyz %f %f %f %c", &x, &y, &z, &extra) == 3) {
        float angles[NUM_MOTORS] = {0.f};
        if (!validate_xyz(x, y, z, angles)) {
            printf("Target out of range: %.1f %.1f %.1f mm\n", x, y, z);
            fflush(stdout);
            return;
        }

        set_manual_xyz(x, y, z);
        printf("Manual target set: %.1f %.1f %.1f mm\n", x, y, z);
        printf("Arm target: %.2f %.2f %.2f deg\n",
               angles[0] * 180.f / M_PI,
               angles[1] * 180.f / M_PI,
               angles[2] * 180.f / M_PI);
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
                 ? delta_ik(kDeltaConfig, 20, 0, 210.f, angles)
                 : delta_ik(kDeltaConfig, 0, 0, 140.f, angles);
        } else {
            ok = delta_ik(kDeltaConfig,
                          control.manual_xyz[0],
                          control.manual_xyz[1],
                          control.manual_xyz[2],
                          angles);
        }

        if (ok) {
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
