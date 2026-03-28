#pragma once

void coordinator_task(void *p);
void coordinator_console_task(void *p);

// Set manual XYZ target (home-relative mm). Returns false if out of IK range.
bool coordinator_set_manual_xyz(float x, float y, float z);
