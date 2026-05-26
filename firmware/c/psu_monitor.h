#pragma once

#include <stdbool.h>
#include <stdint.h>

// If no PIO sample arrives within these windows the reading is flagged stale.
#define VOLTAGE_STALE_THRESHOLD_US   50000U   // 50ms
#define CURRENT_STALE_THRESHOLD_US  1000000U  // 1s

void psu_monitor_init();

// Must be called from Core 0 main loop to drain PIO FIFOs and update averages.
void psu_monitor_update();

// Period in nanoseconds (24 ns resolution). Returns 0 if not yet measured.
uint32_t psu_monitor_get_voltage_period_ns();
uint32_t psu_monitor_get_current_period_ns();

bool psu_monitor_voltage_is_valid();
bool psu_monitor_current_is_valid();
