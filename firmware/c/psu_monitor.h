#pragma once

#include <stdbool.h>
#include <stdint.h>

// If no edge arrives within these windows the reading is flagged stale
#define VOLTAGE_STALE_THRESHOLD_US   50000U   // 50ms  (~3 cycles at 60Hz minimum)
#define CURRENT_STALE_THRESHOLD_US  1000000U  // 1s    (~3 cycles at 3Hz minimum)

void     psu_monitor_init();

uint32_t psu_monitor_get_voltage_period_us();
uint32_t psu_monitor_get_current_period_us();

bool psu_monitor_voltage_is_valid();
bool psu_monitor_current_is_valid();
