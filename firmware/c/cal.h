#pragma once

#include "pico/stdlib.h"

#include <stdbool.h>
#include <stdint.h>

#define CAL_MAX_POINTS 16

typedef struct { float volts; float hz; } cal_point_t;

typedef enum {
    CAL_SRC_DEFAULTS,   // compiled-in fallback
    CAL_SRC_FLASH,      // loaded from flash on boot
    CAL_SRC_RUNTIME,    // edited since boot, not yet persisted
} cal_src_t;

// Load from flash if a valid blob is present, otherwise install compiled defaults.
void cal_init(void);

// Convert PWM period in ns to volts via the current table.
float cal_period_ns_to_volts(uint32_t period_ns);

uint cal_num_points(void);
bool cal_get_point(uint i, cal_point_t *out);

// Capture the current voltage-PWM reading as a point at `volts`.
// Returns false if PWM reading is invalid, volts is negative, or the table is full
// and no existing entry matches `volts`. Updates an existing entry (within ε) in place.
bool cal_capture(float volts);

// Remove the point at index `i`. Returns false if index is out of range.
bool cal_remove(uint i);

// Restore compiled defaults in RAM (call cal_save() to persist).
void cal_reset_to_defaults(void);

// Persist the current table to flash. Must be called from Core 0; uses
// flash_safe_execute() to lock out Core 1 during the operation.
bool cal_save(void);

cal_src_t cal_source(void);
