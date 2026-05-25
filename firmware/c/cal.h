#pragma once

#include "pico/stdlib.h"

#include <stdbool.h>
#include <stdint.h>

#define CAL_MAX_POINTS 16

typedef struct {
    float    volts;
    float    hz;
    uint16_t dac_code;
    uint16_t _pad;
} cal_point_t;

typedef enum {
    CAL_SRC_DEFAULTS,   // compiled-in fallback
    CAL_SRC_FLASH,      // loaded from flash on boot
    CAL_SRC_RUNTIME,    // edited since boot, not yet persisted
} cal_src_t;

// Load from flash if a valid blob is present, otherwise install compiled defaults.
void cal_init(void);

// Convert PWM period in ns to volts via the current table.
float cal_period_ns_to_volts(uint32_t period_ns);

// Convert a target voltage to the interpolated DAC code via the current table.
uint16_t cal_volts_to_dac(float volts);

uint cal_num_points(void);
bool cal_get_point(uint i, cal_point_t *out);

// Capture the current voltage-PWM reading as a point at `volts` with the
// given DAC code. Returns false if PWM reading is invalid, volts is negative,
// or the table is full and no existing entry matches `volts`.
bool cal_capture(float volts, uint16_t dac_code);

// Remove the point at index `i`. Returns false if index is out of range.
bool cal_remove(uint i);

// Restore compiled defaults in RAM (call cal_save() to persist).
void cal_reset_to_defaults(void);

// Persist the current table and ramp config to flash.
// Must be called from Core 0.
bool cal_save(void);

cal_src_t cal_source(void);

// Ramp configuration — persisted alongside the cal table via cal_save().
void cal_get_ramp(uint16_t *step_max, uint16_t *step_min, uint32_t *tick_ms);
void cal_set_ramp(uint16_t step_max, uint16_t step_min, uint32_t tick_ms);
