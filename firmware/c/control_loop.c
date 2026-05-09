#include "control_loop.h"
#include "dac.h"
#include "psu_monitor.h"
#include "picoemp.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <math.h>

// ---------------------------------------------------------------------------
// Calibration tables
// ---------------------------------------------------------------------------

// Voltage → PWM frequency (Hz) — derived from bench measurements.
// Used to convert the voltage-feedback period_ns reading into volts.
typedef struct { float volts; float hz; } cal_point_t;

static const cal_point_t volt_cal[] = {
    {    0.0f,      0.0f },
    {   20.0f,    250.0f },
    {  100.0f,   1160.0f },
    {  500.0f,   6200.0f },
    { 1000.0f,  12425.0f },
    { 3000.0f,  37000.0f },
};
#define VOLT_CAL_N  (sizeof(volt_cal) / sizeof(volt_cal[0]))

// Voltage → DAC code (12-bit, 0-4095).
// Placeholder linear mapping — replace with bench-measured values.
static const cal_point_t dac_cal[] = {
    {    0.0f,    0.0f },
    { 3000.0f, 4095.0f },
};
#define DAC_CAL_N  (sizeof(dac_cal) / sizeof(dac_cal[0]))

// Linear interpolation between table entries.
static float interp(const cal_point_t *tbl, uint n, float x_volts, bool invert) {
    // invert=false: volts→y  invert=true: y→volts (for current, unused here)
    if (n < 2) return 0.0f;
    if (x_volts <= tbl[0].volts) return tbl[0].hz;
    if (x_volts >= tbl[n-1].volts) return tbl[n-1].hz;
    for (uint i = 1; i < n; i++) {
        if (x_volts <= tbl[i].volts) {
            float t = (x_volts - tbl[i-1].volts) / (tbl[i].volts - tbl[i-1].volts);
            return tbl[i-1].hz + t * (tbl[i].hz - tbl[i-1].hz);
        }
    }
    return tbl[n-1].hz;
    (void)invert;
}

// Convert period_ns → volts using the voltage calibration table (inverse lookup).
static float period_ns_to_volts(uint32_t period_ns) {
    if (period_ns == 0) return 0.0f;
    float hz = 1000000000.0f / (float)period_ns;
    // Reverse lookup: find volts given hz
    if (hz <= volt_cal[0].hz) return volt_cal[0].volts;
    if (hz >= volt_cal[VOLT_CAL_N-1].hz) return volt_cal[VOLT_CAL_N-1].volts;
    for (uint i = 1; i < VOLT_CAL_N; i++) {
        if (hz <= volt_cal[i].hz) {
            float t = (hz - volt_cal[i-1].hz) / (volt_cal[i].hz - volt_cal[i-1].hz);
            return volt_cal[i-1].volts + t * (volt_cal[i].volts - volt_cal[i-1].volts);
        }
    }
    return volt_cal[VOLT_CAL_N-1].volts;
}

// Convert volts → DAC code (clamped 0-4095).
static uint16_t volts_to_dac(float volts) {
    float code = interp(dac_cal, DAC_CAL_N, volts, false);
    if (code < 0.0f) code = 0.0f;
    if (code > 4095.0f) code = 4095.0f;
    return (uint16_t)code;
}

// ---------------------------------------------------------------------------
// Control parameters
// ---------------------------------------------------------------------------

// Max DAC counts to change per control tick to limit slew rate.
#define RAMP_STEP_MAX    20u

// Error deadband in volts — don't adjust DAC if within this window.
#define DEADBAND_VOLTS   5.0f

// Control tick interval in µs (10 ms).
#define TICK_INTERVAL_US 10000u

// Current: rough conversion — placeholder (period_ns → amps not yet calibrated).
// Returns Hz for now; replace with calibrated formula.
static float period_ns_to_current(uint32_t period_ns) {
    if (period_ns == 0) return 0.0f;
    return 1000000000.0f / (float)period_ns;  // Hz placeholder
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static bool     cl_enabled      = false;
static float    target_volts    = 0.0f;
static float    soft_limit      = HARD_LIMIT_VOLTS;
static uint16_t current_dac     = 0;
static float    actual_volts    = 0.0f;
static float    actual_current  = 0.0f;
static uint32_t fault_reg       = 0;
static uint64_t last_tick_us    = 0;

// LED flash state machine
static uint64_t led_next_us     = 0;
static bool     led_state       = false;
static uint32_t led_flash_count = 0;  // flashes remaining in current burst
static uint64_t led_pause_until = 0;

#define LED_ON_US    100000u   // 100 ms on
#define LED_OFF_US    80000u   // 80 ms between flashes
#define LED_PAUSE_US 600000u   // 600 ms between bursts

static void set_dac_safe(uint16_t code) {
    if (!dac_write_done()) return;  // previous write still in flight
    current_dac = code;
    dac_write(code);
}

static void shutdown_output() {
    set_dac_safe(0);
    current_dac = 0;
    gpio_put(PIN_OUT_HV_Enable, false);
}

static void check_faults() {
    if (gpio_get(PIN_IN_Circuit_Open))    fault_reg |= FAULT_CIRCUIT_OPEN;
    if (gpio_get(PIN_IN_Circuit_Shorted)) fault_reg |= FAULT_CIRCUIT_SHORTED;
    if (gpio_get(PIN_IN_Circuit_UNKNOWN)) fault_reg |= FAULT_CIRCUIT_UNKNOWN;

    float hard_ceil = (HARD_LIMIT_VOLTS < soft_limit) ? HARD_LIMIT_VOLTS : soft_limit;
    if (actual_volts > hard_ceil + DEADBAND_VOLTS) fault_reg |= FAULT_OVERVOLTAGE;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void control_loop_init() {
    current_dac  = 0;
    target_volts = 0.0f;
    soft_limit   = HARD_LIMIT_VOLTS;
    fault_reg    = 0;
    cl_enabled   = false;
    last_tick_us = time_us_64();
}

void control_loop_tick() {
    uint64_t now = time_us_64();
    if ((now - last_tick_us) < TICK_INTERVAL_US) return;
    last_tick_us = now;

    // Update feedback
    actual_volts   = period_ns_to_volts(
        psu_monitor_voltage_is_valid() ? psu_monitor_get_voltage_period_ns() : 0);
    actual_current = period_ns_to_current(
        psu_monitor_current_is_valid() ? psu_monitor_get_current_period_ns() : 0);

    check_faults();

    if (!cl_enabled || fault_reg != 0) {
        if (fault_reg != 0) shutdown_output();
        return;
    }

    // Clamp target to hard and soft limits
    float ceiling = (HARD_LIMIT_VOLTS < soft_limit) ? HARD_LIMIT_VOLTS : soft_limit;
    float setpoint = (target_volts > ceiling) ? ceiling : target_volts;

    float error = setpoint - actual_volts;

    // Deadband — no adjustment needed
    if (fabsf(error) <= DEADBAND_VOLTS) return;

    // Desired DAC code for setpoint
    uint16_t desired_dac = volts_to_dac(setpoint);

    // Ramp: limit step size
    int32_t delta = (int32_t)desired_dac - (int32_t)current_dac;
    if (delta >  (int32_t)RAMP_STEP_MAX) delta =  (int32_t)RAMP_STEP_MAX;
    if (delta < -(int32_t)RAMP_STEP_MAX) delta = -(int32_t)RAMP_STEP_MAX;

    uint16_t next_dac = (uint16_t)((int32_t)current_dac + delta);
    if (next_dac > 4095) next_dac = 4095;

    set_dac_safe(next_dac);
}

void control_loop_led_tick() {
    if (fault_reg == 0) {
        gpio_put(PIN_LED_STATUS, true);
        led_flash_count = 0;
        return;
    }

    uint64_t now = time_us_64();

    // Count number of set fault bits for burst length
    uint32_t bits = fault_reg;
    uint count = 0;
    while (bits) { count += bits & 1u; bits >>= 1; }

    if (now < led_pause_until) return;

    if (led_flash_count == 0) {
        led_flash_count = count;
        led_next_us = now;
    }

    if (now < led_next_us) return;

    if (led_state) {
        gpio_put(PIN_LED_STATUS, false);
        led_state = false;
        led_flash_count--;
        if (led_flash_count == 0) {
            led_next_us = now + LED_PAUSE_US;
            led_pause_until = led_next_us;
        } else {
            led_next_us = now + LED_OFF_US;
        }
    } else {
        gpio_put(PIN_LED_STATUS, true);
        led_state = true;
        led_next_us = now + LED_ON_US;
    }
}

void control_loop_enable(bool en) {
    cl_enabled = en;
    if (!en) shutdown_output();
}

bool control_loop_is_enabled() { return cl_enabled; }

void control_loop_set_target_volts(float volts) {
    float ceiling = (HARD_LIMIT_VOLTS < soft_limit) ? HARD_LIMIT_VOLTS : soft_limit;
    if (volts < 0.0f)    volts = 0.0f;
    if (volts > ceiling) volts = ceiling;
    target_volts = volts;
}

float control_loop_get_target_volts() { return target_volts; }

void control_loop_set_soft_limit(float volts) {
    if (volts > HARD_LIMIT_VOLTS) volts = HARD_LIMIT_VOLTS;
    if (volts < 0.0f) volts = 0.0f;
    soft_limit = volts;
    // Re-clamp target
    if (target_volts > soft_limit) target_volts = soft_limit;
}

float control_loop_get_soft_limit() { return soft_limit; }

float control_loop_get_actual_volts()   { return actual_volts; }
float control_loop_get_actual_current() { return actual_current; }

uint32_t control_loop_get_faults() { return fault_reg; }

void control_loop_clear_faults() {
    fault_reg = 0;
}
