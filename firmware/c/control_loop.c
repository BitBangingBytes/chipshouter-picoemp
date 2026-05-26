#include "control_loop.h"
#include "cal.h"
#include "dac.h"
#include "psu_monitor.h"
#include "picoemp.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <math.h>

// ---------------------------------------------------------------------------
// Control parameters
// ---------------------------------------------------------------------------

// Periodic DAC refresh: resend current code unconditionally at this interval
// even when the ramp is complete. Keeps the DAC consistent after noise glitches
// on the SPI bus. 0 disables the refresh.
#define DAC_REFRESH_INTERVAL_MS_DEFAULT 50u

// Current: rough conversion — placeholder (period_ns → amps not yet calibrated).
// Returns Hz for now; replace with calibrated formula.
static float period_ns_to_current(uint32_t period_ns) {
    if (period_ns == 0) return 0.0f;
    return 1000000000.0f / (float)period_ns;  // Hz placeholder
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static bool     cl_enabled         = false;
static float    target_volts       = 0.0f;
static uint16_t current_dac        = 0;
static float    actual_volts       = 0.0f;
static float    actual_current     = 0.0f;
static uint32_t fault_reg          = 0;
static bool     shutdown_latched   = false;
static bool     manual_mode        = false;
static bool     faults_ignored     = false;
static uint32_t dac_refresh_interval_ms = DAC_REFRESH_INTERVAL_MS_DEFAULT;
static uint64_t last_dac_refresh_us     = 0;

// Parabolic ramp state
static uint16_t ramp_start_dac      = 0;
static uint32_t ramp_start_distance = 0;
static uint64_t last_ramp_tick_us   = 0;

// LED flash state machine
static uint64_t led_next_us     = 0;
static bool     led_state       = false;
static uint32_t led_flash_count = 0;
static uint64_t led_pause_until = 0;

#define LED_ON_US    150000u
#define LED_OFF_US   100000u
#define LED_PAUSE_US 1000000u

static void set_dac_safe(uint16_t code) {
    if (!dac_write_done()) return;
    current_dac = code;
    dac_write(code);
}

static void shutdown_output() {
    set_dac_safe(0);
    current_dac = 0;
    gpio_put(PIN_OUT_HV_Enable, true);
    dac_deselect();
}

static void check_faults() {
    if (gpio_get(PIN_IN_Circuit_Open))    fault_reg |= FAULT_CIRCUIT_OPEN;
    if (gpio_get(PIN_IN_Circuit_Shorted)) fault_reg |= FAULT_CIRCUIT_SHORTED;
    if (gpio_get(PIN_IN_Circuit_UNKNOWN)) fault_reg |= FAULT_CIRCUIT_UNKNOWN;

    if (actual_volts > cal_get_hard_limit()) fault_reg |= FAULT_OVERVOLTAGE;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void control_loop_init() {
    current_dac             = 0;
    target_volts            = 0.0f;
    fault_reg               = 0;
    cl_enabled              = false;
    shutdown_latched        = false;
    manual_mode             = false;
    faults_ignored          = false;
    dac_refresh_interval_ms = DAC_REFRESH_INTERVAL_MS_DEFAULT;
    last_dac_refresh_us     = 0;
    ramp_start_dac          = 0;
    ramp_start_distance     = 0;
    last_ramp_tick_us       = 0;
    gpio_put(PIN_OUT_HV_Enable, true);
}

void control_loop_tick() {
    actual_volts   = cal_period_ns_to_volts(
        psu_monitor_voltage_is_valid() ? psu_monitor_get_voltage_period_ns() : 0);
    actual_current = period_ns_to_current(
        psu_monitor_current_is_valid() ? psu_monitor_get_current_period_ns() : 0);

    check_faults();

    if (fault_reg != 0 && !faults_ignored) {
        if (!shutdown_latched) {
            shutdown_output();
            shutdown_latched = true;
        }
        return;
    }
    if (!cl_enabled) return;
    if (manual_mode) return;

    // Periodic DAC refresh — resend the current code unconditionally.
    uint64_t now = time_us_64();
    if (dac_refresh_interval_ms > 0 &&
        (now - last_dac_refresh_us) >= (uint64_t)dac_refresh_interval_ms * 1000u) {
        last_dac_refresh_us = now;
        set_dac_safe(current_dac);
    }

    // Throttle ramp to cal-configured tick interval.
    uint16_t step_max; uint16_t step_min; uint32_t tick_ms;
    cal_get_ramp(&step_max, &step_min, &tick_ms);

    if ((now - last_ramp_tick_us) < (uint64_t)tick_ms * 1000u) return;
    last_ramp_tick_us = now;

    uint16_t target_dac = cal_volts_to_dac(target_volts);
    if (current_dac == target_dac) return;

    uint32_t remaining = (target_dac > current_dac)
                         ? (uint32_t)(target_dac - current_dac)
                         : (uint32_t)(current_dac - target_dac);

    // Parabolic deceleration profile: step² interpolates between step_max² (at
    // ramp start) and step_min² (at target), producing smooth kinematic braking.
    float fraction = (ramp_start_distance > 0)
                     ? (float)remaining / (float)ramp_start_distance : 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;
    float step_f = sqrtf((float)step_max * (float)step_max * fraction
                       + (float)step_min * (float)step_min * (1.0f - fraction));
    uint16_t step = (uint16_t)(step_f + 0.5f);
    if (step < step_min)            step = step_min;
    if (step > step_max)            step = step_max;
    if ((uint32_t)step > remaining) step = (uint16_t)remaining;

    if (target_dac > current_dac)
        set_dac_safe(current_dac + step);
    else
        set_dac_safe(current_dac - step);
}

void control_loop_led_tick() {
    if (fault_reg == 0) {
        gpio_put(PIN_LED_STATUS, true);
        led_flash_count = 0;
        return;
    }

    uint64_t now = time_us_64();

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
    cl_enabled  = en;
    manual_mode = false;
    if (en) {
        gpio_put(PIN_OUT_HV_Enable, false);
        while (!dac_write_done()) tight_loop_contents();
        current_dac         = INITIAL_DAC_CODE;
        ramp_start_dac      = INITIAL_DAC_CODE;
        uint16_t tdac       = cal_volts_to_dac(target_volts);
        ramp_start_distance = (tdac > INITIAL_DAC_CODE)
                              ? (uint32_t)(tdac - INITIAL_DAC_CODE)
                              : (uint32_t)(INITIAL_DAC_CODE - tdac);
        last_ramp_tick_us   = time_us_64();
        dac_write(INITIAL_DAC_CODE);
    } else {
        shutdown_output();
    }
}

bool control_loop_is_enabled() { return cl_enabled; }

void control_loop_set_target_volts(float volts) {
    if (volts < 0.0f)                  volts = 0.0f;
    if (volts > cal_get_hard_limit())  volts = cal_get_hard_limit();
    target_volts = volts;

    // Capture ramp origin so the parabolic profile starts from current position.
    ramp_start_dac          = current_dac;
    uint16_t target_dac     = cal_volts_to_dac(target_volts);
    ramp_start_distance     = (target_dac > current_dac)
                              ? (uint32_t)(target_dac - current_dac)
                              : (uint32_t)(current_dac - target_dac);
}

float control_loop_get_target_volts() { return target_volts; }

float control_loop_get_actual_volts()   { return actual_volts; }
float control_loop_get_actual_current() { return actual_current; }
uint16_t control_loop_get_current_dac()          { return current_dac; }
void     control_loop_set_current_dac(uint16_t code) { current_dac = code; }

uint32_t control_loop_get_faults() { return fault_reg; }

void control_loop_clear_faults() {
    fault_reg = 0;
    shutdown_latched = false;
}

void control_loop_set_manual_mode(bool en) { manual_mode = en; }
bool control_loop_in_manual_mode()         { return manual_mode; }

void control_loop_set_faults_ignored(bool en) { faults_ignored = en; }
bool control_loop_get_faults_ignored()         { return faults_ignored; }

void control_loop_set_dac_refresh_ms(uint32_t ms) { dac_refresh_interval_ms = ms; }
uint32_t control_loop_get_dac_refresh_ms()         { return dac_refresh_interval_ms; }
