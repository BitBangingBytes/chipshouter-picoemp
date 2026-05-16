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

// Max DAC counts to change per control tick to limit slew rate.
#define RAMP_STEP_MAX    100u

// Error deadband in volts — don't adjust DAC if within this window.
#define DEADBAND_VOLTS   1.0f

// Proportional gain: DAC counts per volt of error.
// Roughly the slope of the V→DAC characteristic (~2-2.8 codes/V across range).
#define KP_DAC_PER_VOLT  1.5f

// Starting DAC code on enable; closed-loop ramps from here toward setpoint.
#define INITIAL_DAC_CODE 10u

// Minimum wall-clock time between DAC adjustments. Floors the update rate so
// the PSU has time to respond before the next correction — without this, at
// high voltages a fresh PWM average can arrive every few ms and the loop hunts.
#define MIN_ADJUST_DWELL_US 250000u   // 250 ms

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
static float    soft_limit         = HARD_LIMIT_VOLTS;
static uint16_t current_dac        = 0;
static float    actual_volts       = 0.0f;
static float    actual_current     = 0.0f;
static uint32_t fault_reg          = 0;
static uint32_t last_processed_seq = 0;  // last voltage-PWM sample_seq we acted on
static bool     shutdown_latched   = false; // true while fault shutdown_output has already run
static uint64_t last_adjust_us     = 0;     // wall time of last DAC adjustment (for MIN_ADJUST_DWELL_US)
static bool     manual_mode        = false; // closed-loop paused for raw DAC writes

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
    gpio_put(PIN_OUT_HV_Enable, true);   // HIGH = disabled
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
    current_dac        = 0;
    target_volts       = 0.0f;
    soft_limit         = HARD_LIMIT_VOLTS;
    fault_reg          = 0;
    cl_enabled         = false;
    last_processed_seq = 0;
    shutdown_latched   = false;
    last_adjust_us     = 0;
    manual_mode        = false;
    gpio_put(PIN_OUT_HV_Enable, true);   // HIGH = disabled at startup
}

void control_loop_tick() {
    // Always refresh feedback and check faults so safety/observability work
    // independently of how often a fresh PWM average arrives.
    actual_volts   = cal_period_ns_to_volts(
        psu_monitor_voltage_is_valid() ? psu_monitor_get_voltage_period_ns() : 0);
    actual_current = period_ns_to_current(
        psu_monitor_current_is_valid() ? psu_monitor_get_current_period_ns() : 0);

    check_faults();

    if (fault_reg != 0) {
        // Latch the shutdown so we don't keep firing DAC writes (CLK/STR pulses)
        // every main-loop iteration while the fault persists.
        if (!shutdown_latched) {
            shutdown_output();
            shutdown_latched = true;
        }
        return;
    }
    if (!cl_enabled) return;

    // Manual DAC mode (entered by raw `dd` while HV enabled): keep feedback /
    // fault paths alive but stop driving the DAC ourselves.
    if (manual_mode) return;

    // Gate the DAC adjustment on a freshly completed PWM average.
    uint32_t seq = psu_monitor_voltage_sample_seq();
    if (seq == last_processed_seq) return;
    last_processed_seq = seq;

    // Clamp target to hard and soft limits
    float ceiling = (HARD_LIMIT_VOLTS < soft_limit) ? HARD_LIMIT_VOLTS : soft_limit;
    float setpoint = (target_volts > ceiling) ? ceiling : target_volts;

    float error = setpoint - actual_volts;

    // Deadband — no adjustment needed
    if (fabsf(error) <= DEADBAND_VOLTS) return;

    // Minimum dwell between adjustments — caps update rate even when fresh PWM
    // averages arrive faster than the PSU can respond.
    uint64_t now = time_us_64();
    if ((now - last_adjust_us) < MIN_ADJUST_DWELL_US) return;
    last_adjust_us = now;

    // Proportional control: DAC drifts toward whatever code produces setpoint.
    // Sign of error drives direction; ramp limit caps slew rate.
    int32_t delta = (int32_t)(error * KP_DAC_PER_VOLT);
    if (delta >  (int32_t)RAMP_STEP_MAX) delta =  (int32_t)RAMP_STEP_MAX;
    if (delta < -(int32_t)RAMP_STEP_MAX) delta = -(int32_t)RAMP_STEP_MAX;

    int32_t next_dac = (int32_t)current_dac + delta;
    if (next_dac < 0)    next_dac = 0;
    if (next_dac > 4095) next_dac = 4095;

    set_dac_safe((uint16_t)next_dac);
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
    cl_enabled  = en;
    manual_mode = false;   // any enable/disable transition exits manual mode
    if (en) {
        gpio_put(PIN_OUT_HV_Enable, false);  // LOW = enabled — assert before first DAC write
        // Seed DAC at a low starting code; closed-loop ramps from here.
        while (!dac_write_done()) tight_loop_contents();
        current_dac = INITIAL_DAC_CODE;
        dac_write(INITIAL_DAC_CODE);
        // Start dwell timer from the seed so the first closed-loop adjustment
        // waits MIN_ADJUST_DWELL_US, giving the PSU time to respond.
        last_adjust_us = time_us_64();
    } else {
        shutdown_output();                   // zeros DAC, then sets Enable HIGH (disabled)
    }
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
    shutdown_latched = false;
}

void control_loop_set_manual_mode(bool en) { manual_mode = en; }
bool control_loop_in_manual_mode()         { return manual_mode; }
