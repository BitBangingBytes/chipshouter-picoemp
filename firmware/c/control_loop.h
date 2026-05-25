#pragma once

#include <stdint.h>
#include <stdbool.h>

// Fault bit flags — OR'd into the fault register
#define FAULT_CIRCUIT_OPEN    (1u << 0)
#define FAULT_CIRCUIT_SHORTED (1u << 1)
#define FAULT_CIRCUIT_UNKNOWN (1u << 2)
#define FAULT_OVERVOLTAGE     (1u << 3)
#define FAULT_OVERCURRENT     (1u << 4)

// Absolute hardware ceiling — DAC interpolation is clamped so output never exceeds this.
#define HARD_LIMIT_VOLTS 1500.0f

// DAC code seeded on HV enable; open-loop ramps from here toward the interpolated target.
#define INITIAL_DAC_CODE 10u

// Initialize control loop (call after dac_init, psu_monitor_init, and cal_init).
void control_loop_init();

// Must be called every iteration of the Core 0 main loop.
void control_loop_tick();

// Must be called every iteration of the Core 0 main loop (LED flash state machine).
void control_loop_led_tick();

// Enable / disable HV output.  Disabling zeros the DAC and deasserts HV_Enable.
void control_loop_enable(bool en);
bool control_loop_is_enabled();

// Target voltage setpoint.  Clamped to [0, soft_limit] on set.
// Also resets the parabolic ramp origin so the next tick ramps from current_dac.
void control_loop_set_target_volts(float volts);
float control_loop_get_target_volts();

// Read-back of estimated actual values derived from PWM feedback.
float control_loop_get_actual_volts();
float control_loop_get_actual_current();

// Current DAC code being driven.
uint16_t control_loop_get_current_dac();
void     control_loop_set_current_dac(uint16_t code);

// Fault register — sticky bits cleared only by control_loop_clear_faults().
uint32_t control_loop_get_faults();
void control_loop_clear_faults();

// Manual-DAC mode. When enabled (set by a raw `dd` write while HV is on),
// the open-loop ramp stops issuing DAC adjustments but still refreshes feedback
// and checks faults. Cleared on every control_loop_enable() transition.
void control_loop_set_manual_mode(bool en);
bool control_loop_in_manual_mode();

// Fault ignore mode. When enabled, faults are still recorded (readable via
// get_faults) but do not trigger a shutdown. Use with caution.
void control_loop_set_faults_ignored(bool en);
bool control_loop_get_faults_ignored();

// Periodic DAC refresh interval. Set to 0 to disable.
void control_loop_set_dac_refresh_ms(uint32_t ms);
uint32_t control_loop_get_dac_refresh_ms();
