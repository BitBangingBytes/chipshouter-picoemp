#pragma once

// FIFO commands
#define cmd_arm 0
#define cmd_disarm 1
#define cmd_pulse 2
#define cmd_status 3
#define cmd_enable_timeout 4
#define cmd_disable_timeout 5
#define cmd_fast_trigger 6
#define cmd_internal_hvp 7
#define cmd_external_hvp 8
#define cmd_config_pulse_time 9
// cmd_config_pulse_power (10) removed — external PSU replaces on-board HV generation
#define cmd_toggle_gp1 11
#define cmd_config_pulse_delay_cycles 12
#define cmd_config_pulse_time_cycles 13
#define cmd_read_voltage_pwm 14
#define cmd_read_current_pwm 15
#define cmd_hv_enable 16
#define cmd_hv_disable 17
// cmd word + float bits follow
#define cmd_set_voltage 18
#define cmd_get_actual_voltage 19
#define cmd_get_actual_current 20
#define cmd_get_faults 21
#define cmd_clear_faults 22
// cmd word + float bits follow
#define cmd_set_soft_limit 23
// cmd word + uint16 (0-4095) value follows. Triggers manual_mode on the control
// loop when issued while HV is enabled so the closed loop stops fighting the write.
#define cmd_debug_dac_raw 24

// Calibration commands — runtime cal table for PWM→volts.
// cmd_cal_capture: cmd word + float bits (volts) follow.
#define cmd_cal_capture 25
// cmd_cal_remove: cmd word + uint32 index follows.
#define cmd_cal_remove  26
#define cmd_cal_reset   27
#define cmd_cal_save    28
// cmd_cal_list: Core 0 pushes ok, then n_points, then for each point
// pushes volts (ui32 float bits) and hz (ui32 float bits), then source enum.
#define cmd_cal_list    29

// cmd word + uint32 interval_ms follows (0 = disable).
#define cmd_set_dac_refresh 30
#define cmd_get_dac_refresh 31

// cmd word + uint32 (0/1) follows.
#define cmd_set_fault_ignore 32
#define cmd_get_fault_ignore 33

#define return_ok 0
#define return_failed 1

void serial_console();
