#pragma once

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

extern const uint32_t PIN_IN_TRIGGER;
extern const uint32_t PIN_LED_HV;
extern const uint32_t PIN_LED_STATUS;
extern const uint32_t PIN_BTN_PULSE;
extern const uint32_t PIN_OUT_HVPULSE;
extern const uint32_t PIN_IN_CHARGED;
// extern const uint32_t PIN_OUT_HVPWM;  // disabled: external PSU replaces on-board HV generation
extern const uint32_t PIN_LED_CHARGE_ON;
extern const uint32_t PIN_BTN_ARM;

// External PSU inputs
extern const uint32_t PIN_IN_Voltage_PWM;
extern const uint32_t PIN_IN_Current_PWM;
extern const uint32_t PIN_IN_Circuit_Open;
extern const uint32_t PIN_IN_Circuit_Shorted;
extern const uint32_t PIN_IN_Circuit_UNKNOWN;

// External PSU outputs
extern const uint32_t PIN_OUT_HV_Enable;
extern const uint32_t PIN_OUT_HV_Data;
extern const uint32_t PIN_OUT_HV_Clock;
extern const uint32_t PIN_OUT_HV_Strobe;

// void picoemp_enable_pwm(float duty_frac);  // disabled: external PSU replaces on-board HV generation
// void picoemp_disable_pwm();                // disabled: external PSU replaces on-board HV generation
void picoemp_pulse(uint32_t pulse_time);
void picoemp_configure_pulse_output();
void picoemp_configure_pulse_external();
void picoemp_init();
