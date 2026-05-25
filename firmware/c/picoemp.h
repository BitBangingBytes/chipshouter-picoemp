#pragma once

#include "hardware/gpio.h"

extern const uint32_t PIN_LED_STATUS;

// External PSU inputs
extern const uint32_t PIN_IN_Voltage_PWM;
extern const uint32_t PIN_IN_Current_PWM;
extern const uint32_t PIN_IN_Circuit_Open;
extern const uint32_t PIN_IN_Circuit_Shorted;
extern const uint32_t PIN_IN_Circuit_UNKNOWN;

// External PSU outputs (DAC SPI + HV enable)
extern const uint32_t PIN_OUT_HV_Enable;
extern const uint32_t PIN_OUT_HV_Data;
extern const uint32_t PIN_OUT_HV_Clock;
extern const uint32_t PIN_OUT_HV_Strobe;

void picoemp_init();
