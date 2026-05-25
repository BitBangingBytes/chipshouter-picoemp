#include "picoemp.h"
#include "psu_monitor.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

const uint32_t PIN_LED_STATUS = 7;

// External PSU inputs
const uint32_t PIN_IN_Voltage_PWM     = 2;
const uint32_t PIN_IN_Current_PWM     = 3;
const uint32_t PIN_IN_Circuit_Open    = 4;
const uint32_t PIN_IN_Circuit_Shorted = 5;
const uint32_t PIN_IN_Circuit_UNKNOWN = 8;

// External PSU outputs
const uint32_t PIN_OUT_HV_Enable = 19;
const uint32_t PIN_OUT_HV_Data   = 20;
const uint32_t PIN_OUT_HV_Clock  = 21;
const uint32_t PIN_OUT_HV_Strobe = 22;

void picoemp_init() {
    // Status LED
    gpio_init(PIN_LED_STATUS);
    gpio_set_dir(PIN_LED_STATUS, GPIO_OUT);
    gpio_put(PIN_LED_STATUS, true);

    // External PSU inputs
    gpio_init(PIN_IN_Voltage_PWM);
    gpio_init(PIN_IN_Current_PWM);
    gpio_init(PIN_IN_Circuit_Open);
    gpio_init(PIN_IN_Circuit_Shorted);
    gpio_init(PIN_IN_Circuit_UNKNOWN);
    gpio_set_dir(PIN_IN_Voltage_PWM,     GPIO_IN);
    gpio_set_dir(PIN_IN_Current_PWM,     GPIO_IN);
    gpio_set_dir(PIN_IN_Circuit_Open,    GPIO_IN);
    gpio_set_dir(PIN_IN_Circuit_Shorted, GPIO_IN);
    gpio_set_dir(PIN_IN_Circuit_UNKNOWN, GPIO_IN);
    gpio_set_pulls(PIN_IN_Voltage_PWM,     false, false);
    gpio_set_pulls(PIN_IN_Current_PWM,     false, false);
    gpio_set_pulls(PIN_IN_Circuit_Open,    false, false);
    gpio_set_pulls(PIN_IN_Circuit_Shorted, false, false);
    gpio_set_pulls(PIN_IN_Circuit_UNKNOWN, false, false);

    // External PSU outputs
    gpio_init(PIN_OUT_HV_Enable);
    gpio_init(PIN_OUT_HV_Data);
    gpio_init(PIN_OUT_HV_Clock);
    gpio_init(PIN_OUT_HV_Strobe);
    gpio_set_dir(PIN_OUT_HV_Enable, GPIO_OUT);
    gpio_set_dir(PIN_OUT_HV_Data,   GPIO_OUT);
    gpio_set_dir(PIN_OUT_HV_Clock,  GPIO_OUT);
    gpio_set_dir(PIN_OUT_HV_Strobe, GPIO_OUT);
    gpio_put(PIN_OUT_HV_Enable, true);   // HIGH = disabled
    gpio_put(PIN_OUT_HV_Data,   true);   // idle HIGH (logic 0)
    gpio_put(PIN_OUT_HV_Clock,  true);   // idle HIGH
    gpio_put(PIN_OUT_HV_Strobe, true);   // idle HIGH (chip deselected; active-low)

    psu_monitor_init();
}
