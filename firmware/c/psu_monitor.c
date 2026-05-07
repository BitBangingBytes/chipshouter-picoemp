#include "psu_monitor.h"
#include "picoemp.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

// All volatile uint32_t — single-word reads/writes are atomic on Cortex-M0+.
// Timestamps are the lower 32 bits of time_us_64(), giving ~71 minutes of
// rollover range — far beyond either stale threshold.

static volatile bool     voltage_has_edge  = false;
static volatile uint32_t voltage_last_us   = 0;
static volatile uint32_t voltage_period_us = 0;

static volatile bool     current_has_edge  = false;
static volatile uint32_t current_last_us   = 0;
static volatile uint32_t current_period_us = 0;

static void gpio_irq_callback(uint gpio, uint32_t events) {
    uint32_t now = (uint32_t)time_us_64();

    if (gpio == PIN_IN_Voltage_PWM) {
        if (voltage_has_edge)
            voltage_period_us = now - voltage_last_us;
        voltage_last_us  = now;
        voltage_has_edge = true;
    } else if (gpio == PIN_IN_Current_PWM) {
        if (current_has_edge)
            current_period_us = now - current_last_us;
        current_last_us  = now;
        current_has_edge = true;
    }
}

void psu_monitor_init() {
    // First call sets the shared GPIO IRQ callback; second call reuses it.
    gpio_set_irq_enabled_with_callback(PIN_IN_Voltage_PWM, GPIO_IRQ_EDGE_RISE, true, gpio_irq_callback);
    gpio_set_irq_enabled(PIN_IN_Current_PWM, GPIO_IRQ_EDGE_RISE, true);
}

uint32_t psu_monitor_get_voltage_period_us() {
    return voltage_period_us;
}

uint32_t psu_monitor_get_current_period_us() {
    return current_period_us;
}

bool psu_monitor_voltage_is_valid() {
    if (!voltage_has_edge || voltage_period_us == 0)
        return false;
    return ((uint32_t)time_us_64() - voltage_last_us) < VOLTAGE_STALE_THRESHOLD_US;
}

bool psu_monitor_current_is_valid() {
    if (!current_has_edge || current_period_us == 0)
        return false;
    return ((uint32_t)time_us_64() - current_last_us) < CURRENT_STALE_THRESHOLD_US;
}
