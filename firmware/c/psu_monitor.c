#include "psu_monitor.h"
#include "picoemp.h"
#include "freq_measure.pio.h"

#include "hardware/pio.h"
#include "pico/stdlib.h"

// Averaging window per channel.
// Voltage: 16 samples — at 37 kHz collects in ~0.43 ms; at 250 Hz in ~64 ms.
// Current:  4 samples — at 3 Hz collects in ~1.3 s (one reading per ~4 periods).
#define N_AVG_VOLTAGE 16u
#define N_AVG_CURRENT  4u

// pio1 is used so there is no conflict with fast_trigger which uses pio0.
#define PSU_PIO  pio1
#define SM_VOLT  0u
#define SM_CURR  1u

static uint pio_offset;

// Voltage accumulator (SM_VOLT)
static uint64_t voltage_acc          = 0;
static uint32_t voltage_acc_n        = 0;
static uint32_t voltage_period_ns    = 0;
static uint64_t voltage_last_samp_us = 0;

// Current accumulator (SM_CURR)
static uint64_t current_acc          = 0;
static uint32_t current_acc_n        = 0;
static uint32_t current_period_ns    = 0;
static uint64_t current_last_samp_us = 0;

void psu_monitor_init() {
    pio_offset = pio_add_program(PSU_PIO, &freq_measure_program);
    freq_measure_program_init(PSU_PIO, SM_VOLT, pio_offset, PIN_IN_Voltage_PWM);
    freq_measure_program_init(PSU_PIO, SM_CURR, pio_offset, PIN_IN_Current_PWM);
}

// Each raw FIFO value = 0xFFFFFFFF - N_loops.
// period_ns = 32 * N_loops  (4 cycles per loop * 8 ns/cycle, doubled for 50% duty).
// Computed from accumulated sum to preserve sub-loop precision across averages.
static uint32_t acc_to_period_ns(uint64_t acc, uint32_t n) {
    return (uint32_t)((24ULL * acc) / n);
}

void psu_monitor_update() {
    uint64_t now = time_us_64();

    while (!pio_sm_is_rx_fifo_empty(PSU_PIO, SM_VOLT)) {
        voltage_acc         += 0xFFFFFFFFu - pio_sm_get(PSU_PIO, SM_VOLT);
        voltage_acc_n++;
        voltage_last_samp_us = now;
        if (voltage_acc_n >= N_AVG_VOLTAGE) {
            voltage_period_ns = acc_to_period_ns(voltage_acc, voltage_acc_n);
            voltage_acc   = 0;
            voltage_acc_n = 0;
        }
    }

    while (!pio_sm_is_rx_fifo_empty(PSU_PIO, SM_CURR)) {
        current_acc         += 0xFFFFFFFFu - pio_sm_get(PSU_PIO, SM_CURR);
        current_acc_n++;
        current_last_samp_us = now;
        if (current_acc_n >= N_AVG_CURRENT) {
            current_period_ns = acc_to_period_ns(current_acc, current_acc_n);
            current_acc   = 0;
            current_acc_n = 0;
        }
    }
}

uint32_t psu_monitor_get_voltage_period_ns() {
    return voltage_period_ns;
}

uint32_t psu_monitor_get_current_period_ns() {
    return current_period_ns;
}

bool psu_monitor_voltage_is_valid() {
    if (voltage_period_ns == 0) return false;
    return (time_us_64() - voltage_last_samp_us) < VOLTAGE_STALE_THRESHOLD_US;
}

bool psu_monitor_current_is_valid() {
    if (current_period_ns == 0) return false;
    return (time_us_64() - current_last_samp_us) < CURRENT_STALE_THRESHOLD_US;
}
