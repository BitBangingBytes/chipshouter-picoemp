#include "psu_monitor.h"
#include "picoemp.h"
#include "freq_measure.pio.h"

#include "hardware/pio.h"
#include "pico/stdlib.h"

// Averaging window per channel. PIO measures every period (no skip), so the
// collection time is N_AVG * PWM_period.
// Voltage: 32 samples — at 18 kHz collects in ~1.7 ms; at 250 Hz in ~128 ms.
// Current:  4 samples — at 3 Hz collects in ~1.3 s.
#define N_AVG_VOLTAGE 32u
#define N_AVG_CURRENT  4u

// Median filter over the last N published 32-sample voltage averages.
// Rejects up to (N-1)/2 consecutive bad windows without distorting a steady signal.
#define VOLTAGE_MEDIAN_N 5u

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
static uint32_t voltage_sample_seq   = 0;

// Median filter state for voltage
static uint32_t voltage_median_buf[VOLTAGE_MEDIAN_N];
static uint32_t voltage_median_head  = 0;
static uint32_t voltage_median_count = 0;

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
// period_ns = 24 * N_loops  (3 PIO cycles per loop * 8 ns/cycle at 125 MHz sys_clk;
//                            measures full rise-to-rise period, no duty assumption).
// Computed from accumulated sum to preserve sub-loop precision across averages.
static uint32_t acc_to_period_ns(uint64_t acc, uint32_t n) {
    return (uint32_t)((24ULL * acc) / n);
}

static void voltage_median_push(uint32_t period_ns) {
    voltage_median_buf[voltage_median_head] = period_ns;
    voltage_median_head = (voltage_median_head + 1) % VOLTAGE_MEDIAN_N;
    if (voltage_median_count < VOLTAGE_MEDIAN_N) voltage_median_count++;
}

static uint32_t voltage_median_get(void) {
    if (voltage_median_count == 0) return 0;
    uint32_t tmp[VOLTAGE_MEDIAN_N];
    for (uint32_t i = 0; i < voltage_median_count; i++) tmp[i] = voltage_median_buf[i];
    // insertion sort — cheap for N=5
    for (uint32_t i = 1; i < voltage_median_count; i++) {
        uint32_t key = tmp[i];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = key;
    }
    return tmp[voltage_median_count / 2];
}

void psu_monitor_update() {
    uint64_t now = time_us_64();

    while (!pio_sm_is_rx_fifo_empty(PSU_PIO, SM_VOLT)) {
        voltage_acc         += 0xFFFFFFFFu - pio_sm_get(PSU_PIO, SM_VOLT);
        voltage_acc_n++;
        voltage_last_samp_us = now;
        if (voltage_acc_n >= N_AVG_VOLTAGE) {
            voltage_period_ns = acc_to_period_ns(voltage_acc, voltage_acc_n);
            voltage_median_push(voltage_period_ns);
            voltage_acc   = 0;
            voltage_acc_n = 0;
            voltage_sample_seq++;
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
    return voltage_median_get();
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

uint32_t psu_monitor_voltage_sample_seq() {
    return voltage_sample_seq;
}
