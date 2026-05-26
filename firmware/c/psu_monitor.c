#include "psu_monitor.h"
#include "picoemp.h"
#include "freq_measure.pio.h"

#include "hardware/pio.h"
#include "pico/stdlib.h"

// Per-window sample counts. Each raw sample is one full PWM period.
// Voltage uses a trimmed mean to reject EMI/glitch outliers (the PIO can
// occasionally push a near-zero period if a fast transient slips through
// after the rising-edge detection).
#define N_AVG_VOLTAGE  32u
#define N_TRIM_VOLTAGE  4u   // drop lowest 4 and highest 4 -> mean of middle 24
#define N_AVG_CURRENT   4u

// pio1 is used so there is no conflict with fast_trigger which uses pio0.
#define PSU_PIO  pio1
#define SM_VOLT  0u
#define SM_CURR  1u

static uint pio_offset;

// Voltage channel (SM_VOLT) — ring of raw period samples awaiting trim+mean.
static uint32_t voltage_buf[N_AVG_VOLTAGE];
static uint32_t voltage_buf_n        = 0;
static uint32_t voltage_period_ns    = 0;
static uint64_t voltage_last_samp_us = 0;

// Current channel (SM_CURR) — plain mean (window too small to trim usefully).
static uint64_t current_acc          = 0;
static uint32_t current_acc_n        = 0;
static uint32_t current_period_ns    = 0;
static uint64_t current_last_samp_us = 0;

void psu_monitor_init() {
    pio_offset = pio_add_program(PSU_PIO, &freq_measure_program);
    freq_measure_program_init(PSU_PIO, SM_VOLT, pio_offset, PIN_IN_Voltage_PWM);
    freq_measure_program_init(PSU_PIO, SM_CURR, pio_offset, PIN_IN_Current_PWM);
}

// Sort n uint32 in place (insertion sort — n is small, runs once per window).
static void sort_u32(uint32_t *a, uint32_t n) {
    for (uint32_t i = 1; i < n; i++) {
        uint32_t key = a[i];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
        a[j + 1] = key;
    }
}

// Trimmed mean: sort, discard `trim` from each end, average the rest.
static uint32_t trimmed_mean(uint32_t *samples, uint32_t n, uint32_t trim) {
    sort_u32(samples, n);
    uint64_t sum = 0;
    uint32_t kept = n - 2 * trim;
    for (uint32_t i = trim; i < n - trim; i++) sum += samples[i];
    return (uint32_t)(sum / kept);
}

void psu_monitor_update() {
    uint64_t now = time_us_64();

    // Each raw FIFO value = 0xFFFFFFFF - N_loops.  period_ns = 24 * N_loops
    // (3 PIO cycles per loop * 8 ns/cycle at 125 MHz sys_clk).
    while (!pio_sm_is_rx_fifo_empty(PSU_PIO, SM_VOLT)) {
        uint32_t loops = 0xFFFFFFFFu - pio_sm_get(PSU_PIO, SM_VOLT);
        voltage_buf[voltage_buf_n++] = 24u * loops;
        voltage_last_samp_us = now;
        if (voltage_buf_n >= N_AVG_VOLTAGE) {
            voltage_period_ns = trimmed_mean(voltage_buf, N_AVG_VOLTAGE, N_TRIM_VOLTAGE);
            voltage_buf_n = 0;
        }
    }

    while (!pio_sm_is_rx_fifo_empty(PSU_PIO, SM_CURR)) {
        current_acc += 0xFFFFFFFFu - pio_sm_get(PSU_PIO, SM_CURR);
        current_acc_n++;
        current_last_samp_us = now;
        if (current_acc_n >= N_AVG_CURRENT) {
            current_period_ns = (uint32_t)((24ULL * current_acc) / current_acc_n);
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
