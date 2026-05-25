#include "cal.h"
#include "psu_monitor.h"

#include "hardware/flash.h"
#include "hardware/sync.h"

#include <string.h>

// ---------------------------------------------------------------------------
// Flash storage
// ---------------------------------------------------------------------------

#define CAL_FLASH_OFFSET  (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define CAL_FLASH_ADDR    ((const uint8_t *)(XIP_BASE + CAL_FLASH_OFFSET))

#define CAL_MAGIC    0x4C414350u   // 'PCAL'
#define CAL_VERSION  3u

typedef struct {
    uint32_t     magic;
    uint32_t     version;
    uint32_t     n_points;
    uint32_t     reserved;
    cal_point_t  points[CAL_MAX_POINTS];
    // v2: ramp configuration
    uint16_t     ramp_step_max;
    uint16_t     ramp_step_min;
    uint32_t     ramp_tick_ms;
    // v3: soft and hard voltage limits
    float        soft_limit_volts;
    float        hard_limit_volts;
} cal_blob_t;

_Static_assert(sizeof(cal_blob_t) <= FLASH_SECTOR_SIZE,
               "cal_blob_t must fit in one flash sector");

// ---------------------------------------------------------------------------
// Compiled defaults
// ---------------------------------------------------------------------------

static const cal_point_t DEFAULT_POINTS[] = {
    {    0.0f,      0.0f, 10, 0 },
    {   20.0f,    250.0f, 10, 0 },
    {  100.0f,   1160.0f, 10, 0 },
    {  500.0f,   6210.0f, 10, 0 },
    { 1000.0f,  12450.0f, 10, 0 },
    { 1500.0f,  18657.0f, 10, 0 },
};
#define DEFAULT_POINTS_N  (sizeof(DEFAULT_POINTS) / sizeof(DEFAULT_POINTS[0]))

#define DEFAULT_RAMP_STEP_MAX    100u
#define DEFAULT_RAMP_STEP_MIN      1u
#define DEFAULT_RAMP_TICK_MS      20u
#define DEFAULT_SOFT_LIMIT_VOLTS  1500.0f
#define DEFAULT_HARD_LIMIT_VOLTS  1500.0f
#define ABSOLUTE_MAX_VOLTS        1500.0f

// ---------------------------------------------------------------------------
// Live state in RAM
// ---------------------------------------------------------------------------

static cal_point_t s_points[CAL_MAX_POINTS];
static uint        s_n_points = 0;
static cal_src_t   s_source   = CAL_SRC_DEFAULTS;

static uint16_t s_ramp_step_max   = DEFAULT_RAMP_STEP_MAX;
static uint16_t s_ramp_step_min   = DEFAULT_RAMP_STEP_MIN;
static uint32_t s_ramp_tick_ms    = DEFAULT_RAMP_TICK_MS;
static float    s_soft_limit_volts = DEFAULT_SOFT_LIMIT_VOLTS;
static float    s_hard_limit_volts = DEFAULT_HARD_LIMIT_VOLTS;

#define VOLTS_EPSILON  0.05f

// ---------------------------------------------------------------------------

void cal_reset_to_defaults(void) {
    s_n_points = DEFAULT_POINTS_N;
    memcpy(s_points, DEFAULT_POINTS, sizeof(DEFAULT_POINTS));
    s_ramp_step_max    = DEFAULT_RAMP_STEP_MAX;
    s_ramp_step_min    = DEFAULT_RAMP_STEP_MIN;
    s_ramp_tick_ms     = DEFAULT_RAMP_TICK_MS;
    s_soft_limit_volts = DEFAULT_SOFT_LIMIT_VOLTS;
    s_hard_limit_volts = DEFAULT_HARD_LIMIT_VOLTS;
    s_source = CAL_SRC_DEFAULTS;
}

void cal_init(void) {
    const cal_blob_t *blob = (const cal_blob_t *)CAL_FLASH_ADDR;
    if (blob->magic == CAL_MAGIC
        && blob->version == CAL_VERSION
        && blob->n_points >= 2
        && blob->n_points <= CAL_MAX_POINTS) {
        s_n_points = blob->n_points;
        memcpy(s_points, blob->points, s_n_points * sizeof(cal_point_t));
        s_ramp_step_max    = blob->ramp_step_max;
        s_ramp_step_min    = blob->ramp_step_min;
        s_ramp_tick_ms     = blob->ramp_tick_ms;
        s_soft_limit_volts = blob->soft_limit_volts;
        s_hard_limit_volts = blob->hard_limit_volts;
        s_source = CAL_SRC_FLASH;
    } else {
        cal_reset_to_defaults();
    }
}

float cal_period_ns_to_volts(uint32_t period_ns) {
    if (period_ns == 0 || s_n_points < 2) return 0.0f;
    float hz = 1000000000.0f / (float)period_ns;
    if (hz <= s_points[0].hz)              return s_points[0].volts;
    if (hz >= s_points[s_n_points-1].hz)   return s_points[s_n_points-1].volts;
    for (uint i = 1; i < s_n_points; i++) {
        if (hz <= s_points[i].hz) {
            float t = (hz - s_points[i-1].hz) / (s_points[i].hz - s_points[i-1].hz);
            return s_points[i-1].volts + t * (s_points[i].volts - s_points[i-1].volts);
        }
    }
    return s_points[s_n_points-1].volts;
}

uint16_t cal_volts_to_dac(float volts) {
    if (s_n_points < 2) return 10;
    if (volts <= s_points[0].volts) return s_points[0].dac_code;
    if (volts >= s_points[s_n_points-1].volts) return s_points[s_n_points-1].dac_code;
    for (uint i = 1; i < s_n_points; i++) {
        if (volts <= s_points[i].volts) {
            float t = (volts - s_points[i-1].volts)
                    / (s_points[i].volts - s_points[i-1].volts);
            float dac_f = (float)s_points[i-1].dac_code
                        + t * ((float)s_points[i].dac_code - (float)s_points[i-1].dac_code);
            return (uint16_t)(dac_f + 0.5f);
        }
    }
    return s_points[s_n_points-1].dac_code;
}

uint cal_num_points(void) { return s_n_points; }

bool cal_get_point(uint i, cal_point_t *out) {
    if (i >= s_n_points || out == NULL) return false;
    *out = s_points[i];
    return true;
}

bool cal_capture(float volts, uint16_t dac_code) {
    if (volts < 0.0f) return false;
    if (!psu_monitor_voltage_is_valid()) return false;
    uint32_t period_ns = psu_monitor_get_voltage_period_ns();
    if (period_ns == 0) return false;
    float hz = 1000000000.0f / (float)period_ns;

    for (uint i = 0; i < s_n_points; i++) {
        float d = s_points[i].volts - volts;
        if (d < 0) d = -d;
        if (d <= VOLTS_EPSILON) {
            s_points[i].hz       = hz;
            s_points[i].dac_code = dac_code;
            s_source = CAL_SRC_RUNTIME;
            return true;
        }
    }

    if (s_n_points >= CAL_MAX_POINTS) return false;

    uint pos = 0;
    while (pos < s_n_points && s_points[pos].volts < volts) pos++;
    for (uint i = s_n_points; i > pos; i--) s_points[i] = s_points[i-1];
    s_points[pos].volts    = volts;
    s_points[pos].hz       = hz;
    s_points[pos].dac_code = dac_code;
    s_points[pos]._pad     = 0;
    s_n_points++;
    s_source = CAL_SRC_RUNTIME;
    return true;
}

bool cal_remove(uint i) {
    if (i >= s_n_points) return false;
    for (uint k = i; k + 1 < s_n_points; k++) s_points[k] = s_points[k+1];
    s_n_points--;
    s_source = CAL_SRC_RUNTIME;
    return true;
}

cal_src_t cal_source(void) { return s_source; }

void cal_get_ramp(uint16_t *step_max, uint16_t *step_min, uint32_t *tick_ms) {
    *step_max = s_ramp_step_max;
    *step_min = s_ramp_step_min;
    *tick_ms  = s_ramp_tick_ms;
}

void cal_set_ramp(uint16_t step_max, uint16_t step_min, uint32_t tick_ms) {
    s_ramp_step_max = step_max;
    s_ramp_step_min = step_min;
    s_ramp_tick_ms  = tick_ms;
    s_source = CAL_SRC_RUNTIME;
}

float cal_get_soft_limit(void) { return s_soft_limit_volts; }

void cal_set_soft_limit(float volts) {
    if (volts < 0.0f)               volts = 0.0f;
    if (volts > s_hard_limit_volts) volts = s_hard_limit_volts;
    s_soft_limit_volts = volts;
    s_source = CAL_SRC_RUNTIME;
}

float cal_get_hard_limit(void) { return s_hard_limit_volts; }

void cal_set_hard_limit(float volts) {
    if (volts < 0.0f)              volts = 0.0f;
    if (volts > ABSOLUTE_MAX_VOLTS) volts = ABSOLUTE_MAX_VOLTS;
    s_hard_limit_volts = volts;
    // Clamp soft limit if it now exceeds hard limit.
    if (s_soft_limit_volts > s_hard_limit_volts) s_soft_limit_volts = s_hard_limit_volts;
    s_source = CAL_SRC_RUNTIME;
}

// ---------------------------------------------------------------------------
// Flash save (Core 0)
// ---------------------------------------------------------------------------

static cal_blob_t s_pending_blob;

static void __not_in_flash_func(cal_do_flash_write)(void) {
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(CAL_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CAL_FLASH_OFFSET,
                        (const uint8_t *)&s_pending_blob,
                        sizeof(s_pending_blob));
    restore_interrupts(ints);
}

bool cal_save(void) {
    memset(&s_pending_blob, 0, sizeof(s_pending_blob));
    s_pending_blob.magic         = CAL_MAGIC;
    s_pending_blob.version       = CAL_VERSION;
    s_pending_blob.n_points      = s_n_points;
    s_pending_blob.ramp_step_max    = s_ramp_step_max;
    s_pending_blob.ramp_step_min    = s_ramp_step_min;
    s_pending_blob.ramp_tick_ms     = s_ramp_tick_ms;
    s_pending_blob.soft_limit_volts = s_soft_limit_volts;
    s_pending_blob.hard_limit_volts = s_hard_limit_volts;
    memcpy(s_pending_blob.points, s_points, s_n_points * sizeof(cal_point_t));

    cal_do_flash_write();

    s_source = CAL_SRC_FLASH;
    return true;
}
