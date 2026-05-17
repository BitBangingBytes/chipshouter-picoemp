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
#define CAL_VERSION  1u

typedef struct {
    uint32_t     magic;
    uint32_t     version;
    uint32_t     n_points;
    uint32_t     reserved;
    cal_point_t  points[CAL_MAX_POINTS];
} cal_blob_t;

_Static_assert(sizeof(cal_blob_t) <= FLASH_SECTOR_SIZE,
               "cal_blob_t must fit in one flash sector");

// ---------------------------------------------------------------------------
// Compiled defaults — first PSU's bench values; used when flash is blank or
// the user requests a reset.
// ---------------------------------------------------------------------------

static const cal_point_t DEFAULT_POINTS[] = {
    {    0.0f,      0.0f },
    {   20.0f,    250.0f },
    {  100.0f,   1160.0f },
    {  500.0f,   6210.0f },
    { 1000.0f,  12450.0f },
    { 1500.0f,  18657.0f },
};
#define DEFAULT_POINTS_N  (sizeof(DEFAULT_POINTS) / sizeof(DEFAULT_POINTS[0]))

// ---------------------------------------------------------------------------
// Live table in RAM
// ---------------------------------------------------------------------------

static cal_point_t s_points[CAL_MAX_POINTS];
static uint        s_n_points = 0;
static cal_src_t   s_source   = CAL_SRC_DEFAULTS;

// Voltages within this tolerance are treated as the same point (for update vs insert).
#define VOLTS_EPSILON  0.05f

// ---------------------------------------------------------------------------

void cal_reset_to_defaults(void) {
    s_n_points = DEFAULT_POINTS_N;
    memcpy(s_points, DEFAULT_POINTS, sizeof(DEFAULT_POINTS));
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

uint cal_num_points(void) { return s_n_points; }

bool cal_get_point(uint i, cal_point_t *out) {
    if (i >= s_n_points || out == NULL) return false;
    *out = s_points[i];
    return true;
}

bool cal_capture(float volts) {
    if (volts < 0.0f) return false;
    if (!psu_monitor_voltage_is_valid()) return false;
    uint32_t period_ns = psu_monitor_get_voltage_period_ns();
    if (period_ns == 0) return false;
    float hz = 1000000000.0f / (float)period_ns;

    // Update existing entry within ε volts.
    for (uint i = 0; i < s_n_points; i++) {
        float d = s_points[i].volts - volts;
        if (d < 0) d = -d;
        if (d <= VOLTS_EPSILON) {
            s_points[i].hz = hz;
            s_source = CAL_SRC_RUNTIME;
            return true;
        }
    }

    if (s_n_points >= CAL_MAX_POINTS) return false;

    // Sorted insert by volts.
    uint pos = 0;
    while (pos < s_n_points && s_points[pos].volts < volts) pos++;
    for (uint i = s_n_points; i > pos; i--) s_points[i] = s_points[i-1];
    s_points[pos].volts = volts;
    s_points[pos].hz    = hz;
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

// ---------------------------------------------------------------------------
// Flash save (Core 0)
// ---------------------------------------------------------------------------
//
// We deliberately do NOT use flash_safe_execute() here. That API requires
// flash_safe_execute_core_init() to be called on the other (non-flashing) core,
// which installs an SIO IRQ handler that drains the inter-core FIFO looking
// for a lockout sentinel — and silently discards any other words. That breaks
// the normal Core 1 → Core 0 command/ack handshake used by every serial
// command. Instead we disable interrupts on Core 0 and run the flash ops from
// a function placed in RAM (__not_in_flash_func) so XIP being torn down
// mid-operation doesn't stall the calling function. Core 1 may briefly stall
// if it tries to fetch a flash instruction during the ~tens of ms erase+program
// window, but it recovers automatically when XIP is re-enabled.

static cal_blob_t s_pending_blob;  // built before flash op, lives in BSS

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
    s_pending_blob.magic    = CAL_MAGIC;
    s_pending_blob.version  = CAL_VERSION;
    s_pending_blob.n_points = s_n_points;
    memcpy(s_pending_blob.points, s_points, s_n_points * sizeof(cal_point_t));

    cal_do_flash_write();

    s_source = CAL_SRC_FLASH;
    return true;
}
