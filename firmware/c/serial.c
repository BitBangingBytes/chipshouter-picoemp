#include "serial.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"

#include "control_loop.h"

static char serial_buffer[256];
static char last_command[256];

static union float_union {float f; uint32_t ui32;} float_xfer;

void read_line() {
    memset(serial_buffer, 0, sizeof(serial_buffer));
    size_t len = 0;
    while (1) {
        int c = getchar();
        if (c == EOF) return;
        if (c == '\r') { putchar('\r'); putchar('\n'); return; }
        if (c == '\n') continue;
        if (c == 0x08 || c == 0x7F) {
            if (len > 0) {
                len--;
                serial_buffer[len] = '\0';
                putchar(0x08); putchar(' '); putchar(0x08);
            }
            continue;
        }
        if (len >= 255) return;
        serial_buffer[len++] = (char)c;
        putchar(c);
    }
}

bool handle_command(char *command) {
    if (command[0] == 0 && last_command[0] != 0) {
        printf("Repeat previous command (%s)\n", last_command);
        return handle_command(last_command);
    } else {
        strcpy(last_command, command);
    }

    if(strcmp(command, "h") == 0 || strcmp(command, "help") == 0)
        return false;

    if(strcmp(command, "s") == 0 || strcmp(command, "status") == 0) {
        // Gather all state via sequential FIFO round-trips
        uint32_t hv_en = 0, dac_code = 0, manual = 0, refresh_ms = 0;
        uint32_t smax = 0, smin = 0, tms = 0, faults = 0, fi = 0;
        float target_v = 0.0f, actual_v = 0.0f, soft_lim = 0.0f;

        multicore_fifo_push_blocking(cmd_get_hv_enabled);
        if (multicore_fifo_pop_blocking() == return_ok) hv_en = multicore_fifo_pop_blocking();

        multicore_fifo_push_blocking(cmd_get_target_voltage);
        if (multicore_fifo_pop_blocking() == return_ok) {
            float_xfer.ui32 = multicore_fifo_pop_blocking(); target_v = float_xfer.f;
        }

        multicore_fifo_push_blocking(cmd_get_actual_voltage);
        if (multicore_fifo_pop_blocking() == return_ok) {
            float_xfer.ui32 = multicore_fifo_pop_blocking(); actual_v = float_xfer.f;
        }

        multicore_fifo_push_blocking(cmd_get_dac);
        if (multicore_fifo_pop_blocking() == return_ok) dac_code = multicore_fifo_pop_blocking();

        multicore_fifo_push_blocking(cmd_get_manual_mode);
        if (multicore_fifo_pop_blocking() == return_ok) manual = multicore_fifo_pop_blocking();

        multicore_fifo_push_blocking(cmd_get_soft_limit);
        if (multicore_fifo_pop_blocking() == return_ok) {
            float_xfer.ui32 = multicore_fifo_pop_blocking(); soft_lim = float_xfer.f;
        }

        multicore_fifo_push_blocking(cmd_get_dac_refresh);
        if (multicore_fifo_pop_blocking() == return_ok) refresh_ms = multicore_fifo_pop_blocking();

        multicore_fifo_push_blocking(cmd_get_ramp);
        if (multicore_fifo_pop_blocking() == return_ok) {
            smax = multicore_fifo_pop_blocking();
            smin = multicore_fifo_pop_blocking();
            tms  = multicore_fifo_pop_blocking();
        }

        multicore_fifo_push_blocking(cmd_get_faults);
        if (multicore_fifo_pop_blocking() == return_ok) faults = multicore_fifo_pop_blocking();

        multicore_fifo_push_blocking(cmd_get_fault_ignore);
        if (multicore_fifo_pop_blocking() == return_ok) fi = multicore_fifo_pop_blocking();

        printf("BIO-RAD PSU Status:\n");
        printf("  HV output:       %s\n", hv_en ? "enabled" : "disabled");
        printf("  Target voltage:  %8.2f V\n", (double)target_v);
        printf("  Actual voltage:  %8.2f V  (PWM cal table)\n", (double)actual_v);
        printf("  Current DAC:     %u  (0x%03x)\n", (unsigned)dac_code, (unsigned)dac_code);
        printf("  Manual DAC mode: %s\n", manual ? "yes" : "no");
        printf("  Soft limit:      %8.2f V\n", (double)soft_lim);
        printf("  Hard limit:      %8.2f V  (fixed)\n", (double)HARD_LIMIT_VOLTS);
        if (refresh_ms == 0)
            printf("  DAC refresh:     disabled\n");
        else
            printf("  DAC refresh:     %u ms\n", (unsigned)refresh_ms);
        printf("  Ramp:            step_max=%u  step_min=%u  tick_ms=%u\n",
               (unsigned)smax, (unsigned)smin, (unsigned)tms);
        if (faults == 0) {
            printf("  Faults:          none\n");
        } else {
            printf("  Faults:          0x%02x", (unsigned)faults);
            if (faults & 0x01) printf(" [circuit-open]");
            if (faults & 0x02) printf(" [circuit-shorted]");
            if (faults & 0x04) printf(" [circuit-unknown]");
            if (faults & 0x08) printf(" [overvoltage]");
            if (faults & 0x10) printf(" [overcurrent]");
            printf("\n");
        }
        printf("  Fault ignore:    %s\n", fi ? "on" : "off");
        return true;
    }

    if(strcmp(command, "rv") == 0 || strcmp(command, "read_voltage") == 0) {
        multicore_fifo_push_blocking(cmd_read_voltage_pwm);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok) {
            uint32_t period_ns = multicore_fifo_pop_blocking();
            float_xfer.ui32 = multicore_fifo_pop_blocking();
            float volts = float_xfer.f;
            if(period_ns == 0) {
                printf("Voltage PWM: No signal\n");
            } else {
                float hz = 1000000000.0f / (float)period_ns;
                printf("Voltage PWM: period=%uns  freq=%.2fHz  %.1fV (cal table)\n",
                       period_ns, hz, volts);
            }
        } else {
            printf("Read voltage failed!\n");
        }
        return true;
    }

    if(strcmp(command, "ri") == 0 || strcmp(command, "read_current") == 0) {
        multicore_fifo_push_blocking(cmd_read_current_pwm);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok) {
            uint32_t period_ns = multicore_fifo_pop_blocking();
            if(period_ns == 0) {
                printf("Current PWM: No signal\n");
            } else {
                float hz = 1000000000.0f / (float)period_ns;
                printf("Current PWM: period=%uns  freq=%.2fHz  [calibration TODO]\n",
                       period_ns, hz);
            }
        } else {
            printf("Read current failed!\n");
        }
        return true;
    }

    if(strcmp(command, "hve") == 0 || strcmp(command, "hv_enable") == 0) {
        multicore_fifo_push_blocking(cmd_hv_enable);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok)
            printf("HV output enabled.\n");
        else
            printf("HV enable failed!\n");
        return true;
    }

    if(strcmp(command, "hvd") == 0 || strcmp(command, "hv_disable") == 0) {
        multicore_fifo_push_blocking(cmd_hv_disable);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok)
            printf("HV output disabled.\n");
        else
            printf("HV disable failed!\n");
        return true;
    }

    if(strcmp(command, "sv") == 0 || strcmp(command, "set_voltage") == 0) {
        char **unused;
        printf(" target voltage in volts (0 - 1500)?\n> ");
        read_line();
        if (serial_buffer[0] == 0) {
            printf("Cancelled.\n");
            return true;
        }
        float_xfer.f = strtof(serial_buffer, unused);
        multicore_fifo_push_blocking(cmd_set_voltage);
        multicore_fifo_push_blocking(float_xfer.ui32);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok)
            printf("Target voltage set to %.1f V\n", float_xfer.f);
        else
            printf("Set voltage failed!\n");
        return true;
    }

    if(strcmp(command, "sl") == 0 || strcmp(command, "set_limit") == 0) {
        char **unused;
        printf(" soft voltage limit in volts (0 - 1500)?\n> ");
        read_line();
        if (serial_buffer[0] == 0) {
            printf("Cancelled.\n");
            return true;
        }
        float_xfer.f = strtof(serial_buffer, unused);
        multicore_fifo_push_blocking(cmd_set_soft_limit);
        multicore_fifo_push_blocking(float_xfer.ui32);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok)
            printf("Soft limit set to %.1f V\n", float_xfer.f);
        else
            printf("Set soft limit failed!\n");
        return true;
    }

    if(strcmp(command, "av") == 0 || strcmp(command, "actual_voltage") == 0) {
        multicore_fifo_push_blocking(cmd_get_actual_voltage);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok) {
            float_xfer.ui32 = multicore_fifo_pop_blocking();
            printf("Actual voltage: %.1f V\n", float_xfer.f);
        } else {
            printf("Read actual voltage failed!\n");
        }
        return true;
    }

    if(strcmp(command, "ai") == 0 || strcmp(command, "actual_current") == 0) {
        multicore_fifo_push_blocking(cmd_get_actual_current);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok) {
            float_xfer.ui32 = multicore_fifo_pop_blocking();
            printf("Actual current feedback: %.2f Hz (uncalibrated)\n", float_xfer.f);
        } else {
            printf("Read actual current failed!\n");
        }
        return true;
    }

    if(strcmp(command, "gf") == 0 || strcmp(command, "get_faults") == 0) {
        multicore_fifo_push_blocking(cmd_get_faults);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok) {
            uint32_t faults = multicore_fifo_pop_blocking();
            if(faults == 0) {
                printf("No faults.\n");
            } else {
                printf("Faults (0x%02x):\n", faults);
                if(faults & 0x01) printf("  - Circuit open\n");
                if(faults & 0x02) printf("  - Circuit shorted\n");
                if(faults & 0x04) printf("  - Circuit unknown\n");
                if(faults & 0x08) printf("  - Overvoltage\n");
                if(faults & 0x10) printf("  - Overcurrent\n");
            }
        } else {
            printf("Get faults failed!\n");
        }
        return true;
    }

    if(strcmp(command, "cf") == 0 || strcmp(command, "clear_faults") == 0) {
        multicore_fifo_push_blocking(cmd_clear_faults);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok)
            printf("Faults cleared.\n");
        else
            printf("Clear faults failed!\n");
        return true;
    }

    if(strcmp(command, "fi") == 0 || strcmp(command, "fault_ignore") == 0) {
        multicore_fifo_push_blocking(cmd_set_fault_ignore);
        multicore_fifo_push_blocking(1u);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok)
            printf("Fault ignore enabled. Faults will be recorded but will not shut down.\n");
        else
            printf("Failed!\n");
        return true;
    }

    if(strcmp(command, "fn") == 0 || strcmp(command, "fault_normal") == 0) {
        multicore_fifo_push_blocking(cmd_set_fault_ignore);
        multicore_fifo_push_blocking(0u);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok)
            printf("Fault ignore disabled. Faults will trigger shutdown.\n");
        else
            printf("Failed!\n");
        return true;
    }

    if(strcmp(command, "dacw") == 0 || strcmp(command, "dac_write") == 0) {
        char **unused;
        printf(" raw DAC code (0-4095, hex with 0x or decimal)?\n> ");
        read_line();
        if (serial_buffer[0] == 0) {
            printf("Cancelled.\n");
            return true;
        }
        unsigned long val = strtoul(serial_buffer, unused, 0);
        if (val > 4095) {
            printf("Value out of range (0-4095).\n");
            return true;
        }
        multicore_fifo_push_blocking(cmd_debug_dac_raw);
        multicore_fifo_push_blocking((uint32_t)val);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok) {
            printf("Wrote raw DAC code %lu (0x%03lx, 0b", val, val);
            for (int b = 11; b >= 0; b--) putchar(((val >> b) & 1u) ? '1' : '0');
            printf(")\n");
        } else {
            printf("Raw DAC write failed!\n");
        }
        return true;
    }

    if(strcmp(command, "dacr") == 0 || strcmp(command, "dac_read") == 0) {
        multicore_fifo_push_blocking(cmd_get_dac);
        uint32_t result = multicore_fifo_pop_blocking();
        if (result == return_ok) {
            uint32_t code = multicore_fifo_pop_blocking();
            printf("Current DAC code: %u (0x%03x)\n", (unsigned)code, (unsigned)code);
        } else {
            printf("Read DAC failed!\n");
        }
        return true;
    }

    if(strcmp(command, "sdr") == 0 || strcmp(command, "set_dac_refresh") == 0) {
        char **unused;
        printf(" DAC refresh interval in ms (0 = disable, default: 500)?\n> ");
        read_line();
        if (serial_buffer[0] == 0) {
            printf("Cancelled.\n");
            return true;
        }
        uint32_t ms = (uint32_t)strtoul(serial_buffer, unused, 10);
        multicore_fifo_push_blocking(cmd_set_dac_refresh);
        multicore_fifo_push_blocking(ms);
        uint32_t result = multicore_fifo_pop_blocking();
        if (result == return_ok)
            printf("DAC refresh interval set to %u ms%s\n", ms, ms == 0 ? " (disabled)" : "");
        else
            printf("Set DAC refresh failed!\n");
        return true;
    }

    if(strcmp(command, "gdr") == 0 || strcmp(command, "get_dac_refresh") == 0) {
        multicore_fifo_push_blocking(cmd_get_dac_refresh);
        uint32_t result = multicore_fifo_pop_blocking();
        if (result == return_ok) {
            uint32_t ms = multicore_fifo_pop_blocking();
            if (ms == 0)
                printf("DAC refresh: disabled\n");
            else
                printf("DAC refresh: %u ms\n", ms);
        } else {
            printf("Get DAC refresh failed!\n");
        }
        return true;
    }

    if(strcmp(command, "sr") == 0 || strcmp(command, "set_ramp") == 0) {
        char **unused;
        printf(" Ramp step_max (DAC codes, largest step at start)?\n> ");
        read_line();
        if (serial_buffer[0] == 0) { printf("Cancelled.\n"); return true; }
        uint16_t smax = (uint16_t)strtoul(serial_buffer, unused, 10);

        printf(" Ramp step_min (DAC codes, smallest step near target)?\n> ");
        read_line();
        if (serial_buffer[0] == 0) { printf("Cancelled.\n"); return true; }
        uint16_t smin = (uint16_t)strtoul(serial_buffer, unused, 10);

        printf(" Ramp tick_ms (ms between steps)?\n> ");
        read_line();
        if (serial_buffer[0] == 0) { printf("Cancelled.\n"); return true; }
        uint32_t tms = (uint32_t)strtoul(serial_buffer, unused, 10);

        multicore_fifo_push_blocking(cmd_set_ramp);
        multicore_fifo_push_blocking((uint32_t)smax);
        multicore_fifo_push_blocking((uint32_t)smin);
        multicore_fifo_push_blocking(tms);
        uint32_t result = multicore_fifo_pop_blocking();
        if (result == return_ok)
            printf("Ramp set: step_max=%u  step_min=%u  tick_ms=%u  (use csv to persist)\n",
                   (unsigned)smax, (unsigned)smin, (unsigned)tms);
        else
            printf("Set ramp failed!\n");
        return true;
    }

    if(strcmp(command, "gr") == 0 || strcmp(command, "get_ramp") == 0) {
        multicore_fifo_push_blocking(cmd_get_ramp);
        uint32_t result = multicore_fifo_pop_blocking();
        if (result == return_ok) {
            uint32_t smax = multicore_fifo_pop_blocking();
            uint32_t smin = multicore_fifo_pop_blocking();
            uint32_t tms  = multicore_fifo_pop_blocking();
            printf("Ramp: step_max=%u  step_min=%u  tick_ms=%u\n",
                   (unsigned)smax, (unsigned)smin, (unsigned)tms);
        } else {
            printf("Get ramp failed!\n");
        }
        return true;
    }

    if(strcmp(command, "cap") == 0 || strcmp(command, "cal_capture") == 0) {
        char **unused;
        printf(" measured voltage at current PWM (V)?\n> ");
        read_line();
        if (serial_buffer[0] == 0) {
            printf("Cancelled.\n");
            return true;
        }
        float_xfer.f = strtof(serial_buffer, unused);
        multicore_fifo_push_blocking(cmd_cal_capture);
        multicore_fifo_push_blocking(float_xfer.ui32);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok)
            printf("Captured cal point at %.2f V.\n", float_xfer.f);
        else
            printf("Capture failed (no valid PWM signal, table full, or invalid value).\n");
        return true;
    }

    if(strcmp(command, "cls") == 0 || strcmp(command, "cal_list") == 0) {
        multicore_fifo_push_blocking(cmd_cal_list);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result != return_ok) {
            printf("Cal list failed!\n");
            return true;
        }
        uint32_t n = multicore_fifo_pop_blocking();
        printf("Cal points (%u):\n", (unsigned)n);
        printf("  %-3s | %-10s | %-6s | %s\n", "Idx", "Voltage", "DAC", "PWM Frequency");
        printf("  ----+------------+--------+--------------\n");
        for (uint32_t i = 0; i < n; i++) {
            float_xfer.ui32 = multicore_fifo_pop_blocking();
            float volts = float_xfer.f;
            float_xfer.ui32 = multicore_fifo_pop_blocking();
            float hz = float_xfer.f;
            uint32_t dac = multicore_fifo_pop_blocking();
            printf("  %3u | %8.2f V | %6u | %10.2f Hz\n", (unsigned)i, volts, (unsigned)dac, hz);
        }
        uint32_t src = multicore_fifo_pop_blocking();
        const char *src_str = (src == 0) ? "defaults" : (src == 1) ? "flash" : "runtime (unsaved)";
        printf("Source: %s\n", src_str);
        return true;
    }

    if(strcmp(command, "crm") == 0 || strcmp(command, "cal_remove") == 0) {
        char **unused;
        printf(" index to remove?\n> ");
        read_line();
        if (serial_buffer[0] == 0) {
            printf("Cancelled.\n");
            return true;
        }
        unsigned long idx = strtoul(serial_buffer, unused, 10);
        multicore_fifo_push_blocking(cmd_cal_remove);
        multicore_fifo_push_blocking((uint32_t)idx);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok)
            printf("Removed cal point %lu.\n", idx);
        else
            printf("Remove failed (index out of range).\n");
        return true;
    }

    if(strcmp(command, "csv") == 0 || strcmp(command, "cal_save") == 0) {
        multicore_fifo_push_blocking(cmd_cal_save);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok)
            printf("Cal saved to flash.\n");
        else
            printf("Cal save failed!\n");
        return true;
    }

    if(strcmp(command, "crd") == 0 || strcmp(command, "cal_default") == 0) {
        multicore_fifo_push_blocking(cmd_cal_reset);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok)
            printf("Cal reset to compiled defaults (use 'csv' to persist).\n");
        else
            printf("Cal reset failed!\n");
        return true;
    }

    if(strcmp(command, "r") == 0 || strcmp(command, "reset") == 0) {
        watchdog_enable(1, 1);
        while(1);
    }

    if(strcmp(command, "b") == 0 || strcmp(command, "bootload") == 0) {
        printf("Rebooting to firmware loading mode...\n");
        reset_usb_boot(0, 0);
    }

    return false;
}

void serial_console() {
    multicore_fifo_drain();
    memset(last_command, 0, sizeof(last_command));

    while(1) {
        read_line();
        if(!handle_command(serial_buffer)) {
            printf("BIO-RAD Commands:\n");
            printf("- <empty to repeat last command>\n");
            printf("- [h]elp\n");
            printf("- [s]tatus: show all configurable and measured state\n");
            printf("- [r]eset\n");
            printf("- [b]ootload: reboot into USB firmware loading mode\n");
            printf("- [rv] read_voltage: raw PWM period, Hz, and cal-table volts\n");
            printf("- [ri] read_current: raw PWM period, Hz\n");
            printf("- [hve] hv_enable: enable HV output / control loop\n");
            printf("- [hvd] hv_disable: disable HV output / control loop\n");
            printf("- [sv] set_voltage: set target voltage (V)\n");
            printf("- [sl] set_limit: set soft voltage limit (V)\n");
            printf("- [av] actual_voltage: read estimated actual voltage (PWM cal table)\n");
            printf("- [ai] actual_current: read current feedback (Hz, uncalibrated)\n");
            printf("- [gf] get_faults: show active fault flags\n");
            printf("- [cf] clear_faults: clear sticky fault register\n");
            printf("- [fi] fault_ignore: record faults but suppress shutdown\n");
            printf("- [fn] fault_normal: restore normal fault shutdown behavior\n");
            printf("- [dacw] dac_write: write raw 12-bit DAC code (0-4095); pauses ramp if HV is on\n");
            printf("- [dacr] dac_read: read current DAC code\n");
            printf("- [sdr] set_dac_refresh: set periodic DAC resend interval in ms (0 = disable)\n");
            printf("- [gdr] get_dac_refresh: show current DAC refresh interval\n");
            printf("- [sr] set_ramp: set parabolic ramp params (step_max, step_min, tick_ms)\n");
            printf("- [gr] get_ramp: show current ramp params\n");
            printf("- [cap] cal_capture: record cal point at user-measured volts with current DAC\n");
            printf("- [cls] cal_list: show voltage->DAC cal table and its source\n");
            printf("- [crm] cal_remove: remove a cal point by index\n");
            printf("- [csv] cal_save: persist current cal (including ramp params) to flash\n");
            printf("- [crd] cal_default: reset cal to compiled defaults (use csv to persist)\n");
        }
        printf("\n");

        if (last_command[0] != 0) {
            printf("[%s] > ", last_command);
        } else {
            printf(" > ");
        }
    }
}
