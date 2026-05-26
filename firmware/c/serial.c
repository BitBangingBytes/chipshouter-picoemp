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
static float s_soft_limit = 1500.0f;
static float s_hard_limit = 1500.0f;

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

// Returns pointer to the inline argument if command matches name (optionally followed
// by ' ' + argument). Returns "" if matched with no argument. Returns NULL if no match.
static const char *cmd_match(const char *command, const char *name) {
    size_t n = strlen(name);
    if (strncmp(command, name, n) != 0) return NULL;
    if (command[n] == '\0') return "";
    if (command[n] == ' ') return command + n + 1;
    return NULL;
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

    // -------------------------------------------------------------------------
    // Status
    // -------------------------------------------------------------------------

    if(strcmp(command, "s") == 0 || strcmp(command, "status") == 0) {
        uint32_t hv_en = 0, dac_code = 0, manual = 0;
        uint32_t smax = 0, smin = 0, tms = 0, faults = 0, fi = 0;
        float target_v = 0.0f, actual_v = 0.0f;

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
        printf("  HV output:       %s\n",       hv_en  ? "enabled"  : "disabled");
        printf("  Target voltage:  %8.2f V\n",  (double)target_v);
        printf("  Actual voltage:  %8.2f V\n",  (double)actual_v);
        printf("  Current DAC:     %u  (0x%03x)\n", (unsigned)dac_code, (unsigned)dac_code);
        printf("  Manual DAC mode: %s\n",        manual ? "yes" : "no");
        printf("  Soft limit:      %8.2f V\n",  (double)s_soft_limit);
        printf("  Hard limit:      %8.2f V\n",  (double)s_hard_limit);
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

    // -------------------------------------------------------------------------
    // HV output control
    // -------------------------------------------------------------------------

    if(strcmp(command, "hve") == 0 || strcmp(command, "hv_enable") == 0) {
        multicore_fifo_push_blocking(cmd_hv_enable);
        if (multicore_fifo_pop_blocking() == return_ok)
            printf("HV output enabled.\n");
        else
            printf("HV enable failed!\n");
        return true;
    }

    if(strcmp(command, "hvd") == 0 || strcmp(command, "hv_disable") == 0) {
        multicore_fifo_push_blocking(cmd_hv_disable);
        if (multicore_fifo_pop_blocking() == return_ok)
            printf("HV output disabled.\n");
        else
            printf("HV disable failed!\n");
        return true;
    }

    {
        const char *arg = cmd_match(command, "sv");
        if (!arg) arg = cmd_match(command, "set_voltage");
        if (arg) {
            float v;
            if (arg[0] != '\0') {
                v = strtof(arg, NULL);
            } else {
                printf(" target voltage in volts (0 - %.1f)?\n> ", (double)s_soft_limit);
                read_line();
                if (serial_buffer[0] == 0) { printf("Cancelled.\n"); return true; }
                v = strtof(serial_buffer, NULL);
            }
            if (v > s_soft_limit) {
                printf("%.1f V exceeds soft limit (%.1f V). Use lim to raise the limit.\n",
                       (double)v, (double)s_soft_limit);
                return true;
            }
            float_xfer.f = v;
            multicore_fifo_push_blocking(cmd_set_voltage);
            multicore_fifo_push_blocking(float_xfer.ui32);
            if (multicore_fifo_pop_blocking() == return_ok)
                printf("Target voltage set to %.1f V\n", (double)v);
            else
                printf("Set voltage failed!\n");
            return true;
        }
    }

    if(strcmp(command, "lim") == 0 || strcmp(command, "limits") == 0) {
        char **unused;

        printf(" Hard limit — absolute maximum the control loop will target (0 - 1500 V):\n");
        printf(" current: %.1f V\n> ", (double)s_hard_limit);
        read_line();
        if (serial_buffer[0] == 0) { printf("Cancelled.\n"); return true; }
        float hard = strtof(serial_buffer, unused);
        if (hard < 0.0f)    hard = 0.0f;
        if (hard > 1500.0f) hard = 1500.0f;

        printf(" Soft limit — maximum value sv will accept (0 - %.1f V):\n", (double)hard);
        printf(" current: %.1f V\n> ", (double)s_soft_limit);
        read_line();
        if (serial_buffer[0] == 0) { printf("Cancelled.\n"); return true; }
        float soft = strtof(serial_buffer, unused);
        if (soft < 0.0f) soft = 0.0f;
        if (soft > hard) soft = hard;

        float_xfer.f = hard;
        multicore_fifo_push_blocking(cmd_set_hard_limit);
        multicore_fifo_push_blocking(float_xfer.ui32);
        uint32_t r1 = multicore_fifo_pop_blocking();

        float_xfer.f = soft;
        multicore_fifo_push_blocking(cmd_set_soft_limit);
        multicore_fifo_push_blocking(float_xfer.ui32);
        uint32_t r2 = multicore_fifo_pop_blocking();

        if (r1 == return_ok && r2 == return_ok) {
            s_hard_limit = hard;
            s_soft_limit = soft;
            printf("Hard limit: %.1f V   Soft limit: %.1f V  (use csv to persist)\n",
                   (double)hard, (double)soft);
        } else {
            printf("Set limits failed!\n");
        }
        return true;
    }

    // -------------------------------------------------------------------------
    // Monitoring
    // -------------------------------------------------------------------------

    {
        const char *arg = cmd_match(command, "rv");
        if (!arg) arg = cmd_match(command, "read_voltage");
        if (arg) {
            multicore_fifo_push_blocking(cmd_read_voltage_pwm);
            if (multicore_fifo_pop_blocking() == return_ok) {
                uint32_t period_ns = multicore_fifo_pop_blocking();
                float_xfer.ui32 = multicore_fifo_pop_blocking();
                if (strcmp(arg, "period") == 0) {
                    if (period_ns == 0) printf("NULL\n");
                    else printf("%u\n", (unsigned)period_ns);
                } else if (strcmp(arg, "freq") == 0) {
                    if (period_ns == 0) printf("NULL\n");
                    else printf("%.2f\n", (double)(1000000000.0f / (float)period_ns));
                } else if (strcmp(arg, "value") == 0) {
                    if (period_ns == 0) printf("NULL\n");
                    else printf("%.1f\n", (double)float_xfer.f);
                } else {
                    if (period_ns == 0) {
                        printf("Voltage PWM: No signal\n");
                    } else {
                        float hz = 1000000000.0f / (float)period_ns;
                        printf("Voltage PWM: period=%uns  freq=%.2fHz  %.1fV (cal table)\n",
                               period_ns, hz, float_xfer.f);
                    }
                }
            } else {
                printf("Read voltage failed!\n");
            }
            return true;
        }
    }

    {
        const char *arg = cmd_match(command, "ri");
        if (!arg) arg = cmd_match(command, "read_current");
        if (arg) {
            multicore_fifo_push_blocking(cmd_read_current_pwm);
            if (multicore_fifo_pop_blocking() == return_ok) {
                uint32_t period_ns = multicore_fifo_pop_blocking();
                if (strcmp(arg, "period") == 0) {
                    if (period_ns == 0) printf("NULL\n");
                    else printf("%u\n", (unsigned)period_ns);
                } else if (strcmp(arg, "freq") == 0) {
                    if (period_ns == 0) printf("NULL\n");
                    else printf("%.2f\n", (double)(1000000000.0f / (float)period_ns));
                } else if (strcmp(arg, "value") == 0) {
                    printf("NULL\n");
                } else {
                    if (period_ns == 0) {
                        printf("Current PWM: No signal\n");
                    } else {
                        float hz = 1000000000.0f / (float)period_ns;
                        printf("Current PWM: period=%uns  freq=%.2fHz  [no cal table]\n",
                               period_ns, hz);
                    }
                }
            } else {
                printf("Read current failed!\n");
            }
            return true;
        }
    }

    // -------------------------------------------------------------------------
    // DAC / ramp
    // -------------------------------------------------------------------------

    {
        const char *arg = cmd_match(command, "dacw");
        if (!arg) arg = cmd_match(command, "dac_write");
        if (arg) {
            unsigned long val;
            if (arg[0] != '\0') {
                val = strtoul(arg, NULL, 0);
            } else {
                printf(" raw DAC code (0-4095, hex with 0x or decimal)?\n> ");
                read_line();
                if (serial_buffer[0] == 0) { printf("Cancelled.\n"); return true; }
                val = strtoul(serial_buffer, NULL, 0);
            }
            if (val > 4095) {
                printf("Value out of range (0-4095).\n");
                return true;
            }
            multicore_fifo_push_blocking(cmd_debug_dac_raw);
            multicore_fifo_push_blocking((uint32_t)val);
            if (multicore_fifo_pop_blocking() == return_ok) {
                printf("Wrote raw DAC code %lu (0x%03lx, 0b", val, val);
                for (int b = 11; b >= 0; b--) putchar(((val >> b) & 1u) ? '1' : '0');
                printf(")\n");
            } else {
                printf("Raw DAC write failed!\n");
            }
            return true;
        }
    }

    if(strcmp(command, "dacr") == 0 || strcmp(command, "dac_read") == 0) {
        multicore_fifo_push_blocking(cmd_get_dac);
        if (multicore_fifo_pop_blocking() == return_ok) {
            uint32_t code = multicore_fifo_pop_blocking();
            printf("Current DAC code: %u (0x%03x)\n", (unsigned)code, (unsigned)code);
        } else {
            printf("Read DAC failed!\n");
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
        if (multicore_fifo_pop_blocking() == return_ok)
            printf("Ramp set: step_max=%u  step_min=%u  tick_ms=%u  (use csv to persist)\n",
                   (unsigned)smax, (unsigned)smin, (unsigned)tms);
        else
            printf("Set ramp failed!\n");
        return true;
    }

    // -------------------------------------------------------------------------
    // Calibration
    // -------------------------------------------------------------------------

    {
        const char *arg = cmd_match(command, "cap");
        if (!arg) arg = cmd_match(command, "cal_capture");
        if (arg) {
            float v;
            if (arg[0] != '\0') {
                v = strtof(arg, NULL);
            } else {
                printf(" measured voltage at current DAC setting (V)?\n> ");
                read_line();
                if (serial_buffer[0] == 0) { printf("Cancelled.\n"); return true; }
                v = strtof(serial_buffer, NULL);
            }
            float_xfer.f = v;
            multicore_fifo_push_blocking(cmd_cal_capture);
            multicore_fifo_push_blocking(float_xfer.ui32);
            if (multicore_fifo_pop_blocking() == return_ok)
                printf("Captured cal point at %.2f V.\n", (double)v);
            else
                printf("Capture failed (table full or invalid value).\n");
            return true;
        }
    }

    if(strcmp(command, "cls") == 0 || strcmp(command, "cal_list") == 0) {
        multicore_fifo_push_blocking(cmd_cal_list);
        if (multicore_fifo_pop_blocking() != return_ok) {
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
        if (serial_buffer[0] == 0) { printf("Cancelled.\n"); return true; }
        unsigned long idx = strtoul(serial_buffer, unused, 10);
        multicore_fifo_push_blocking(cmd_cal_remove);
        multicore_fifo_push_blocking((uint32_t)idx);
        if (multicore_fifo_pop_blocking() == return_ok)
            printf("Removed cal point %lu.\n", idx);
        else
            printf("Remove failed (index out of range).\n");
        return true;
    }

    if(strcmp(command, "csv") == 0 || strcmp(command, "cal_save") == 0) {
        multicore_fifo_push_blocking(cmd_cal_save);
        if (multicore_fifo_pop_blocking() == return_ok)
            printf("Cal saved to flash.\n");
        else
            printf("Cal save failed!\n");
        return true;
    }

    if(strcmp(command, "crd") == 0 || strcmp(command, "cal_default") == 0) {
        multicore_fifo_push_blocking(cmd_cal_reset);
        if (multicore_fifo_pop_blocking() == return_ok)
            printf("Cal reset to compiled defaults (use csv to persist).\n");
        else
            printf("Cal reset failed!\n");
        return true;
    }

    // -------------------------------------------------------------------------
    // Faults
    // -------------------------------------------------------------------------

    if(strcmp(command, "gf") == 0 || strcmp(command, "get_faults") == 0) {
        multicore_fifo_push_blocking(cmd_get_faults);
        if (multicore_fifo_pop_blocking() == return_ok) {
            uint32_t faults = multicore_fifo_pop_blocking();
            if (faults == 0) {
                printf("No faults.\n");
            } else {
                printf("Faults (0x%02x):\n", faults);
                if (faults & 0x01) printf("  - Circuit open\n");
                if (faults & 0x02) printf("  - Circuit shorted\n");
                if (faults & 0x04) printf("  - Circuit unknown\n");
                if (faults & 0x08) printf("  - Overvoltage\n");
                if (faults & 0x10) printf("  - Overcurrent\n");
            }
        } else {
            printf("Get faults failed!\n");
        }
        return true;
    }

    if(strcmp(command, "cf") == 0 || strcmp(command, "clear_faults") == 0) {
        multicore_fifo_push_blocking(cmd_clear_faults);
        if (multicore_fifo_pop_blocking() == return_ok)
            printf("Faults cleared.\n");
        else
            printf("Clear faults failed!\n");
        return true;
    }

    {
        const char *arg = cmd_match(command, "fi");
        if (!arg) arg = cmd_match(command, "fault_ignore");
        if (arg) {
            if (arg[0] == '\0') {
                // No argument: get current state.
                multicore_fifo_push_blocking(cmd_get_fault_ignore);
                if (multicore_fifo_pop_blocking() == return_ok) {
                    uint32_t v = multicore_fifo_pop_blocking();
                    printf("Fault ignore: %s\n", v ? "on" : "off");
                } else {
                    printf("Get fault ignore failed!\n");
                }
            } else {
                // Argument: set state.
                bool enable;
                if (strcmp(arg, "true") == 0 || strcmp(arg, "1") == 0 || strcmp(arg, "on") == 0)
                    enable = true;
                else if (strcmp(arg, "false") == 0 || strcmp(arg, "0") == 0 || strcmp(arg, "off") == 0)
                    enable = false;
                else {
                    printf("Expected true/false (or 1/0, on/off).\n");
                    return true;
                }
                multicore_fifo_push_blocking(cmd_set_fault_ignore);
                multicore_fifo_push_blocking(enable ? 1u : 0u);
                if (multicore_fifo_pop_blocking() == return_ok)
                    printf("Fault ignore: %s\n", enable ? "on" : "off");
                else
                    printf("Set fault ignore failed!\n");
            }
            return true;
        }
    }

    // -------------------------------------------------------------------------
    // System
    // -------------------------------------------------------------------------

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

    // Load persisted limits from Core 0 (cal module).
    multicore_fifo_push_blocking(cmd_get_hard_limit);
    if (multicore_fifo_pop_blocking() == return_ok) {
        float_xfer.ui32 = multicore_fifo_pop_blocking();
        s_hard_limit = float_xfer.f;
    }
    multicore_fifo_push_blocking(cmd_get_soft_limit);
    if (multicore_fifo_pop_blocking() == return_ok) {
        float_xfer.ui32 = multicore_fifo_pop_blocking();
        s_soft_limit = float_xfer.f;
    }

    while(1) {
        read_line();
        if (!handle_command(serial_buffer)) {
            printf("BIO-RAD Commands:\n");
            printf("\n");
            printf("  System:\n");
            printf("    h / help       show this help\n");
            printf("    s / status     show all state at a glance\n");
            printf("    r / reset      reset device\n");
            printf("    b / bootload   reboot into USB firmware loading mode\n");
            printf("\n");
            printf("  HV Output:\n");
            printf("    hve            hv_enable: enable HV output\n");
            printf("    hvd            hv_disable: disable HV output\n");
            printf("    sv [V]         set target voltage (capped at soft limit)\n");
            printf("    lim            set soft limit (sv cap) and hard limit (control ceiling)\n");
            printf("\n");
            printf("  Monitoring:\n");
            printf("    rv [period|freq|value]  read_voltage: all fields, or one (NULL if no signal/cal)\n");
            printf("    ri [period|freq|value]  read_current: all fields, or one (value always NULL)\n");
            printf("\n");
            printf("  DAC / Ramp:\n");
            printf("    dacw [code]    dac_write: raw 12-bit code (0-4095); pauses ramp if HV on\n");
            printf("    dacr           dac_read: current DAC code\n");
            printf("    sr             set_ramp: parabolic ramp params (step_max, step_min, tick_ms)\n");
            printf("\n");
            printf("  Calibration:\n");
            printf("    cap [V]        cal_capture: record cal point at measured volts\n");
            printf("    cls            cal_list: show voltage->DAC table\n");
            printf("    crm            cal_remove: remove a cal point by index\n");
            printf("    csv            cal_save: persist cal, limits, and ramp to flash\n");
            printf("    crd            cal_default: reset to compiled defaults (csv to persist)\n");
            printf("\n");
            printf("  Faults:\n");
            printf("    gf             get_faults: show active fault flags\n");
            printf("    cf             clear_faults: clear sticky fault register\n");
            printf("    fi [true|false]  fault_ignore: get state, or set on/off\n");
        }
        printf("\n");

        if (last_command[0] != 0)
            printf("[%s] > ", last_command);
        else
            printf(" > ");
    }
}
