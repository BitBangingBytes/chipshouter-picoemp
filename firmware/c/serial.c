#include "serial.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/watchdog.h"

static char serial_buffer[256];
static char last_command[256];

#define PULSE_DELAY_CYCLES_DEFAULT 0
#define PULSE_TIME_CYCLES_DEFAULT 625 // 5us in 8ns cycles
#define PULSE_TIME_US_DEFAULT 5 // 5us
static uint32_t pulse_time;
static uint32_t pulse_delay_cycles;
static uint32_t pulse_time_cycles;
static union float_union {float f; uint32_t ui32;} float_xfer;

void read_line() {
    memset(serial_buffer, 0, sizeof(serial_buffer));
    while(1) {
        int c = getchar();
        if(c == EOF) {
            return;
        }

        putchar(c);

        if(c == '\r') {
            return;
        }
        if(c == '\n') {
            continue;
        }

        // buffer full, just return.
        if(strlen(serial_buffer) >= 255) {
            return;
        }

        serial_buffer[strlen(serial_buffer)] = (char)c;
    }
}

void print_status(uint32_t status) {
    bool armed = (status >> 0) & 1;
    bool charged = (status >> 1) & 1;
    bool timeout_active = (status >> 2) & 1;
    bool hvp_mode = (status >> 3) & 1;
    printf("Status:\n");
    if(armed) {
        printf("- Armed\n");
    } else {
        printf("- Disarmed\n");
    }
    if(charged) {
        printf("- Charged\n");
    } else {
        printf("- Not charged\n");
    }
    if(timeout_active) {
        printf("- Timeout active\n");
    } else {
        printf("- Timeout disabled\n");
    }
    if(hvp_mode) {
        printf("- HVP internal\n");
    } else {
        printf("- HVP external\n");
    }
    bool faults_ignored = (status >> 4) & 1;
    if(faults_ignored) {
        printf("- Fault ignore ON (faults recorded but do not shut down)\n");
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

    if(strcmp(command, "a") == 0 || strcmp(command, "arm") == 0) {
        multicore_fifo_push_blocking(cmd_arm);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok) {
            printf("Device armed!\n");
        } else {
            printf("Arming failed!\n");
        }
        return true;
    }
    if(strcmp(command, "d") == 0 || strcmp(command, "disarm") == 0) {
        multicore_fifo_push_blocking(cmd_disarm);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok) {
            printf("Device disarmed!\n");
        } else {
            printf("Disarming failed!\n");
        }
        return true;
    }
    if(strcmp(command, "p") == 0 || strcmp(command, "pulse") == 0) {
        multicore_fifo_push_blocking(cmd_pulse);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok) {
            printf("Pulsed!\n");
        } else {
            printf("Pulse failed!\n");
        }
        return true;
    }
    if(strcmp(command, "s") == 0 || strcmp(command, "status") == 0) {
        multicore_fifo_push_blocking(cmd_status);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok) {
            print_status(multicore_fifo_pop_blocking());
        } else {
            printf("Getting status failed!\n");
        }
        return true;
    }
    if(strcmp(command, "en") == 0 || strcmp(command, "enable_timeout") == 0) {
        multicore_fifo_push_blocking(cmd_enable_timeout);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok) {
            printf("Timeout enabled!\n");
        } else {
            printf("Enabling timeout failed!\n");
        }
        return true;
    }
    if(strcmp(command, "di") == 0 || strcmp(command, "disable_timeout") == 0) {
        multicore_fifo_push_blocking(cmd_disable_timeout);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok) {
            printf("Timeout disabled!\n");
        } else {
            printf("Disabling timeout failed!\n");
        }
        return true;
    }
    if(strcmp(command, "f") == 0 || strcmp(command, "fast_trigger") == 0) {
        multicore_fifo_push_blocking(cmd_fast_trigger);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok) {
            printf("Fast trigger active...\n");
            multicore_fifo_pop_blocking();
            printf("Triggered!\n");
        } else {
            printf("Setting up fast trigger failed.");
        }
        return true;
    }
    if(strcmp(command, "fa") == 0 || strcmp(command, "fast_trigger_configure") == 0) {
        char **unused;
        printf(" configure in cycles\n");
        printf("  1 cycle = 8ns\n");
        printf("  1us = 125 cycles\n");
        printf("  1ms = 125000 cycles\n");
        printf("  max = MAX_UINT32 = 4294967295 cycles = 34359ms\n");

        printf(" pulse_delay_cycles (current: %d, default: %d)?\n> ", pulse_delay_cycles, PULSE_DELAY_CYCLES_DEFAULT);
        read_line();
        printf("\n");
        if (serial_buffer[0] == 0)
            printf("Using default\n");
        else
            pulse_delay_cycles = strtoul(serial_buffer, unused, 10);
        
        printf(" pulse_time_cycles (current: %d, default: %d)?\n> ", pulse_time_cycles, PULSE_TIME_CYCLES_DEFAULT);
        read_line();
        printf("\n");
        if (serial_buffer[0] == 0)
            printf("Using default\n");
        else
            pulse_time_cycles = strtoul(serial_buffer, unused, 10);

        multicore_fifo_push_blocking(cmd_config_pulse_delay_cycles);
        multicore_fifo_push_blocking(pulse_delay_cycles);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result != return_ok) {
            printf("Config pulse_delay_cycles failed.");
        }

        multicore_fifo_push_blocking(cmd_config_pulse_time_cycles);
        multicore_fifo_push_blocking(pulse_time_cycles);
        result = multicore_fifo_pop_blocking();
        if(result != return_ok) {
            printf("Config pulse_time_cycles failed.");
        }

        printf("pulse_delay_cycles=%d, pulse_time_cycles=%d\n", pulse_delay_cycles, pulse_time_cycles);

        return true;
    }
    if(strcmp(command, "in") == 0 || strcmp(command, "internal_hvp") == 0) {
        multicore_fifo_push_blocking(cmd_internal_hvp);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok) {
            printf("Internal HVP mode active!\n");
        } else {
            printf("Setting up internal HVP mode failed.");
        }
        return true;
    }
    if(strcmp(command, "ex") == 0 || strcmp(command, "external_hvp") == 0) {
        multicore_fifo_push_blocking(cmd_external_hvp);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result == return_ok) {
            printf("External HVP mode active!\n");
        } else {
            printf("Setting up external HVP mode failed.");
        }
        return true;
    }

    if(strcmp(command, "c") == 0 || strcmp(command, "configure") == 0) {
        char **unused;
        printf(" pulse_time (current: %d, default: %d)?\n> ", pulse_time, PULSE_TIME_US_DEFAULT);
        read_line();
        printf("\n");
        if (serial_buffer[0] == 0)
            printf("Using default\n");
        else
            pulse_time = strtoul(serial_buffer, unused, 10);

        multicore_fifo_push_blocking(cmd_config_pulse_time);
        multicore_fifo_push_blocking(pulse_time);
        uint32_t result = multicore_fifo_pop_blocking();
        if(result != return_ok) {
            printf("Config pulse_time failed.");
        }

        printf("pulse_time=%d\n", pulse_time);

        return true;
    }

    if(strcmp(command, "t") == 0 || strcmp(command, "toggle_gp1") == 0) {
        multicore_fifo_push_blocking(cmd_toggle_gp1);
        
        uint32_t result = multicore_fifo_pop_blocking();
        if(result != return_ok) {
            printf("target_reset failed.");
        }

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
                // TODO: replace with calibrated lookup table once bench measurements are taken
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
        printf("\n");
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
        printf("\n");
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

    if(strcmp(command, "sdr") == 0 || strcmp(command, "set_dac_refresh") == 0) {
        char **unused;
        printf(" DAC refresh interval in ms (0 = disable, default: 500)?\n> ");
        read_line();
        printf("\n");
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

    if(strcmp(command, "dd") == 0 || strcmp(command, "debug_dac") == 0) {
        char **unused;
        printf(" raw DAC code (0-4095, hex with 0x or decimal)?\n> ");
        read_line();
        printf("\n");
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

    if(strcmp(command, "cap") == 0 || strcmp(command, "cal_capture") == 0) {
        char **unused;
        printf(" measured voltage at current PWM (V)?\n> ");
        read_line();
        printf("\n");
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
        for (uint32_t i = 0; i < n; i++) {
            float_xfer.ui32 = multicore_fifo_pop_blocking();
            float volts = float_xfer.f;
            float_xfer.ui32 = multicore_fifo_pop_blocking();
            float hz = float_xfer.f;
            printf("  [%u] %8.2f V -> %10.2f Hz\n", (unsigned)i, volts, hz);
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
        printf("\n");
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

    return false;
}

void serial_console() {
    multicore_fifo_drain();

    memset(last_command, 0, sizeof(last_command));

    pulse_time = PULSE_TIME_US_DEFAULT;
    pulse_delay_cycles = PULSE_DELAY_CYCLES_DEFAULT;
    pulse_time_cycles = PULSE_TIME_CYCLES_DEFAULT;
    
    while(1) {
        read_line();
        printf("\n");
        if(!handle_command(serial_buffer)) {
            printf("PicoEMP Commands:\n");
            printf("- <empty to repeat last command>\n");
            printf("- [h]elp\n");
            printf("- [a]rm\n");
            printf("- [d]isarm\n");
            printf("- [p]ulse\n");
            printf("- [en]able_timeout\n");
            printf("- [di]sable_timeout\n");
            printf("- [f]ast_trigger\n");
            printf("- [fa]st_trigger_configure: delay_cycles=%d, time_cycles=%d\n", pulse_delay_cycles, pulse_time_cycles);
            printf("- [in]ternal_hvp\n");
            printf("- [ex]ternal_hvp\n");
            printf("- [c]onfigure: pulse_time=%d\n", pulse_time);
            printf("- [t]oggle_gp1\n");
            printf("- [s]tatus\n");
            printf("- [r]eset\n");
            printf("- [rv] read_voltage: raw PWM period, Hz, and cal-table volts\n");
            printf("- [ri] read_current: raw PWM period, Hz\n");
            printf("- [hve] hv_enable: enable HV output / control loop\n");
            printf("- [hvd] hv_disable: disable HV output / control loop\n");
            printf("- [sv] set_voltage: set target voltage (V)\n");
            printf("- [sl] set_limit: set soft voltage limit (V)\n");
            printf("- [av] actual_voltage: read estimated actual voltage\n");
            printf("- [ai] actual_current: read current feedback (Hz)\n");
            printf("- [gf] get_faults: show active fault flags\n");
            printf("- [cf] clear_faults: clear sticky fault register\n");
            printf("- [fi] fault_ignore: record faults but suppress shutdown\n");
            printf("- [fn] fault_normal: restore normal fault shutdown behavior\n");
            printf("- [dd] debug_dac: write raw 12-bit DAC code (0-4095); pauses closed loop if HV is on\n");
            printf("- [sdr] set_dac_refresh: set periodic DAC resend interval in ms (0 = disable)\n");
            printf("- [gdr] get_dac_refresh: show current DAC refresh interval\n");
            printf("- [cap] cal_capture: record current PWM as a cal point at user-measured volts\n");
            printf("- [cls] cal_list: show current PWM->volts cal table and its source\n");
            printf("- [crm] cal_remove: remove a cal point by index\n");
            printf("- [csv] cal_save: persist current cal to flash\n");
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