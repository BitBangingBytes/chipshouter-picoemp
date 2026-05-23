#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "picoemp.h"
#include "psu_monitor.h"
#include "dac.h"
#include "cal.h"
#include "control_loop.h"
#include "serial.h"

#include "trigger_basic.pio.h"

static bool armed = false;
static bool timeout_active = true;
static bool hvp_internal = true;
static absolute_time_t timeout_time;
static uint offset = 0xFFFFFFFF;

// defaults taken from original code
#define PULSE_DELAY_CYCLES_DEFAULT 0
#define PULSE_TIME_CYCLES_DEFAULT 625 // 5us in 8ns cycles
#define PULSE_TIME_US_DEFAULT 5 // 5us
static uint32_t pulse_time;
static uint32_t pulse_delay_cycles;
static uint32_t pulse_time_cycles;
static union float_union {float f; uint32_t ui32;} float_xfer;

void arm() {
    gpio_put(PIN_LED_CHARGE_ON, true);
    armed = true;
}

void disarm() {
    gpio_put(PIN_LED_CHARGE_ON, false);
    armed = false;
    // picoemp_disable_pwm();  // disabled: external PSU replaces on-board HV generation
}

uint32_t get_status() {
    uint32_t result = 0;
    if(armed) {
        result |= 0b1;
    }
    if(gpio_get(PIN_IN_CHARGED)) {
        result |= 0b10;
    }
    if(timeout_active) {
        result |= 0b100;
    }
    if(hvp_internal) {
        result |= 0b1000;
    }
    if(control_loop_get_faults_ignored()) {
        result |= 0b10000;
    }
    return result;
}

void update_timeout() {
    timeout_time = delayed_by_ms(get_absolute_time(), 60 * 1000);
}

void fast_trigger() {
    // Choose which PIO instance to use (there are two instances)
    PIO pio = pio0;

    // Our assembled program needs to be loaded into this PIO's instruction
    // memory. This SDK function will find a location (offset) in the
    // instruction memory where there is enough space for our program. We need
    // to remember this location!
    if (offset == 0xFFFFFFFF) { // Only load the program once
        offset = pio_add_program(pio, &trigger_basic_program);
    }
    
    // Find a free state machine on our chosen PIO (erroring if there are
    // none). Configure it to run our program, and start it, using the
    // helper function we included in our .pio file.
    uint sm = 0;
    trigger_basic_init(pio, sm, offset, PIN_IN_TRIGGER, PIN_OUT_HVPULSE);
    pio_sm_put_blocking(pio, sm, pulse_delay_cycles);
    pio_sm_put_blocking(pio, sm, pulse_time_cycles);

}

int main() {
    // Initialize USB-UART as STDIO
    stdio_init_all();

    picoemp_init();
    dac_init();
    cal_init();
    control_loop_init();

    // Init for reset pin (move somewhere else)
    gpio_init(1);
    gpio_set_dir(1, GPIO_OUT);
    gpio_put(1, 1);

    // Run serial-console on second core
    multicore_launch_core1(serial_console);

    pulse_time = PULSE_TIME_US_DEFAULT;
    pulse_delay_cycles = PULSE_DELAY_CYCLES_DEFAULT;
    pulse_time_cycles = PULSE_TIME_CYCLES_DEFAULT;

    while(1) {
        psu_monitor_update();
        control_loop_tick();
        control_loop_led_tick();

        gpio_put(PIN_LED_HV, gpio_get(PIN_IN_CHARGED));

        // Handle serial commands (if any)
        while(multicore_fifo_rvalid()) {
            uint32_t command = multicore_fifo_pop_blocking();
            switch(command) {
                case cmd_arm:
                    arm();
                    update_timeout();
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_disarm:
                    disarm();
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_pulse:
                    picoemp_pulse(pulse_time);
                    update_timeout();
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_status:
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking(get_status());
                    break;
                case cmd_enable_timeout:
                    timeout_active = true;
                    update_timeout();
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_disable_timeout:
                    timeout_active = false;
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_config_pulse_delay_cycles:
                    pulse_delay_cycles = multicore_fifo_pop_blocking();
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_config_pulse_time_cycles:
                    pulse_time_cycles = multicore_fifo_pop_blocking();
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_fast_trigger:
                    fast_trigger();
                    multicore_fifo_push_blocking(return_ok);
                    while(!pio_interrupt_get(pio0, 0));
                    multicore_fifo_push_blocking(return_ok);
                    pio_sm_set_enabled(pio0, 0, false);
                    picoemp_configure_pulse_output();
                    break;
                case cmd_internal_hvp:
                    picoemp_configure_pulse_output();
                    hvp_internal = true;
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_external_hvp:
                    picoemp_configure_pulse_external();
                    hvp_internal = false;
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_config_pulse_time:
                    pulse_time = multicore_fifo_pop_blocking();
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_toggle_gp1:
                    gpio_xor_mask(1<<1);
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_read_voltage_pwm: {
                    uint32_t period_ns = psu_monitor_voltage_is_valid()
                        ? psu_monitor_get_voltage_period_ns() : 0;
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking(period_ns);
                    // Also push the cal-table-derived volts so Core 1 doesn't
                    // re-implement the conversion with stale constants.
                    float_xfer.f = cal_period_ns_to_volts(period_ns);
                    multicore_fifo_push_blocking(float_xfer.ui32);
                    break;
                }
                case cmd_read_current_pwm:
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking(
                        psu_monitor_current_is_valid() ? psu_monitor_get_current_period_ns() : 0
                    );
                    break;
                case cmd_hv_enable:
                    control_loop_enable(true);
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_hv_disable:
                    control_loop_enable(false);
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_set_voltage:
                    float_xfer.ui32 = multicore_fifo_pop_blocking();
                    control_loop_set_target_volts(float_xfer.f);
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_get_actual_voltage:
                    float_xfer.f = control_loop_get_actual_volts();
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking(float_xfer.ui32);
                    break;
                case cmd_get_actual_current:
                    float_xfer.f = control_loop_get_actual_current();
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking(float_xfer.ui32);
                    break;
                case cmd_get_faults:
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking(control_loop_get_faults());
                    break;
                case cmd_clear_faults:
                    control_loop_clear_faults();
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_set_soft_limit:
                    float_xfer.ui32 = multicore_fifo_pop_blocking();
                    control_loop_set_soft_limit(float_xfer.f);
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_debug_dac_raw: {
                    uint32_t raw = multicore_fifo_pop_blocking();
                    // Wait for any in-flight DMA to drain before issuing a new write.
                    while (!dac_write_done()) tight_loop_contents();
                    dac_write((uint16_t)(raw & 0x0FFFu));
                    // If HV is enabled, pause the closed loop so it doesn't fight us.
                    if (control_loop_is_enabled()) control_loop_set_manual_mode(true);
                    multicore_fifo_push_blocking(return_ok);
                    break;
                }
                case cmd_cal_capture: {
                    float_xfer.ui32 = multicore_fifo_pop_blocking();
                    bool ok = cal_capture(float_xfer.f);
                    multicore_fifo_push_blocking(ok ? return_ok : return_failed);
                    break;
                }
                case cmd_cal_remove: {
                    uint32_t idx = multicore_fifo_pop_blocking();
                    bool ok = cal_remove(idx);
                    multicore_fifo_push_blocking(ok ? return_ok : return_failed);
                    break;
                }
                case cmd_cal_reset:
                    cal_reset_to_defaults();
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_cal_save: {
                    bool ok = cal_save();
                    multicore_fifo_push_blocking(ok ? return_ok : return_failed);
                    break;
                }
                case cmd_set_fault_ignore:
                    control_loop_set_faults_ignored((bool)multicore_fifo_pop_blocking());
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_get_fault_ignore:
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking((uint32_t)control_loop_get_faults_ignored());
                    break;
                case cmd_set_dac_refresh: {
                    uint32_t ms = multicore_fifo_pop_blocking();
                    control_loop_set_dac_refresh_ms(ms);
                    multicore_fifo_push_blocking(return_ok);
                    break;
                }
                case cmd_get_dac_refresh:
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking(control_loop_get_dac_refresh_ms());
                    break;
                case cmd_cal_list: {
                    uint n = cal_num_points();
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking((uint32_t)n);
                    for (uint i = 0; i < n; i++) {
                        cal_point_t pt;
                        cal_get_point(i, &pt);
                        float_xfer.f = pt.volts;
                        multicore_fifo_push_blocking(float_xfer.ui32);
                        float_xfer.f = pt.hz;
                        multicore_fifo_push_blocking(float_xfer.ui32);
                    }
                    multicore_fifo_push_blocking((uint32_t)cal_source());
                    break;
                }
            }
        }

        // Pulse
        if(gpio_get(PIN_BTN_PULSE)) {
            update_timeout();
            picoemp_pulse(pulse_time);
        }

        if(gpio_get(PIN_BTN_ARM)) {
            update_timeout();
            if(!armed) {
                arm();
            } else {
                disarm();
            }
            // YOLO debouncing
            while(gpio_get(PIN_BTN_ARM));
            sleep_ms(100);
        }

        // disabled: external PSU replaces on-board HV generation
        // if(!gpio_get(PIN_IN_CHARGED) && armed) {
        //     picoemp_enable_pwm(pulse_power.f);
        // }

        if(timeout_active && (get_absolute_time() > timeout_time) && armed) {
            disarm();
        }
    }
    
    return 0;
}
