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

static union float_union {float f; uint32_t ui32;} float_xfer;

int main() {
    stdio_init_all();

    picoemp_init();
    dac_init();
    cal_init();
    control_loop_init();

    multicore_launch_core1(serial_console);

    while(1) {
        psu_monitor_update();
        control_loop_tick();
        control_loop_led_tick();

        while(multicore_fifo_rvalid()) {
            uint32_t command = multicore_fifo_pop_blocking();
            switch(command) {
                case cmd_read_voltage_pwm: {
                    uint32_t period_ns = psu_monitor_voltage_is_valid()
                        ? psu_monitor_get_voltage_period_ns() : 0;
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking(period_ns);
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
                    cal_set_soft_limit(float_xfer.f);
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_debug_dac_raw: {
                    uint32_t raw = multicore_fifo_pop_blocking();
                    uint16_t code = (uint16_t)(raw & 0x0FFFu);
                    while (!dac_write_done()) tight_loop_contents();
                    dac_write(code);
                    control_loop_set_current_dac(code);
                    if (control_loop_is_enabled()) control_loop_set_manual_mode(true);
                    multicore_fifo_push_blocking(return_ok);
                    break;
                }
                case cmd_cal_capture: {
                    float_xfer.ui32 = multicore_fifo_pop_blocking();
                    bool ok = cal_capture(float_xfer.f, control_loop_get_current_dac());
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
                case cmd_set_ramp: {
                    uint16_t smax = (uint16_t)multicore_fifo_pop_blocking();
                    uint16_t smin = (uint16_t)multicore_fifo_pop_blocking();
                    uint32_t tms  = multicore_fifo_pop_blocking();
                    cal_set_ramp(smax, smin, tms);
                    multicore_fifo_push_blocking(return_ok);
                    break;
                }
                case cmd_get_ramp: {
                    uint16_t smax, smin; uint32_t tms;
                    cal_get_ramp(&smax, &smin, &tms);
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking((uint32_t)smax);
                    multicore_fifo_push_blocking((uint32_t)smin);
                    multicore_fifo_push_blocking(tms);
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
                        multicore_fifo_push_blocking((uint32_t)pt.dac_code);
                    }
                    multicore_fifo_push_blocking((uint32_t)cal_source());
                    break;
                }
                case cmd_get_dac:
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking((uint32_t)control_loop_get_current_dac());
                    break;
                case cmd_get_hv_enabled:
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking((uint32_t)control_loop_is_enabled());
                    break;
                case cmd_get_target_voltage:
                    float_xfer.f = control_loop_get_target_volts();
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking(float_xfer.ui32);
                    break;
                case cmd_get_soft_limit:
                    float_xfer.f = cal_get_soft_limit();
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking(float_xfer.ui32);
                    break;
                case cmd_get_manual_mode:
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking((uint32_t)control_loop_in_manual_mode());
                    break;
                case cmd_set_hard_limit:
                    float_xfer.ui32 = multicore_fifo_pop_blocking();
                    cal_set_hard_limit(float_xfer.f);
                    multicore_fifo_push_blocking(return_ok);
                    break;
                case cmd_get_hard_limit:
                    float_xfer.f = cal_get_hard_limit();
                    multicore_fifo_push_blocking(return_ok);
                    multicore_fifo_push_blocking(float_xfer.ui32);
                    break;
            }
        }
    }

    return 0;
}
