# chipshouter-picoemp / BIO-RAD PSU Controller

Firmware for a Raspberry Pi Pico that controls a BIO-RAD 3000Xi high-voltage
power supply. The Pico replaces the original front-panel controller, providing
closed-loop voltage regulation via a calibration table, a parabolic DAC ramp,
and a USB serial command interface.

Hardware schematics are in the [`hardware/`](hardware/) folder.
All active firmware is in [`firmware/c/`](firmware/c/).

---

## Building the Firmware

Requires the [Pico SDK](https://github.com/raspberrypi/pico-sdk).

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
cd firmware/c
cmake -S . -B build
cmake --build build
```

Output: `firmware/c/build/picoemp.uf2`

## Flashing

Hold **BOOTSEL** on the Pico while connecting USB, then copy the `.uf2` to the
mounted `RPI-RP2` drive. The Pico reboots automatically into the new firmware.

---

## Serial Console

Connect at any baud rate over USB-CDC. Type `h` for help. An empty line repeats
the last command.

```
BIO-RAD Commands:

  System:
    h / help                show this help
    s / status              show all state at a glance
    r / reset               reset device
    b / bootload            reboot into USB firmware loading mode

  HV Output:
    hve                     hv_enable: enable HV output
    hvd                     hv_disable: disable HV output
    sv [V]                  set target voltage (capped at soft limit)
    lim                     set soft limit (sv cap) and hard limit (control ceiling)

  Monitoring:
    rv [period|freq|value]  read_voltage: all fields, or one (NULL if no signal/cal)
    ri [period|freq|value]  read_current: all fields, or one (value always NULL)

  DAC / Ramp:
    dacw [code]             dac_write: raw 12-bit code (0-4095); pauses ramp if HV on
    dacr                    dac_read: current DAC code
    sr                      set_ramp: parabolic ramp params (step_max, step_min, tick_ms)

  Calibration:
    cap [V]                 cal_capture: record cal point at measured volts
    cls                     cal_list: show voltage->DAC table
    crm                     cal_remove: remove a cal point by index
    csv                     cal_save: persist cal, limits, and ramp to flash
    crd                     cal_default: reset to compiled defaults (csv to persist)

  Faults:
    gf                      get_faults: show active fault flags
    cf                      clear_faults: clear sticky fault register
    fi [true|false]         fault_ignore: get state, or set on/off
```

### Calibration workflow

1. Enable HV output (`hve`) and set a target voltage (`sv <V>`).
2. Measure the actual output voltage with a meter.
3. Capture a cal point at the measured voltage (`cap <measured_V>`).
4. Repeat across the desired voltage range.
5. Save to flash (`csv`).

The control loop interpolates between captured points to map DAC codes to
volts. `cls` shows the current table; `crm` removes a point by index; `crd`
resets to compiled defaults.
