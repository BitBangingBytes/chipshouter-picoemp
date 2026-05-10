#include "dac.h"
#include "picoemp.h"
#include "dac_sequencer.pio.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"

// pio1 SM2: SM0=volt, SM1=curr (psu_monitor), SM2=dac_sequencer
#define DAC_PIO  pio1
#define DAC_SM   2u

// 3-bit pin field encoding.
// RP2040 PIO 'out pins, 3' with shift-left maps the LOWEST bit of the shifted
// group to the BASE pin (GPIO 20), and the HIGHEST bit to BASE+2 (GPIO 22):
//   bit 29 (LSB of group) → GPIO 20 = PIN_OUT_HV_Data
//   bit 30               → GPIO 21 = PIN_OUT_HV_Clock
//   bit 31 (MSB of group) → GPIO 22 = PIN_OUT_HV_Strobe
//
// Enable (GPIO 19) is NOT driven by the PIO — held by control_loop via gpio_put().
#define PIN_DAT  (1u << 29)   // GPIO 20 = Data  (HIGH = logic 0, LOW = logic 1)
#define PIN_CLK  (1u << 30)   // GPIO 21 = Clock (HIGH = idle, LOW = active)
#define PIN_STR  (1u << 31)   // GPIO 22 = Strobe

// Timing — delay values for the PIO word.
// Pin-state duration = delay_value + 4 µs (4-cycle instruction overhead per word).
// Empirically calibrated: delay=1 → 5 µs, overhead = 4 µs.
//
// Clock:  43 µs LOW  → delay = 43 - 4 = 39
//         84 µs HIGH total, split into two 42 µs halves → delay = 42 - 4 = 38 each
//           Half 1: hold current data stable after rising edge
//           Half 2: present next data bit (setup for next falling edge)
// Strobe: 40 µs HIGH → delay = 40 - 4 = 36
#define T_CLK_LOW_US        39u   // → 43 µs
#define T_CLK_HIGH_HOLD_US  38u   // → 42 µs (data hold after rising edge)
#define T_CLK_HIGH_SETUP_US 38u   // → 42 µs (next data setup before falling edge)
#define T_STROBE_US         36u   // → 40 µs
#define T_IDLE_US            0u   // →  4 µs trailing idle

// Maximum words: 12 bits × 3 words + 2 (strobe + idle) = 38 → round to 40
#define SEQ_MAX_WORDS 40u

static uint pio_offset;
static int  dma_chan;

// Double-buffer so we can build the next sequence while current DMA runs.
static uint32_t seq_buf[2][SEQ_MAX_WORDS];
static uint     seq_active = 0;

static inline uint32_t word(uint32_t pins, uint32_t delay_us) {
    // pins occupy bits [31:29]; delay occupies bits [28:0]
    return pins | (delay_us & 0x1FFFFFFFu);
}

// Idle state: CLK=HIGH, DAT=HIGH (= logic 0), STR=LOW
#define IDLE_PINS (PIN_CLK | PIN_DAT)

// Build the 38-word sequence for a 12-bit DAC write.
// Protocol: clock idles HIGH; data captured on rising edge; 3 words per bit:
//   Word A: CLK=LOW,  DAT=current bit          → 43 µs
//   Word B: CLK=HIGH, DAT=current bit (hold)   → 42 µs  ← rising edge at start
//   Word C: CLK=HIGH, DAT=next bit    (setup)  → 42 µs  ← data changes here
// Data is inverted: logic 0 → GPIO HIGH (PIN_DAT set), logic 1 → GPIO LOW.
static uint build_sequence(uint32_t *buf, uint16_t value) {
    uint n = 0;

    for (int bit = 11; bit >= 0; bit--) {
        uint32_t dat = ((value >> bit) & 1u) ? 0u : PIN_DAT;   // inverted

        uint32_t next_dat;
        if (bit > 0)
            next_dat = ((value >> (bit-1)) & 1u) ? 0u : PIN_DAT;
        else
            next_dat = PIN_DAT;   // data returns to idle (HIGH = logic 0) after last bit

        // A: CLK LOW — current bit stable, falling edge at start of this word
        buf[n++] = word(dat, T_CLK_LOW_US);

        // B: CLK HIGH — rising edge, hold current bit stable for 42 µs
        buf[n++] = word(PIN_CLK | dat, T_CLK_HIGH_HOLD_US);

        // C: CLK HIGH — data transitions to next bit, setup time 42 µs
        buf[n++] = word(PIN_CLK | next_dat, T_CLK_HIGH_SETUP_US);
    }

    // Strobe HIGH for 40 µs to latch shift register into DAC output register
    buf[n++] = word(PIN_CLK | PIN_DAT | PIN_STR, T_STROBE_US);
    // Return to idle
    buf[n++] = word(IDLE_PINS, T_IDLE_US);

    return n;
}

void dac_init() {
    pio_offset = pio_add_program(DAC_PIO, &dac_sequencer_program);
    // PIO controls GPIO 20-22 (Data, Clock, Strobe); GPIO 19 (Enable) stays as GPIO.
    dac_sequencer_program_init(DAC_PIO, DAC_SM, pio_offset, PIN_OUT_HV_Data);

    dma_chan = dma_claim_unused_channel(true);

    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, pio_get_dreq(DAC_PIO, DAC_SM, true));
    dma_channel_configure(dma_chan, &cfg,
        &DAC_PIO->txf[DAC_SM],
        seq_buf[0],
        SEQ_MAX_WORDS,
        false);  // don't start yet
}

void dac_write(uint16_t value) {
    uint next = seq_active ^ 1u;
    uint count = build_sequence(seq_buf[next], value);
    seq_active = next;

    dma_channel_set_read_addr(dma_chan, seq_buf[seq_active], false);
    dma_channel_set_trans_count(dma_chan, count, true);
}

bool dac_write_done() {
    return !dma_channel_is_busy(dma_chan);
}
