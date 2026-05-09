#include "dac.h"
#include "picoemp.h"
#include "dac_sequencer.pio.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"

// pio1 SM2: SM0=volt, SM1=curr (psu_monitor), SM2=dac_sequencer
#define DAC_PIO  pio1
#define DAC_SM   2u

// 3-bit pin field encoding (MSB→LSB maps to lowest→highest GPIO):
//   bit 31 → GPIO 20 = Data  (HIGH = logic 0, LOW = logic 1 — inverted)
//   bit 30 → GPIO 21 = Clock (HIGH = idle,    LOW = active  — inverted)
//   bit 29 → GPIO 22 = Strobe
//   bits [28:0] → delay count
//
// Enable (GPIO 19) is NOT driven by the PIO — held by control_loop via gpio_put().
#define PIN_DAT  (1u << 31)   // GPIO 20
#define PIN_CLK  (1u << 30)   // GPIO 21
#define PIN_STR  (1u << 29)   // GPIO 22

// Timing — delay values for the PIO word.
// Pin-state duration = delay_value + 4 µs (4-cycle instruction overhead per word).
// Empirically verified: delay=1 → 5 µs, so overhead = 4 µs.
#define T_CLK_LOW_US   39u   // 39 + 4 = 43 µs clock-low  phase
#define T_CLK_HIGH_US  80u   // 80 + 4 = 84 µs clock-high phase
#define T_STROBE_US     0u   //  0 + 4 =  4 µs strobe pulse (minimum)
#define T_IDLE_US       0u   //  0 + 4 =  4 µs trailing idle

// Maximum words per DAC write: 12 bits × 2 words + 2 (strobe + idle) = 26
#define SEQ_MAX_WORDS 28u

static uint pio_offset;
static int  dma_chan;

// Double-buffer so we can build the next sequence while current DMA runs.
static uint32_t seq_buf[2][SEQ_MAX_WORDS];
static uint     seq_active = 0;

static inline uint32_t word(uint32_t pins, uint32_t delay_us) {
    return pins | (delay_us & 0x1FFFFFFFu);
}

// Idle state: CLK=HIGH, DAT=HIGH (= logic 0), STR=LOW
#define IDLE_PINS (PIN_CLK | PIN_DAT)

// Build the 26-word sequence for a 12-bit DAC write into buf[].
// Protocol: clock idles HIGH; falling edge transfers each bit; 2 words per bit.
//   Word A: CLK=LOW,  DAT=current_bit  → 43 µs (data stable, falling edge at start)
//   Word B: CLK=HIGH, DAT=next_bit     → 84 µs (rising edge at start, next bit setup)
// Data is inverted: logical 0 → GPIO HIGH, logical 1 → GPIO LOW.
static uint build_sequence(uint32_t *buf, uint16_t value) {
    uint n = 0;

    for (int bit = 11; bit >= 0; bit--) {
        // Data pin is inverted: bit=0 → PIN_DAT (HIGH), bit=1 → 0 (LOW)
        uint32_t dat = ((value >> bit) & 1u) ? 0u : PIN_DAT;

        // CLK LOW — falling edge, current bit stable
        buf[n++] = word(dat, T_CLK_LOW_US);

        // CLK HIGH — rising edge, set data for next bit simultaneously
        uint32_t next_dat;
        if (bit > 0)
            next_dat = ((value >> (bit-1)) & 1u) ? 0u : PIN_DAT;
        else
            next_dat = PIN_DAT;   // return data to idle (HIGH = logic 0)

        buf[n++] = word(PIN_CLK | next_dat, T_CLK_HIGH_US);
    }

    // Strobe pulse to latch the shift register into the DAC output register
    buf[n++] = word(PIN_CLK | PIN_DAT | PIN_STR, T_STROBE_US);
    // Return to full idle
    buf[n++] = word(IDLE_PINS, T_IDLE_US);

    return n;
}

void dac_init() {
    pio_offset = pio_add_program(DAC_PIO, &dac_sequencer_program);
    // PIO controls GPIO 20-22 (Data, Clock, Strobe); GPIO 19 (Enable) stays as GPIO.
    dac_sequencer_program_init(DAC_PIO, DAC_SM, pio_offset, PIN_OUT_HV_Data);

    // Prime the SM with one idle word so CLK and DAT start HIGH immediately.
    pio_sm_put_blocking(DAC_PIO, DAC_SM, word(IDLE_PINS, 0));

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
    // Build into the inactive buffer
    uint next = seq_active ^ 1u;
    uint count = build_sequence(seq_buf[next], value);
    seq_active = next;

    dma_channel_set_read_addr(dma_chan, seq_buf[seq_active], false);
    dma_channel_set_trans_count(dma_chan, count, true);  // true = start
}

bool dac_write_done() {
    return !dma_channel_is_busy(dma_chan);
}
