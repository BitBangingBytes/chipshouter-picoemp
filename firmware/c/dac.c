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
#define PIN_DAT  (1u << 29)   // GPIO 20 = Data   (HIGH = logic 0, LOW = logic 1)
#define PIN_CLK  (1u << 30)   // GPIO 21 = Clock  (HIGH = idle, LOW = active)
#define PIN_STR  (1u << 31)   // GPIO 22 = Strobe (LOW = chip engaged while HV on; HIGH 40 µs to latch; HIGH continuous = HV off / deselected)

// Timing — delay values for the PIO word.
// Pin-state duration = delay_value + 4 µs (4-cycle instruction overhead per word).
// Empirically calibrated: delay=1 → 5 µs, overhead = 4 µs.
//
// Clock:  43 µs LOW  → delay = 43 - 4 = 39
//         84 µs HIGH total, split into two 42 µs halves → delay = 42 - 4 = 38 each
//           Half 1: hold current data stable after rising edge
//           Half 2: present next data bit (setup for next falling edge)
// Strobe: 40 µs STR HIGH after the L→H latch edge → delay = 40 - 4 = 36
#define T_CLK_LOW_US        39u   // → 43 µs
#define T_CLK_HIGH_HOLD_US  38u   // → 42 µs (data hold after rising edge)
#define T_CLK_HIGH_SETUP_US 38u   // → 42 µs (next data setup before falling edge)
#define T_STROBE_US         36u   // → 40 µs (STR HIGH hold after latch edge)
#define T_IDLE_US            0u   // →  4 µs trailing idle

// Maximum words: 1 (preamble) + 12 bits × 3 words + 2 (strobe + idle) = 39 → round to 40
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

// Idle between writes while HV is on: CLK=HIGH, DAT=HIGH (logic 0), STR=LOW
// (chip engaged, ready for the next sequence's preamble).
#define ENGAGED_IDLE_PINS  (PIN_CLK | PIN_DAT)
// Deselect (HV off): CLK=HIGH, DAT=HIGH, STR=HIGH.
#define DESELECT_PINS      (PIN_CLK | PIN_DAT | PIN_STR)

// Build the 39-word sequence for a 12-bit DAC write.
// Protocol: STR LOW throughout the data clocking, then a 40 µs HIGH strobe
// pulse at the end (the rising edge latches the shift register into the DAC
// output), then back LOW so the chip stays engaged for the next write. CLK
// idles HIGH, data captured on CLK rising edge. 3 words per bit:
//   Word A: CLK=LOW,  DAT=current bit          → 43 µs
//   Word B: CLK=HIGH, DAT=current bit (hold)   → 42 µs  ← CLK rising edge at start
//   Word C: CLK=HIGH, DAT=next bit    (setup)  → 42 µs  ← data changes here
// The preamble word sets up bit 11 on DAT (CLK still HIGH) for 42 µs before
// the first falling edge — bits 10..0 get the same setup window from the
// previous bit's Word C, but bit 11 has no predecessor. If STR was HIGH
// (deselected) at sequence start, the preamble also drops it LOW.
// Data is inverted: logic 0 → GPIO HIGH (PIN_DAT set), logic 1 → GPIO LOW.
static uint build_sequence(uint32_t *buf, uint16_t value) {
    uint n = 0;

    // Preamble: STR LOW (engage), setup bit 11 on DAT, CLK HIGH.
    uint32_t first_dat = ((value >> 11) & 1u) ? 0u : PIN_DAT;
    buf[n++] = word(PIN_CLK | first_dat, T_CLK_HIGH_SETUP_US);

    for (int bit = 11; bit >= 0; bit--) {
        uint32_t dat = ((value >> bit) & 1u) ? 0u : PIN_DAT;   // inverted

        uint32_t next_dat;
        if (bit > 0)
            next_dat = ((value >> (bit-1)) & 1u) ? 0u : PIN_DAT;
        else
            next_dat = PIN_DAT;   // DAT returns to idle (HIGH = logic 0) after last bit

        // A: CLK LOW — current bit stable, falling edge at start of this word
        buf[n++] = word(dat, T_CLK_LOW_US);

        // B: CLK HIGH — rising edge, hold current bit stable for 42 µs
        buf[n++] = word(PIN_CLK | dat, T_CLK_HIGH_HOLD_US);

        // C: CLK HIGH — data transitions to next bit, setup time 42 µs
        buf[n++] = word(PIN_CLK | next_dat, T_CLK_HIGH_SETUP_US);
    }

    // Strobe pulse: STR rising edge (LOW→HIGH) latches data; hold HIGH 40 µs.
    buf[n++] = word(DESELECT_PINS, T_STROBE_US);
    // Drop STR back LOW — chip remains engaged for the next write.
    buf[n++] = word(ENGAGED_IDLE_PINS, T_IDLE_US);

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

void dac_deselect() {
    // Drain any in-flight write so we don't fight the DMA for the TX FIFO,
    // then push a single PIO word that holds CLK/DAT/STR all HIGH.
    while (!dac_write_done()) tight_loop_contents();
    pio_sm_put_blocking(DAC_PIO, DAC_SM, word(DESELECT_PINS, T_IDLE_US));
}
