#include "dac.h"
#include "picoemp.h"
#include "dac_sequencer.pio.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"

// pio1 SM2: SM0=volt, SM1=curr (psu_monitor), SM2=dac_sequencer
#define DAC_PIO  pio1
#define DAC_SM   2u

// AD7543 timing (µs), all values chosen conservatively above datasheet minimums.
// Enable strobe pulse is 200 ns min; we use 1 µs resolution so minimum is 1.
#define T_SETUP_US   1u   // data setup before clock rise
#define T_CLOCK_US   1u   // clock high time
#define T_HOLD_US    1u   // data hold after clock fall
#define T_STROBE_US  2u   // strobe (load) pulse width

// 4-bit pin field encoding: [Enable, Data, Clock, Strobe] maps to bits [31..28]
// PIN_OUT_HV_Enable = GPIO 19 → bit 31 (MSB of the 4-bit field)
// PIN_OUT_HV_Data   = GPIO 20 → bit 30
// PIN_OUT_HV_Clock  = GPIO 21 → bit 29
// PIN_OUT_HV_Strobe = GPIO 22 → bit 28
#define PIN_EN   (1u << 31)
#define PIN_DAT  (1u << 30)
#define PIN_CLK  (1u << 29)
#define PIN_STR  (1u << 28)

// Maximum words per DAC write:
//   1 (enable assert) + 12*3 (bit clocks) + 2 (strobe) + 1 (idle) = 40
#define SEQ_MAX_WORDS 40u

static uint pio_offset;
static int  dma_chan;

// Double-buffer so we can build the next sequence while current DMA runs.
static uint32_t seq_buf[2][SEQ_MAX_WORDS];
static uint     seq_active = 0;

static inline uint32_t word(uint32_t pins, uint32_t delay_us) {
    return pins | (delay_us & 0x0FFFFFFFu);
}

// Build the complete 40-word sequence for a 12-bit DAC write into buf[].
// Returns word count written.
static uint build_sequence(uint32_t *buf, uint16_t value) {
    uint n = 0;

    // Assert Enable, all others low, hold 1 µs
    buf[n++] = word(PIN_EN, T_SETUP_US);

    // Clock out 12 bits MSB-first
    for (int bit = 11; bit >= 0; bit--) {
        uint32_t dat = (value >> bit) & 1u ? PIN_DAT : 0u;

        // Data + Enable, clock low, setup time
        buf[n++] = word(PIN_EN | dat, T_SETUP_US);
        // Clock high
        buf[n++] = word(PIN_EN | dat | PIN_CLK, T_CLOCK_US);
        // Clock low, hold time
        buf[n++] = word(PIN_EN | dat, T_HOLD_US);
    }

    // Deassert clock & data, assert Strobe to latch
    buf[n++] = word(PIN_EN | PIN_STR, T_STROBE_US);
    // Deassert strobe, keep Enable
    buf[n++] = word(PIN_EN, T_SETUP_US);
    // Deassert Enable — all pins idle low
    buf[n++] = word(0u, 1u);

    return n;
}

void dac_init() {
    pio_offset = pio_add_program(DAC_PIO, &dac_sequencer_program);
    dac_sequencer_program_init(DAC_PIO, DAC_SM, pio_offset, PIN_OUT_HV_Enable);

    dma_chan = dma_claim_unused_channel(true);

    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, pio_get_dreq(DAC_PIO, DAC_SM, true));
    dma_channel_configure(dma_chan, &cfg,
        &DAC_PIO->txf[DAC_SM],  // write to PIO TX FIFO
        seq_buf[0],              // initial read address (overridden in dac_write)
        SEQ_MAX_WORDS,           // max count (overridden in dac_write)
        false);                  // don't start yet
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
