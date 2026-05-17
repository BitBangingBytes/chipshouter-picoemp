#pragma once

#include <stdbool.h>
#include <stdint.h>

// Initialize PIO state machine and DMA channel for AD7543 writes.
// Must be called after picoemp_init() (GPIO directions already set).
void dac_init();

// Write a 12-bit value to the AD7543 DAC (0–4095).
// Non-blocking: loads DMA transfer and returns immediately.
// Caller must not call again until dac_write_done() returns true.
void dac_write(uint16_t value);

// Returns true when the previous DMA transfer has completed.
bool dac_write_done();

// Drive STR HIGH (chip deselected). Used when HV is disabled. Blocks until
// the previous DMA transfer (if any) drains, then pushes a single PIO word.
void dac_deselect();
