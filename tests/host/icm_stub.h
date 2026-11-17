// Register-level model of the ICM-42688-P for the host tests: five register
// banks behind REG_BANK_SEL, the power-on values the filter code depends on,
// a FIFO the test fills by hand, and a log of every register access.
#pragma once
#include <stdint.h>
#include <stddef.h>

struct StubAccess {
  bool write;
  uint8_t bank;    // Bank selected when the access happened
  uint8_t reg;
  uint8_t val;     // Written value, or value returned to the driver
  uint8_t pwr;     // PWR_MGMT0 at the time (0x00 = gyro + accel off)
};

void stub_reset();                       // Power-on state, empty log
size_t stub_log_size();
const StubAccess &stub_log(size_t i);
void stub_log_clear();
uint8_t stub_reg(uint8_t bank, uint8_t reg);
void stub_set_reg(uint8_t bank, uint8_t reg, uint8_t v);
void stub_set_readonly(uint8_t bank, uint8_t reg, bool ro);  // Writes are ignored
uint8_t stub_bank();                     // Bank currently selected
void stub_fifo_push(const uint8_t *bytes, size_t n);
void stub_set_lost(uint16_t lost);
