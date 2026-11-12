// BMP580 configuration against the simulated part, which, like the real one,
// ignores a DSP_IIR / DSP_CONFIG write outside STANDBY.
//
//   make baro && ./baro
//
// The IIR coefficients belong in DSP_IIR (0x31), not in OOR_CONFIG (0x35),
// and a non-bypass IIR needs deep standby disabled (ODR_CONFIG bit 7).
#include "Arduino.h"
#include "TriSense.h"

void sim_reset(int motion, uint32_t seed);
uint8_t  sim_bmp_reg(uint8_t reg);
uint32_t sim_bmp_ignored(void);

static int fails = 0;
static void check(bool ok, const char *what) {
  printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) fails++;
}

int main() {
  sim_reset(0, 1);
  TriSense sensor;
  if (!sensor.beginAll(MODE_HYBRID, 17, 10000000)) { check(false, "beginAll"); return 1; }

  // The order a sketch typically uses, with the part already in NORMAL mode.
  sensor.bmp.setOversampling(BMP580_OSR_x2, BMP580_OSR_x1);
  sensor.bmp.setIIRFilter(BMP580_IIR_3, BMP580_IIR_OFF);
  sensor.bmp.setODR(BMP580_ODR_240Hz);

  const uint8_t iir = sim_bmp_reg(0x31), oor = sim_bmp_reg(0x35), odr = sim_bmp_reg(0x37);
  printf("     DSP_IIR 0x%02X  OOR_CONFIG 0x%02X  ODR_CONFIG 0x%02X  ignored writes %u\n",
         iir, oor, odr, (unsigned)sim_bmp_ignored());
  check(iir == 0x10, "IIR_3 on pressure, off on temperature, in DSP_IIR");
  check(oor == 0x00, "OOR_CONFIG untouched");
  check((sim_bmp_reg(0x30) & 0x78) == 0x60, "filtered pressure routed to data registers and FIFO");
  check(sim_bmp_ignored() == 0, "every DSP_IIR / DSP_CONFIG write made in STANDBY");
  check((odr & 0x03) == BMP580_MODE_NORMAL, "back in NORMAL mode");
  check((odr & 0x80) != 0, "deep standby disabled for the IIR");
  check(((odr >> 2) & 0x1F) == BMP580_ODR_240Hz, "ODR kept");

  sensor.bmp.setIIRFilter(BMP580_IIR_OFF, BMP580_IIR_OFF);
  check(sim_bmp_reg(0x31) == 0x00 && (sim_bmp_reg(0x37) & 0x03) == BMP580_MODE_NORMAL,
        "IIR off again, still NORMAL");

  printf(fails ? "%d FAILED\n" : "ALL PASSED\n", fails);
  return fails ? 1 : 0;
}
