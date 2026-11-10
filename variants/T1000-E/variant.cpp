// ===========================================================================
//  Board variant: SenseCAP T1000-E, identity pin map.
//  Arduino pin N == physical GPIO N == P0.N (N<=31) / P1.(N-32) (N>=32).
//  See variant.h for why this is necessary.
// ===========================================================================

#include "variant.h"
#include "wiring_constants.h"
#include "wiring_digital.h"
#include "nrf.h"

const uint32_t g_ADigitalPinMap[] = {
  // ---- P0.00 .. P0.31 ----
   0,  1,  2,  3,  4,  5,  6,  7,
   8,  9, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26, 27, 28, 29, 30, 31,
  // ---- P1.00 .. P1.15 ----
  32, 33, 34, 35, 36, 37, 38, 39,
  40, 41, 42, 43, 44, 45, 46, 47,
};

// P0.00 and P0.01 (crystal) and P0.18 (reset) appear in the table for
// completeness but must not be used.

void initVariant()
{
  // Put the LEDs into a defined state. Everything else, including the power
  // rails and the buzzer, is handled explicitly by the application.
  pinMode(PIN_LED1, OUTPUT);
  ledOff(PIN_LED1);

  pinMode(PIN_LED2, OUTPUT);
  ledOff(PIN_LED2);
}
