// ===========================================================================
//  Board variant: SenseCAP T1000-E
// ---------------------------------------------------------------------------
//  Why this file exists:
//
//  The Adafruit nRF52 core translates every Arduino pin number through
//  g_ADigitalPinMap[] and silently discards anything at or above PINS_COUNT in
//  digitalWrite() and analogRead(). With a stock variant the raw port numbers
//  used by this project land on the wrong pins or nowhere at all.
//
//  This variant uses an identity map:
//        Arduino pin N  ==  physical GPIO N  ==  P0.N       (N <= 31)
//                                            ==  P1.(N-32)  (N >= 32)
//  which makes the pin header correct by construction.
// ===========================================================================

#ifndef _VARIANT_T1000E_
#define _VARIANT_T1000E_

#define VARIANT_MCK       (64000000ul)

// Low frequency clock source for the SoftDevice.
// Measured on the device, the bootloader had already started the internal RC
// oscillator, so whether a 32.768 kHz crystal is fitted was never established.
// The risk is asymmetric: with RC the radio always works and only costs a
// little quiescent current, whereas selecting the crystal when none is fitted
// prevents the SoftDevice from starting at all. Prove the crystal first, then
// switch for the power saving.
// #define USE_LFXO
#define USE_LFRC

#include "WVariant.h"

#ifdef __cplusplus
extern "C" {
#endif

// P0.00 .. P1.15, 48 pins, identity map
#define PINS_COUNT           (48)
#define NUM_DIGITAL_PINS     (48)
#define NUM_ANALOG_INPUTS    (8)
#define NUM_ANALOG_OUTPUTS   (0)

// --- LEDs -------------------------------------------------------------------
#define PIN_LED1             (24)   // P0.24  green
#define PIN_LED2             (3)    // P0.03  red

#define LED_BUILTIN          PIN_LED1
#define LED_CONN             PIN_LED2

#define LED_GREEN            PIN_LED1
#define LED_RED              PIN_LED2

// Bluefruit requires LED_BLUE for its connection indicator. This board only
// has green and red, so the red one shows the link state.
#define LED_BLUE             PIN_LED2

// If the LEDs turn out to be active low, set this to 0.
#define LED_STATE_ON         1

// --- Button -----------------------------------------------------------------
#define PIN_BUTTON1          (6)    // P0.06, active high

// --- Analog -----------------------------------------------------------------
// analogRead() maps through the table and then switches on the physical pin
// number, so with an identity map the ADC channels line up automatically.
#define PIN_A0               (2)    // P0.02  AIN0  battery
#define PIN_A1               (4)    // P0.04  AIN2  VCC
#define PIN_A2               (29)   // P0.29  AIN5  ambient light
#define PIN_A3               (31)   // P0.31  AIN7  temperature

static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;

#define ADC_RESOLUTION       14

#define PIN_VBAT             PIN_A0

// --- UART: GNSS -------------------------------------------------------------
#define PIN_SERIAL1_RX       (13)   // P0.13
#define PIN_SERIAL1_TX       (14)   // P0.14

// --- SPI: LR1110 ------------------------------------------------------------
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (40)   // P1.08
#define PIN_SPI_MOSI         (41)   // P1.09
#define PIN_SPI_SCK          (11)   // P0.11

static const uint8_t SS   = (12);   // P0.12
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

// --- I2C --------------------------------------------------------------------
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (26)   // P0.26
#define PIN_WIRE_SCL         (27)   // P0.27

// --- QSPI flash -------------------------------------------------------------
// Pins are documented but the external flash is not enabled; the device type
// has not been identified.
#define PIN_QSPI_SCK         (19)   // P0.19
#define PIN_QSPI_CS          (20)   // P0.20
#define PIN_QSPI_IO0         (21)   // P0.21  MOSI
#define PIN_QSPI_IO1         (22)   // P0.22  MISO
#define PIN_QSPI_IO2         (23)   // P0.23  WP
#define PIN_QSPI_IO3         (32)   // P1.00  HOLD

#ifdef __cplusplus
}
#endif

#endif // _VARIANT_T1000E_
