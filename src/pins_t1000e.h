#ifndef PINS_T1000E_H
#define PINS_T1000E_H

// ---------------------------------------------------------------------------
// SenseCAP T1000-E pin assignments, established on the device.
//
// Arduino pin number (Adafruit nRF52 core) = 32 * port + bit
//   P0.xx -> xx        P1.xx -> 32 + xx
// ---------------------------------------------------------------------------

// --- Semtech LR1110, SPI ---------------------------------------------------
#define PIN_LR_NSS      12   // P0.12  chip select
#define PIN_LR_SCK      11   // P0.11
#define PIN_LR_MISO     40   // P1.08
#define PIN_LR_MOSI     41   // P1.09
#define PIN_LR_RST      42   // P1.10
#define PIN_LR_BUSY      7   // P0.07
#define PIN_LR_IRQ      33   // P1.01  DIO9
// The LR1110 uses a TCXO powered from DIO3.

// --- GNSS AG3335, UART ------------------------------------------------------
// Verified on the device: NMEA arrives on P0.14.
#define PIN_GNSS_MCU_RX 14   // P0.14  GNSS -> MCU
#define PIN_GNSS_MCU_TX 13   // P0.13  MCU  -> GNSS
#define PIN_GNSS_EN     43   // P1.11  enable          -> high
#define PIN_GNSS_RST    47   // P1.15  reset, idle low -> low
#define PIN_GNSS_VRTC    8   // P0.08  backup domain  -> high
#define PIN_GNSS_RINT   15   // P0.15                  -> low
#define PIN_GNSS_SINT   44   // P1.12                  -> high
#define PIN_GNSS_RESETB 46   // P1.14  driven by the GNSS -> input, pull-up
#define GNSS_BAUD    115200

// --- Controls and indicators ------------------------------------------------
#define PIN_BUTTON       6   // P0.06  active high
#define PIN_LED_GREEN   24   // P0.24
#define PIN_LED_RED      3   // P0.03
#define PIN_BUZZER      25   // P0.25
#define PIN_BUZZER_EN   37   // P1.05  must be high or the buzzer is silent

// --- I2C: QMA6100P accelerometer --------------------------------------------
#define PIN_I2C_SDA     26   // P0.26
#define PIN_I2C_SCL     27   // P0.27
#define QMA6100P_ADDR   0x12 // AD0 tied low
#define QMA6100P_CHIPID 0x90 // register 0x00

// --- Sensors and power ------------------------------------------------------
// The sensor supply feeds the thermistor, the light sensor and the battery
// divider through its own regulator. It is only needed while one of them is
// being sampled and draws several hundred microamps for as long as it is on.
// The button does not depend on it.
#define PIN_SENSOR_EN   38   // P1.06  sensor supply, on only while sampling
#define PIN_ACC_EN      39   // P1.07  accelerometer supply
#define PIN_ACC_INT     34   // P1.02  motion interrupt, unused
#define PIN_BAT_ADC      2   // P0.02  AIN0, 1:2 divider
#define PIN_VCC_ADC      4   // P0.04  AIN2, sensor supply voltage
#define PIN_TEMP_ADC    31   // P0.31  AIN7
#define PIN_LUX_ADC     29   // P0.29  ambient light

// --- QSPI flash, not used by this firmware ----------------------------------
#define PIN_FLASH_EN    45   // P1.13  supply enable, active high -> low

// --- Charger ----------------------------------------------------------------
#define PIN_CHG_PWR      5   // P0.05
#define PIN_CHG_STATUS  35   // P1.03
#define PIN_CHG_DONE    36   // P1.04

#endif // PINS_T1000E_H
