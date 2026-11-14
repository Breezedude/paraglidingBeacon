// ===========================================================================
//  Breezedude Beacon for the SenseCAP T1000-E
// ---------------------------------------------------------------------------
//  Transmits FANET, FLARM and ADS-L position reports, streams NMEA over
//  Bluetooth LE and is configured entirely from a browser. The USB serial
//  port is diagnostic output only.
// ===========================================================================

#include <algorithm>
#include <Arduino.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <bluefruit.h>
#include <nrf_gpio.h>

#include "accel.h"
#include "battery.h"
#include "ble_link.h"
#include "config.h"
#include "flight_state.h"
#include "gnss_filter.h"
#include "rx_cache.h"
#include "pins_t1000e.h"
#include "proto/ads_l.h"
#include "proto/bits.h"
#include "proto/fanet.h"
#include "proto/flarm.h"
#include "proto/nmea.h"
#include "proto/report.h"
#include "radio.h"
#include "slot_clock.h"

namespace {

// The protocol layer keeps its own namespace; these are the pieces this file
// works with directly.
namespace nmea = proto::nmea;
using proto::AircraftType;
using proto::Report;
using proto::ReportKind;
using proto::Source;
using proto::Traffic;
using proto::Weather;

Radio       radio;
TinyGPSPlus gps;

uint32_t lastTrackingMs = 0;
uint32_t lastNameMs     = 0;
uint32_t lastStatusMs   = 0;
uint32_t lastMotionMs   = 0;
uint32_t txFlashMs      = 0;

// Where in the UTC second we are, estimated from the arrival of the GNSS
// report, and one timer per transmission that has to hit a slot.
timing::SecondClock utcClock;
timing::SlotTimer   fanetSlot;
timing::SlotTimer   flarmSlot;
timing::SlotTimer   adslSlot;

// ADS-L alternates between its two channels from one frame to the next.
bool adslUpperChannel = false;

// Last idle (no-motion) tracking emission per protocol; see idleTrackingDue().
uint32_t lastIdleFlarmMs = 0;
uint32_t lastIdleAdslMs  = 0;

bool shuttingDown = false;
bool wakeConfirmed = false;

// Everything transmitted comes from the filter rather than straight from the
// parser, so a jump or an altitude spike never reaches the air. Climb rate is
// derived from the smoothed altitude; the board has no barometer.
gnss::Filter gnssFilter;

// Whether the device is walking or flying. FANET states this outright with two
// different frame types and ADS-L with a flag, so it has to be decided rather
// than guessed separately in each of them.
flight::Detector flightState;

// A fix older than this is stale. The receiver reports once a second, so this
// tolerates a few missed messages before the fix counts as lost.
constexpr uint32_t kFixMaxAgeMs = 4000;

// A fix that drops out for a second or two is normal and says nothing worth
// interrupting a pilot for. Only a loss that lasts is announced, and only such
// a loss makes the return worth announcing either.
constexpr uint32_t kFixLossGraceMs = 10000;

// Assembles complete NMEA sentences so they can be forwarded intact.
char    nmeaLine[100];
uint8_t nmeaLen = 0;

uint8_t lowVoltageStrikes = 0;
bool    lastFrameWasV7    = false;

// Satellites in view per constellation, parsed from GSV. GGA only reports the
// satellites used in the fix, which is 0 while the receiver is acquiring.
uint8_t gsvViewSats[4] = {0, 0, 0, 0};     // GP, GL, GA, BD
bool    gsvViewSeen[4] = {false, false, false, false};

// ---------------------------------------------------------------------------
//  Buzzer
// ---------------------------------------------------------------------------
//  tone() offers no duty cycle control, so the PWM peripheral is driven
//  directly. The piezo sits on a fixed pin, so this is not a true volume
//  control, but the duty cycle does grade the level usefully. Fifty percent is
//  loudest; the perceived scale is far from linear because the element is
//  resonant.
constexpr uint32_t kBuzzerToken = 0xB0BB1E00;

void buzzerInit() {
  HwPWM2.takeOwnership(kBuzzerToken);
  HwPWM2.addPin(PIN_BUZZER);
}

void toneOn(uint16_t frequency, uint8_t volume) {
  HwPWM2.stop();
  HwPWM2.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_16);   // 16 MHz / 16 = 1 MHz
  uint32_t top = 1000000UL / frequency;
  if (top > 32767) top = 32767;
  HwPWM2.setMaxValue(uint16_t(top));
  HwPWM2.writePin(PIN_BUZZER, uint16_t((top * volume) / 200));
  HwPWM2.begin();
}

void toneOff() {
  HwPWM2.stop();
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
}

void beepAt(uint16_t frequency, uint16_t ms, uint8_t volume) {
  if (frequency == 0 || volume == 0) { delay(ms); return; }
  toneOn(frequency, volume);
  delay(ms);
  toneOff();
  delay(20);
}

void beep(uint16_t frequency, uint16_t ms) {
  beepAt(frequency, ms, cfg.buzzerVol);
}

bool buttonPressed() { return digitalRead(PIN_BUTTON) == HIGH; }

// ---------------------------------------------------------------------------
//  LED effects
// ---------------------------------------------------------------------------
//  Software PWM at roughly 260 Hz. The hardware PWM block belongs to the
//  buzzer, and this way the LED stays a plain output afterwards.
//  Returns false if `requireButton` is set and the button was released early.
bool fadeGreen(uint8_t from, uint8_t to, uint16_t ms, bool requireButton) {
  const uint32_t start = millis();
  for (;;) {
    const uint32_t elapsed = millis() - start;
    if (elapsed >= ms) break;
    if (requireButton && !buttonPressed()) return false;

    int duty = int(from) + (int(to) - int(from)) * int32_t(elapsed) / int32_t(ms);
    duty = constrain(duty, 0, 255);

    digitalWrite(PIN_LED_RED, ((elapsed % 160) < 40) ? HIGH : LOW);
    if (duty)       { digitalWrite(PIN_LED_GREEN, HIGH); delayMicroseconds(duty * 15); }
    if (duty < 255) { digitalWrite(PIN_LED_GREEN, LOW);  delayMicroseconds((255 - duty) * 15); }
  }
  digitalWrite(PIN_LED_RED, LOW);
  return true;
}

void melodyFix()   { beep(1175, 90); beep(1568, 180); }
// The same two notes the other way round, so losing the fix is recognisable
// as the opposite of gaining it without having to be learned separately.
void melodyFixLost() { beep(1568, 90); beep(1175, 220); }
// Deliberately unlike any of the others: a device asking for permission should
// not be mistaken for one reporting a fix.
void melodyAskPairing() {
  for (uint8_t i = 0; i < 2; ++i) { beep(1319, 70); delay(60); beep(988, 70); delay(140); }
}
void melodyPaired() { beep(988, 70); beep(1319, 70); beep(1760, 160); }
void melodyOn()    { beep(880, 80); beep(1175, 80); beep(1568, 150); }
// A cable plugged into or pulled out of a device that is switched off. Short,
// because nothing else happens: up for in, down for out.
void melodyPlugged()   { beep(1047, 60); beep(1319, 90); }
void melodyUnplugged() { beep(1319, 60); beep(1047, 90); }
void melodyRefuse(){ beep(300, 260); delay(70); beep(240, 420); }

void melodyLowBattery() {
  const uint8_t v = cfg.buzzerVol;
  beep(587, 220);
  beep(523, 220);
  beep(466, 260);
  // A falling slide, which needs the tone to keep running while the frequency
  // changes rather than a sequence of separate beeps.
  for (uint16_t f = 440; f > 175; f -= 11) { toneOn(f, v); delay(24); }
  toneOff();
  delay(120);
  for (uint16_t f = 196; f > 147; f -= 4) { toneOn(f, v); delay(30); }
  toneOff();
}

void shutdownIndication() {
  shuttingDown = true;
  constexpr uint16_t notes[3] = {1568, 1175, 784};
  constexpr uint16_t spans[3] = {160, 160, 320};
  constexpr uint16_t total    = 640;

  uint16_t done = 0;
  for (int i = 0; i < 3; ++i) {
    toneOn(notes[i], cfg.buzzerVol);
    const uint8_t from = uint8_t(255 - uint32_t(done) * 255 / total);
    done = uint16_t(done + spans[i]);
    const uint8_t to = uint8_t(255 - uint32_t(done) * 255 / total);
    fadeGreen(from, to, spans[i], false);
  }
  toneOff();
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);
}

// ---------------------------------------------------------------------------
//  Pins
// ---------------------------------------------------------------------------
//  The level is written before the pin becomes an output. The core sets every
//  output latch high on start-up, so the other way round a supply enable would
//  pulse on for a moment each time the device wakes.
void driveOutput(uint8_t pin, uint8_t level) {
  digitalWrite(pin, level);
  pinMode(pin, OUTPUT);
}

// Hands a line to one of the internal pull resistors. Used for everything that
// has to hold a level while the device is off: a pulled input stays as it is
// through the wake-up reset, where a driven output would be forced high by the
// core before setup() gets to it, and the resistor bounds what can flow into a
// chip that has no supply.
void holdWithPull(uint8_t pin, bool high) {
  nrf_gpio_cfg_input(g_ADigitalPinMap[pin],
                     high ? NRF_GPIO_PIN_PULLUP : NRF_GPIO_PIN_PULLDOWN);
}

// Input buffer disconnected: draws nothing whatever the line does.
void releasePin(uint8_t pin) { nrf_gpio_cfg_default(g_ADigitalPinMap[pin]); }

// Everything the board needs before any peripheral is started. The GNSS
// receiver is left exactly as it was while the device was off, with only its
// backup supply on, so that a press that does not lead to a start does not
// disturb it.
void boardInit() {
  driveOutput(PIN_SENSOR_EN, LOW);
  driveOutput(PIN_FLASH_EN, LOW);
  driveOutput(PIN_ACC_EN, HIGH);
  driveOutput(PIN_BUZZER_EN, HIGH);
  driveOutput(PIN_BUZZER, LOW);
  driveOutput(PIN_LED_GREEN, LOW);
  driveOutput(PIN_LED_RED, LOW);
  pinMode(PIN_BUTTON, INPUT_PULLDOWN);         // the button is active high

  driveOutput(PIN_GNSS_EN, LOW);
  driveOutput(PIN_GNSS_RST, LOW);
  driveOutput(PIN_GNSS_RINT, LOW);
  driveOutput(PIN_GNSS_SINT, HIGH);
  driveOutput(PIN_GNSS_VRTC, HIGH);
}

// ---------------------------------------------------------------------------
//  Watchdog
// ---------------------------------------------------------------------------
//  Nothing else gets the device out of a hang. The button is read by the same
//  loop that would be stuck, and neither the button nor plugging in USB resets
//  the chip unless it is in system-off, so a stuck device looks dead while it
//  empties the battery.
//
//  The watchdog stops by itself in system-off and the bootloader feeds it, so
//  it gets in the way of neither. It cannot be stopped or reconfigured once it
//  runs and it survives a soft reset, which is why it is only set up when it
//  is not running yet.
constexpr uint32_t kWatchdogSeconds = 60;

void watchdogFeed() {
  // All eight reload registers, in case other firmware started it with a
  // different one enabled before this one was installed.
  for (uint8_t i = 0; i < 8; ++i) NRF_WDT->RR[i] = WDT_RR_RR_Reload;
}

void watchdogStart() {
  if (!(NRF_WDT->RUNSTATUS & WDT_RUNSTATUS_RUNSTATUS_Msk)) {
    NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) |
                      (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);
    NRF_WDT->CRV    = kWatchdogSeconds * 32768UL - 1UL;
    NRF_WDT->RREN   = WDT_RREN_RR0_Msk;
    NRF_WDT->TASKS_START = 1;
  }
  watchdogFeed();
}

// ---------------------------------------------------------------------------
//  Battery
// ---------------------------------------------------------------------------
//  The divider is fed from the sensor supply, which the board's own firmware
//  switches on for the duration of a measurement and off again. Left on it
//  also powers the thermistor and the light sensor, several hundred microamps
//  for nothing. The voltage is therefore sampled every few seconds and
//  everything else reads the stored value.
constexpr uint32_t kBatteryPeriodMs = 10000;
constexpr uint32_t kSensorSettleMs  = 30;

float    batteryVolts     = 0.0f;
bool     batteryValid     = false;
uint32_t batterySampledMs = 0;
uint32_t sensorOnSinceMs  = 0;      // non-zero while the supply is settling

// Requires the sensor supply to be on and settled.
float readBatteryPin() {
  analogReadResolution(12);
  analogReference(AR_INTERNAL_3_0);
  analogSampleTime(40);              // the divider is a high impedance source
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 4; ++i) sum += analogRead(PIN_BAT_ADC);
  return (float(sum) / 4.0f * 3.0f / 4096.0f) * 2.0f;      // 1:2 divider
}

// Blocking variant for start-up, where the answer decides what happens next.
float measureBattery() {
  digitalWrite(PIN_SENSOR_EN, HIGH);
  delay(kSensorSettleMs);
  batteryVolts = readBatteryPin();
  digitalWrite(PIN_SENSOR_EN, LOW);
  batterySampledMs = millis();
  batteryValid     = true;
  sensorOnSinceMs  = 0;
  return batteryVolts;
}

// Non-blocking variant for the main loop.
void batteryTick() {
  const uint32_t now = millis();
  if (sensorOnSinceMs) {
    if (now - sensorOnSinceMs < kSensorSettleMs) return;
    batteryVolts = readBatteryPin();
    digitalWrite(PIN_SENSOR_EN, LOW);
    batterySampledMs = now;
    batteryValid     = true;
    sensorOnSinceMs  = 0;
  } else if (!batteryValid || now - batterySampledMs >= kBatteryPeriodMs) {
    digitalWrite(PIN_SENSOR_EN, HIGH);
    sensorOnSinceMs = now ? now : 1;
  }
}

uint8_t batteryPercent(float volts) {
  return battery::percentFromMillivolts(uint16_t(volts * 1000.0f + 0.5f));
}

// USB presence straight from the hardware register, which avoids guessing the
// polarity of the charger status pins.
bool usbPresent() {
  return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

bool batteryBelowCutoff() {
  if (cfg.lowVoltMv == 0 || !batteryValid) return false;
  if (usbPresent()) return false;            // charging, do not interfere
  return batteryVolts * 1000.0f < float(cfg.lowVoltMv);
}

// ---------------------------------------------------------------------------
//  Power off
// ---------------------------------------------------------------------------
//  True system-off draws a few microamps. It is left through the button, by the
//  GPIO detect logic, or by a cable being plugged in, which the chip detects
//  on its own. Either way waking is a reset, so setup() runs again.
//
//  What the rest of the board draws meanwhile depends entirely on the state
//  its lines are left in, and the processor being off does not help with that:
//  pin configuration is retained. Every line that reaches a part without
//  supply is a path into it through that part's protection diodes. The states
//  below follow the board manufacturer's own firmware and the other firmware
//  known to run on it.

//  Left in the retained register when a power-off did not take effect, so the
//  next start can do it again before the SoftDevice is up. Deliberately none
//  of the values the bootloader looks for, which it passes through untouched.
constexpr uint32_t kPowerOffRetryMagic = 0x5A;

//  Tells the bootloader to start the application without looking for a reason
//  to stay in update mode first.
constexpr uint32_t kBootloaderSkipMagic = 0x6D;

void gnssStop(bool keepBackup);              // defined with the rest of GNSS

bool i2cUp = false;

void waitForButtonRelease() {
  const uint32_t start = millis();
  while (buttonPressed() && millis() - start < 15000) delay(10);
  delay(150);
  while (buttonPressed() && millis() - start < 15000) delay(10);
}

// Puts every part of the board into the state it should have while off.
void shutDownBoard() {
  gnssStop(cfg.gnssVrtc);
  radio.shutdown();

  // Accelerometer. Starting the bus enabled the pull-ups on both of its
  // lines, and with the sensor unpowered they would go on feeding it through
  // its inputs, so the lines are let go before the supply.
  if (i2cUp) Wire.end();
  i2cUp = false;
  releasePin(PIN_I2C_SDA);
  releasePin(PIN_I2C_SCL);
  holdWithPull(PIN_ACC_EN, false);

  holdWithPull(PIN_SENSOR_EN, false);
  holdWithPull(PIN_FLASH_EN, false);

  HwPWM2.stop();
  holdWithPull(PIN_BUZZER, false);
  holdWithPull(PIN_BUZZER_EN, false);
  holdWithPull(PIN_LED_GREEN, false);
  holdWithPull(PIN_LED_RED, false);
}

void armButtonWake() {
  NRF_P0->LATCH = 0xFFFFFFFF;                // clear stale detect state
  NRF_P1->LATCH = 0xFFFFFFFF;
  nrf_gpio_cfg_sense_input(g_ADigitalPinMap[PIN_BUTTON],
                           NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
}

//  Powering off must not fail quietly. A device left in system-on with the CPU
//  parked cannot be reached by the button, because GPIO sense only resets the
//  chip out of system-off, nor by plugging in USB, because VBUS does the same.
//  It looks dead and only a forced reset brings it back. Every path here
//  therefore ends in system-off or in a reset.
void powerOffNow() {
  uint8_t sdEnabled = 0;
  (void)sd_softdevice_is_enabled(&sdEnabled);

  if (sdEnabled) {
    sd_power_gpregret_clr(0, 0xFF);
    sd_power_gpregret_set(0, kBootloaderSkipMagic);

    // Documented to return only when the chip did not power off.
    sd_power_system_off();

    // The POWER peripheral belongs to the SoftDevice while it runs, so writing
    // SYSTEMOFF straight into the register is ignored. Take it back first.
    sd_softdevice_disable();
  }

  NRF_POWER->GPREGRET  = kBootloaderSkipMagic;
  NRF_POWER->SYSTEMOFF = 1;
  __DSB();
  delay(50);

  // Still running. Ask the next start to power off before the SoftDevice comes
  // up, where the write cannot be refused, and reset into it. A device that
  // reboots is recoverable; one parked in system-on is not.
  NRF_POWER->GPREGRET = kPowerOffRetryMagic;
  NVIC_SystemReset();
}

void enterSystemOff() {
  watchdogFeed();

  // The session ends here: persist whatever the client changed, so an
  // ungraceful disconnect cannot lose the configuration.
  cfgSaveIfDirty();

  // Off is a state to return to. Whatever restarts the chip next, a cable
  // going in or the supply dipping as one comes out, must not switch the
  // device on by itself.
  cfgSetPowerIntent(false);

  // Wait for release first. The wake condition is a level, not an edge: going
  // to sleep with the button still down satisfies it immediately and the
  // device wakes straight back up.
  waitForButtonRelease();

  shutDownBoard();

  // Shutting the receiver down properly takes over a second, long enough for
  // the button to have been pressed again.
  waitForButtonRelease();
  watchdogFeed();

  // With a cable in, the device does not go into system-off. Pulling the
  // cable out has to be noticed, to acknowledge it, and system-off does not
  // notice it. Restarting goes to the charging state instead, see
  // chargingIdle(), with every part already switched off.
  if (usbPresent()) NVIC_SystemReset();

  armButtonWake();
  powerOffNow();                             // never returns
}

// Second attempt for a power-off that did not take effect, run early in
// setup() while the SoftDevice is still down. If even this is refused, which
// in practice means a debugger is attached, starting up normally is the
// recoverable outcome and parking here is not.
void retryPowerOff() {
  waitForButtonRelease();
  shutDownBoard();
  waitForButtonRelease();
  armButtonWake();

  NRF_POWER->GPREGRET  = kBootloaderSkipMagic;
  NRF_POWER->SYSTEMOFF = 1;
  __DSB();
  delay(100);

  NRF_POWER->GPREGRET = 0;
  boardInit();
}

bool waitForButton(bool wantPressed, uint32_t timeoutMs) {
  const uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (buttonPressed() == wantPressed) return true;
    delay(10);
  }
  return false;
}

// Two stage wake, so the device cannot switch itself on in a pocket:
// the press that started it, a release, then a deliberate hold. Returns false,
// with buzzer and LEDs off again, when the gesture was not completed.
bool wakeGesture() {
  beep(1200, 100);

  if (!waitForButton(false, 2000)) return false;
  delay(50);
  if (!waitForButton(true, 800)) return false;

  constexpr uint16_t notes[3] = {880, 1175, 1568};
  constexpr uint16_t hold     = 510;

  for (int i = 0; i < 3; ++i) {
    toneOn(notes[i], cfg.buzzerVol);
    const uint8_t from = uint8_t(uint32_t(i) * 255 / 3);
    const uint8_t to   = uint8_t(uint32_t(i + 1) * 255 / 3);
    if (!fadeGreen(from, to, hold / 3, true)) {
      toneOff();
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, LOW);
      return false;
    }
  }
  toneOff();
  digitalWrite(PIN_LED_GREEN, HIGH);
  wakeConfirmed = true;
  return true;
}

void confirmWake() {
  if (!wakeGesture()) enterSystemOff();
}

// ---------------------------------------------------------------------------
//  Charging while switched off
// ---------------------------------------------------------------------------
//  The battery charges by itself. The firmware stays out of the way and
//  watches for two things: the cable coming out, which ends in system-off,
//  and the wake gesture, which switches the device on as if it had not been
//  charging. Nothing else is started, no GNSS, radio or Bluetooth, and the
//  LEDs stay dark. The processor is awake for this, on USB power.
constexpr uint32_t kUnplugSettleMs = 300;

void chargingIdle(bool announce) {
  if (announce) melodyPlugged();

  bool     unplugged = false;
  uint32_t unpluggedMs = 0;
  for (;;) {
    watchdogFeed();

    if (usbPresent()) {
      unplugged = false;
    } else if (!unplugged) {
      unplugged   = true;
      unpluggedMs = millis();
    } else if (millis() - unpluggedMs >= kUnplugSettleMs) {
      melodyUnplugged();
      enterSystemOff();                      // never returns: the cable is out
    }

    if (buttonPressed()) {
      if (wakeGesture()) return;
      waitForButtonRelease();
    }
    delay(20);
  }
}

// ---------------------------------------------------------------------------
//  GNSS
// ---------------------------------------------------------------------------
//  The AG3335 is an Airoha part and takes the $PAIR command set. It tolerates
//  some of the older MediaTek $PMTK sentences but not all of them, and a
//  command it does not understand is silently ignored.
//
//  The sequence follows the driver in the board manufacturer's own firmware:
//
//    start   main supply on, a short pulse on the RTC interrupt line to bring
//            the receiver out of its backup mode, then the command that stops
//            it from dozing between fixes, repeated for a second because the
//            first ones are lost while it is still asleep
//    stop    allow sleep again, send it into backup mode, and only then take
//            the main supply away
//
//  Backup mode is what makes the next fix quick: the receiver keeps its clock
//  and what it knows about the satellites on the backup supply. Cutting the
//  main supply without asking for it first leaves that to chance.

// Command with a NMEA checksum, so a corrupted line is rejected instead of
// half-executed.
void gnssSend(const char *sentence) {
  uint8_t sum = 0;
  for (const char *p = sentence + 1; *p; ++p) sum ^= uint8_t(*p);
  char buf[56];
  snprintf(buf, sizeof(buf), "%s*%02X\r\n", sentence, sum);
  Serial1.print(buf);
}

// Nothing here is saved to the receiver's own memory ($PAIR513): the settings
// are applied on every start, so the module is in a known state regardless of
// what any previous firmware left behind.
const char *const kGnssSetup[] = {
    // Four constellations rather than GPS alone. More satellites in view is
    // what makes the difference to time to first fix and to holding a fix
    // under a wing.     GPS GLO GAL BDS QZSS NAVIC
    "$PAIR066,1,1,1,1,0,0",

    // Only the sentences that are actually read: position, and the satellites
    // in view for the display and the LED.
    "$PAIR062,0,1",   // GGA on
    "$PAIR062,1,0",   // GLL off
    "$PAIR062,2,0",   // GSA off
    "$PAIR062,3,1",   // GSV on
    "$PAIR062,4,1",   // RMC on
    "$PAIR062,5,0",   // VTG off
    "$PAIR062,6,0",   // ZDA off
};
constexpr uint8_t kGnssSetupCount = sizeof(kGnssSetup) / sizeof(kGnssSetup[0]);

constexpr uint8_t  kGnssLockRepeats  = 25;
constexpr uint32_t kGnssCommandGapMs = 40;

// Bringing the receiver up takes about two seconds of mostly waiting, so it
// is stepped from the main loop instead of holding up the start.
enum class GnssPhase : uint8_t { Off, PowerUp, WakePulse, Settle, LockSleep, Setup, Running };

GnssPhase gnssPhase   = GnssPhase::Off;
uint32_t  gnssPhaseMs = 0;
uint8_t   gnssStep    = 0;
bool      gnssUartUp  = false;

void gnssEnter(GnssPhase phase) {
  gnssPhase   = phase;
  gnssPhaseMs = millis();
  gnssStep    = 0;
}

void gnssStart() {
  driveOutput(PIN_GNSS_VRTC, HIGH);
  driveOutput(PIN_GNSS_RST, LOW);            // high holds the receiver in reset
  driveOutput(PIN_GNSS_SINT, HIGH);
  driveOutput(PIN_GNSS_RINT, LOW);
  // Driven by the receiver. Left floating it picks up noise.
  pinMode(PIN_GNSS_RESETB, INPUT_PULLUP);

  Serial1.setPins(PIN_GNSS_MCU_RX, PIN_GNSS_MCU_TX);
  Serial1.begin(GNSS_BAUD);
  gnssUartUp = true;

  driveOutput(PIN_GNSS_EN, HIGH);
  gnssEnter(GnssPhase::PowerUp);
}

void gnssTick() {
  const uint32_t elapsed = millis() - gnssPhaseMs;

  switch (gnssPhase) {
    case GnssPhase::PowerUp:
      if (elapsed < 50) break;
      digitalWrite(PIN_GNSS_RINT, HIGH);     // leave backup mode
      gnssEnter(GnssPhase::WakePulse);
      break;

    case GnssPhase::WakePulse:
      if (elapsed < 3) break;
      digitalWrite(PIN_GNSS_RINT, LOW);
      gnssEnter(GnssPhase::Settle);
      break;

    case GnssPhase::Settle:
      if (elapsed >= 50) gnssEnter(GnssPhase::LockSleep);
      break;

    case GnssPhase::LockSleep:
      if (elapsed < kGnssCommandGapMs) break;
      gnssSend("$PAIR382,1");
      gnssPhaseMs = millis();
      if (++gnssStep >= kGnssLockRepeats) gnssEnter(GnssPhase::Setup);
      break;

    case GnssPhase::Setup:
      if (elapsed < 2 * kGnssCommandGapMs) break;
      gnssSend(kGnssSetup[gnssStep]);
      gnssPhaseMs = millis();
      if (++gnssStep >= kGnssSetupCount) gnssEnter(GnssPhase::Running);
      break;

    default:
      break;
  }
}

// Stops the UART without any wait that could last forever. The core's own
// end() loops until the peripheral reports that it has stopped, and a loop
// with no way out has no place on the way to switching the device off.
void gnssUartStop() {
  NVIC_DisableIRQ(UARTE0_UART0_IRQn);
  NRF_UARTE0->INTENCLR         = 0xFFFFFFFF;
  NRF_UARTE0->EVENTS_RXTO      = 0;
  NRF_UARTE0->EVENTS_TXSTOPPED = 0;
  NRF_UARTE0->TASKS_STOPRX     = 1;
  NRF_UARTE0->TASKS_STOPTX     = 1;

  const uint32_t start = millis();
  while (!(NRF_UARTE0->EVENTS_TXSTOPPED && NRF_UARTE0->EVENTS_RXTO) &&
         millis() - start < 20) {
  }

  NRF_UARTE0->ENABLE   = UARTE_ENABLE_ENABLE_Disabled;
  NRF_UARTE0->PSEL.TXD = 0xFFFFFFFF;
  NRF_UARTE0->PSEL.RXD = 0xFFFFFFFF;
  releasePin(PIN_GNSS_MCU_RX);
  releasePin(PIN_GNSS_MCU_TX);
}

// Leaves the receiver in the state it should have while the device is off.
// With the backup supply kept it sits in its backup mode; without, it is
// simply unpowered. Either way no line is left driving into it.
void gnssStop(bool keepBackup) {
  if (gnssUartUp) {
    for (uint8_t i = 0; i < 4; ++i) {
      gnssSend("$PAIR382,0");                // allow sleep again
      delay(kGnssCommandGapMs);
    }
    if (keepBackup) {
      for (uint8_t i = 0; i < kGnssLockRepeats; ++i) {
        gnssSend("$PAIR650,0");              // backup mode until woken by pin
        delay(kGnssCommandGapMs);
      }
    }
    delay(50);
  }

  digitalWrite(PIN_GNSS_EN, LOW);
  if (gnssUartUp) gnssUartStop();
  gnssUartUp = false;
  gnssPhase  = GnssPhase::Off;

  holdWithPull(PIN_GNSS_EN, false);
  holdWithPull(PIN_GNSS_RST, false);
  holdWithPull(PIN_GNSS_RINT, false);        // a high level would wake it
  holdWithPull(PIN_GNSS_SINT, false);
  holdWithPull(PIN_GNSS_VRTC, keepBackup);
  releasePin(PIN_GNSS_RESETB);
}

// How often a device that is lying still still transmits.
constexpr uint32_t kIdleTrackingMs = 10000;

bool haveFix();   // defined below, after pollGnss()

// Movement, as far as it can be told without a GNSS fix: either the
// accelerometer is absent, or it reported motion within the last fifteen
// seconds.
bool motionActive() {
  return !accel.ok() || (millis() - lastMotionMs < 15000UL);
}

// While the device lies still it still reports, just rarely, so a pilot on the
// ground stays visible without occupying the band. Only asked here; the time
// is noted once the frame has actually been dealt with, so one that had to
// wait for the channel keeps its turn.
bool idleTrackingDue(uint32_t lastMs) {
  return millis() - lastMs >= kIdleTrackingMs;
}

void pollGnss() {
  uint16_t budget = 512;
  while (Serial1.available() && budget--) {
    const char c = char(Serial1.read());
    gps.encode(c);

    if (nmeaLen < sizeof(nmeaLine) - 3) nmeaLine[nmeaLen++] = c;
    if (c == '\n') {
      // $xxGSV,<total>,<msg>,<inview>,... : the view count is repeated in every
      // message of a group, so it is recorded once per group (msg == 1).
      if (nmeaLen > 7 && !memcmp(nmeaLine + 3, "GSV", 3)) {
        nmeaLine[nmeaLen] = '\0';
        int msg = 0, view = 0;
        if (sscanf(nmeaLine + 7, "%*d,%d,%d", &msg, &view) == 2) {
          int idx = -1;
          if      (nmeaLine[1] == 'G' && nmeaLine[2] == 'P') idx = 0;
          else if (nmeaLine[1] == 'G' && nmeaLine[2] == 'L') idx = 1;
          else if (nmeaLine[1] == 'G' && nmeaLine[2] == 'A') idx = 2;
          else if ((nmeaLine[1] == 'G' && nmeaLine[2] == 'B') ||
                   (nmeaLine[1] == 'B' && nmeaLine[2] == 'D')) idx = 3;
          if (idx >= 0 && msg == 1) {
            gsvViewSats[idx] = uint8_t(view > 255 ? 255 : view);
            gsvViewSeen[idx] = true;
          }
        }
      }
      // The position sentence leads each report, which makes its arrival the
      // best available mark for where the UTC second started.
      if (nmeaLen > 6 && !memcmp(nmeaLine + 3, "GGA", 3)) {
        utcClock.onReport(millis());
      }
      // Only the sentences cockpit applications need. The full GNSS stream
      // exceeds what the BLE link can carry and would starve the responses.
      if (nmeaLen > 6 && (!memcmp(nmeaLine + 3, "GGA", 3) ||
                          !memcmp(nmeaLine + 3, "RMC", 3))) {
        bleWriteNmea(nmeaLine, nmeaLen);
      }
      nmeaLen = 0;
    } else if (nmeaLen >= sizeof(nmeaLine) - 3) {
      nmeaLen = 0;
    }
  }

  // One filter step per reported position. isUpdated() clears on read, so this
  // runs once per fix rather than on every pass through the loop.
  if (gps.location.isUpdated() && gps.location.isValid()) {
    gnss::Fix f;
    f.latitude    = gps.location.lat();
    f.longitude   = gps.location.lng();
    f.altitude    = gps.altitude.isValid() ? float(gps.altitude.meters()) : 0.0f;
    f.groundSpeed = gps.speed.isValid() ? float(gps.speed.mps()) : 0.0f;
    f.track       = gps.course.isValid() ? float(gps.course.deg()) : 0.0f;
    f.satellites  = gps.satellites.isValid() ? uint8_t(gps.satellites.value()) : 0;
    f.timeMs      = millis();
    // The detector reads the filtered values: raw speed and altitude jitter
    // would have it changing its mind on the ground.
    if (gnssFilter.update(f)) {
      const gnss::Position& pos = gnssFilter.position();
      flightState.update(f.timeMs, pos.groundSpeed, pos.climbRate, pos.altitude);
    }
  }
}

// TinyGPS++ marks a location valid from the first fix onwards and never takes
// it back, so age is what separates a live fix from the memory of one. The
// filter must have accepted a position as well: a reading it threw away is not
// something to transmit.
bool haveFix() {
  return gnssFilter.valid() &&
         gps.location.isValid() &&
         gps.location.age() < kFixMaxAgeMs &&
         gps.satellites.isValid() &&
         gps.satellites.value() >= gnss::kMinSatellites;
}

// Satellites in view (GSV), summed over all constellations. Falls back to the
// GGA count while no GSV has arrived yet.
uint8_t satsInView() {
  if (!(gsvViewSeen[0] || gsvViewSeen[1] || gsvViewSeen[2] || gsvViewSeen[3]))
    return gps.satellites.isValid() ? uint8_t(gps.satellites.value()) : 0;
  return uint8_t(gsvViewSats[0] + gsvViewSats[1] +
                 gsvViewSats[2] + gsvViewSats[3]);
}

// ---------------------------------------------------------------------------
//  Transmission
// ---------------------------------------------------------------------------
void flashTx() { txFlashMs = millis(); }

void sendFanet() {
  // Without a live fix there is no position worth transmitting. Sending the
  // last known one would put the aircraft somewhere it is not, which is worse
  // than being absent from the network.
  if (!cfg.fanetOn || !radio.ready() || !haveFix()) return;

  const gnss::Position& fix = gnssFilter.position();

  const proto::fanet::Identity id{cfg.vendorId, cfg.deviceId};
  const uint32_t now = millis();

  // Full cadence while moving; one frame every ten seconds while lying still,
  // so a resting pilot stays visible without flooding the band.
  const bool trackingDue = (now - lastTrackingMs) >=
      (motionActive() ? uint32_t(cfg.trackMs) : kIdleTrackingMs);
  const bool nameDue = cfg.nameMs && cfg.pilot[0] &&
                       (now - lastNameMs) >= cfg.nameMs;
  if (!trackingDue && !nameDue) return;

  // LoRa has no slots of its own, but the radio is shared: a frame started at
  // the wrong moment would still be on the air when a FLARM or ADS-L slot
  // comes up. One frame per second, in the stretch that is free of both.
  if (!fanetSlot.due(utcClock, timing::kFanetWindow, now, uint32_t(random(0x7FFFFFFF)))) return;

  // Large enough for any of the three frames below.
  constexpr size_t kFrameMax =
      std::max({proto::fanet::kNameMaxBytes, proto::fanet::kTrackingBytes,
                proto::fanet::kGroundBytes});
  uint8_t frame[kFrameMax];
  size_t  len = 0;

  if (nameDue) {
    len = proto::fanet::buildName(id, cfg.pilot, frame);
  } else if (flightState.airborne()) {
    proto::fanet::State s;
    s.latitude       = fix.latitude;
    s.longitude      = fix.longitude;
    s.altitude       = fix.altitude;
    s.groundSpeed    = fix.groundSpeed * 3.6f;      // m/s to km/h
    s.climbRate      = fix.climbRate;
    s.track          = fix.track;
    s.aircraft       = proto::fanet::Aircraft(cfg.acType);
    s.onlineTracking = cfg.online;
    len = proto::fanet::buildTracking(id, s, frame);
  } else {
    // On the ground there is no aircraft to report: no altitude, no
    // velocity, no category. Ground tracking says where we are and what we
    // are doing, and receivers show it as a walker rather than as traffic.
    len = proto::fanet::buildGround(id, fix.latitude, fix.longitude,
                                    proto::fanet::Ground::Walking, cfg.online, frame);
  }

  const Radio::Result r = radio.sendFanet(frame, len);
  if (r == Radio::Result::Busy) {
    fanetSlot.defer(utcClock, timing::kFanetWindow, millis(), radio.retryDelayMs());
    return;
  }
  // Over budget, the frame stays due and is tried again in a later second.
  if (r == Radio::Result::NoBudget) return;

  if (nameDue) lastNameMs = now;
  else         lastTrackingMs = now;
  if (r == Radio::Result::Sent) flashTx();
}

// UTC date and time to unix seconds.
//
// The timestamp feeds the key derivation of both FLARM frame formats, so it has
// to be exact UTC. That rules out mktime(), which interprets its input as local
// time: anywhere outside UTC the derived key would be wrong and every frame
// would be discarded by the receiver, with nothing to see on the transmitting
// side. Computed directly here, which also avoids depending on libc.
uint32_t toUnixTime(uint16_t year, uint8_t month, uint8_t day,
                    uint8_t hour, uint8_t minute, uint8_t second) {
  const int32_t  y   = int32_t(year) - (month <= 2 ? 1 : 0);
  const int32_t  era = (y >= 0 ? y : y - 399) / 400;
  const uint32_t yoe = uint32_t(y - era * 400);
  const uint32_t doy = uint32_t((153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1);
  const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const int32_t  days = era * 146097 + int32_t(doe) - 719468;
  return uint32_t(days) * 86400UL + hour * 3600UL + minute * 60UL + second;
}

// Current UTC as unix seconds, or 0 while the receiver has no valid time.
uint32_t gnssUnixTime() {
  if (!gps.date.isValid() || !gps.time.isValid()) return 0;
  return toUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                    gps.time.hour(), gps.time.minute(), gps.time.second());
}


proto::flarm::Aircraft flarmAircraft(uint8_t fanetType) {
  using A = proto::flarm::Aircraft;
  switch (fanetType) {
    case 1:  return A::Paraglider;
    case 2:  return A::HangGlider;
    case 3:  return A::Balloon;
    case 4:  return A::Glider;
    case 5:  return A::Powered;
    case 6:  return A::Helicopter;
    case 7:  return A::Uav;
    default: return A::Unknown;
  }
}

void sendFlarm() {
  if (!cfg.flarmOn || !radio.ready() || !haveFix()) return;
  // The frame is keyed on the time of transmission, so a wrong clock is as
  // useless to a receiver as a wrong position.
  if (!gps.date.isValid() || !gps.time.isValid()) return;

  // Once per second, at a random moment inside the first slot. Receivers are
  // on this frequency only for that part of the second.
  if (!flarmSlot.due(utcClock, timing::kFlarmWindow, millis(),
                     uint32_t(random(0x7FFFFFFF)))) return;

  // Full cadence while moving; one frame every ten seconds while lying still.
  if (!motionActive() && !idleTrackingDue(lastIdleFlarmMs)) return;

  const gnss::Position& fix = gnssFilter.position();

  proto::flarm::State s;
  s.latitude    = fix.latitude;
  s.longitude   = fix.longitude;
  s.altitude    = fix.altitude;
  s.groundSpeed = fix.groundSpeed;
  s.climbRate   = fix.climbRate;
  s.track       = fix.track;
  s.address     = (uint32_t(cfg.flarmVendor) << 16) | cfg.flarmDevId;
  s.aircraft    = flarmAircraft(cfg.acType);
  s.utcSeconds  = toUnixTime(gps.date.year(), gps.date.month(), gps.date.day(),
                             gps.time.hour(), gps.time.minute(), gps.time.second());

  // Alternating keeps receivers that only understand the older format served.
  proto::flarm::Format fmt;
  if      (cfg.flarmProto == 6) fmt = proto::flarm::Format::V6;
  else if (cfg.flarmProto == 7) fmt = proto::flarm::Format::V7;
  else fmt = (radio.flarmCount() & 1) ? proto::flarm::Format::V7
                                      : proto::flarm::Format::V6;
  lastFrameWasV7 = (fmt == proto::flarm::Format::V7);

  uint8_t frame[proto::flarm::kFrameBytes];
  proto::flarm::buildFrame(s, fmt, frame);

  const Radio::Result r = radio.sendFlarm(frame, sizeof(frame));
  if (r == Radio::Result::Busy &&
      flarmSlot.defer(utcClock, timing::kFlarmWindow, millis(), radio.retryDelayMs())) return;

  lastIdleFlarmMs = millis();
  if (r == Radio::Result::Sent) flashTx();
}

// ADS-L category (spec G.1.3) for the FANET aircraft type from the config,
// following the same semantics as the OGN mapping in the reference impl.
proto::ads_l::Aircraft adslCategory(uint8_t fanetType) {
  using A = proto::ads_l::Aircraft;
  switch (fanetType) {
    case 1:  case 2: return A::HangGliderParaglider;
    case 3:           return A::LighterThanAir;
    case 4:           return A::Glider;
    case 5:           return A::LightFixedWing;
    case 6:           return A::Rotorcraft;
    case 7:           return A::UasOpen;
    default:          return A::None;
  }
}

// Quarter seconds into the current hour (spec G.1.1). Values 60..63 are
// forbidden, so the count wraps at 60 rather than 64.
uint8_t adslTimestamp(uint32_t utcSeconds) {
  return uint8_t(((utcSeconds % 3600UL) * 4UL) % 60UL);
}

void sendAdsl() {
  if (!cfg.adslOn || !radio.ready() || !haveFix()) return;
  if (!gps.date.isValid() || !gps.time.isValid()) return;

  // The two channels take turns, and each belongs to a different slot: the
  // upper one to the first, the lower one to the second, which is the reverse
  // of FLARM and what receivers that hop between them expect.
  const timing::Window& window = adslUpperChannel ? timing::kAdslUpperWindow
                                                  : timing::kAdslLowerWindow;
  if (!adslSlot.due(utcClock, window, millis(), uint32_t(random(0x7FFFFFFF)))) return;

  // Full cadence while moving; one frame every ten seconds while lying still.
  if (!motionActive() && !idleTrackingDue(lastIdleAdslMs)) return;

  const gnss::Position& fix = gnssFilter.position();

  proto::ads_l::State s;
  s.latitude    = fix.latitude;
  s.longitude   = fix.longitude;
  s.altitude    = fix.altitude;
  s.groundSpeed = fix.groundSpeed;
  s.climbRate   = fix.climbRate;
  s.track       = fix.track;
  s.addressTable = cfg.adslTable;
  s.address      = cfg.adslAddr ? cfg.adslAddr
                                : (uint32_t(cfg.flarmVendor) << 16) | cfg.flarmDevId;
  s.aircraft    = uint8_t(adslCategory(cfg.acType));
  s.timestamp   = adslTimestamp(toUnixTime(gps.date.year(), gps.date.month(),
                                           gps.date.day(), gps.time.hour(),
                                           gps.time.minute(), gps.time.second()));
  s.flightState = flightState.airborne() ? 2 : 1;   // spec G.1.2

  uint8_t frame[proto::ads_l::kFrameBytes];
  proto::ads_l::buildFrame(s, frame);

  const Radio::Result r = radio.sendAdsl(frame, sizeof(frame), adslUpperChannel);
  if (r == Radio::Result::Busy &&
      adslSlot.defer(utcClock, window, millis(), radio.retryDelayMs())) return;

  lastIdleAdslMs = millis();
  if (r == Radio::Result::Sent) flashTx();
  adslUpperChannel = !adslUpperChannel;
}

// ---------------------------------------------------------------------------
//  Automatic shutdown
// ---------------------------------------------------------------------------
void checkAutoOff() {
  // Debounced over three measurements: the supply dips briefly during
  // transmission and a single reading must not switch the device off.
  static uint32_t judgedSampleMs = 0;
  if (batteryValid && batterySampledMs != judgedSampleMs) {
    judgedSampleMs = batterySampledMs;
    if (batteryBelowCutoff()) {
      if (++lowVoltageStrikes >= 3) {
        melodyLowBattery();
        enterSystemOff();
      }
    } else {
      lowVoltageStrikes = 0;
    }
  }

  // Only active while the sensor actually answers. A silent sensor looks like
  // no movement and must never shut the device down by itself.
  if (cfg.autoOffMin && accel.ok()) {
    if (accel.motionSince()) lastMotionMs = millis();
    if (millis() - lastMotionMs >= uint32_t(cfg.autoOffMin) * 60000UL) {
      shutdownIndication();
      enterSystemOff();
    }
  }
}

// ---------------------------------------------------------------------------
//  Status LED
// ---------------------------------------------------------------------------
//  No fix: amber, with red blinking out twice a second as a sign of life and
//  one short gap per satellite. With a fix: green flashes on transmission.
void updateLeds() {
  static uint32_t phaseMs       = 0;
  static uint8_t  step          = 0;
  static bool     everHadFix    = false;
  static bool     lossAnnounced = false;
  static uint32_t lostSinceMs   = 0;

  if (shuttingDown) {
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, LOW);
    return;
  }

  const uint32_t now = millis();

  // A client waiting for confirmation overrides the fix display: it is the
  // only thing the device needs from the user at that moment.
  if (bleAuthPending()) {
    const bool on = (now % 400) < 200;
    digitalWrite(PIN_LED_GREEN, on ? HIGH : LOW);
    digitalWrite(PIN_LED_RED,   on ? LOW : HIGH);
    return;
  }

  const bool fix = haveFix();

  if (fix) {
    // Announced on the first fix, and again after a loss that was announced.
    // A short dropout stays silent in both directions.
    if (!everHadFix || lossAnnounced) {
      melodyFix();
      digitalWrite(PIN_LED_RED, LOW);
    }
    everHadFix    = true;
    lossAnnounced = false;
    lostSinceMs   = 0;
  } else if (everHadFix) {
    if (!lostSinceMs) {
      lostSinceMs = now;
    } else if (!lossAnnounced && now - lostSinceMs >= kFixLossGraceMs) {
      // Transmission stopped the moment the fix went stale; the tone is what
      // tells the pilot, who is not looking at the device.
      melodyFixLost();
      // The next fix may be a long way from here, and holding this one as a
      // reference would have the filter reject it as a jump.
      gnssFilter.reset();
      // The state stands; what happened while there was no fix is unknown and
      // must not count towards a takeoff or a landing.
      flightState.reset();
      lossAnnounced = true;
    }
  }

  if (!fix) {
    const bool redBeat = (now % 500) >= 60;   // red blinks out twice a second
    digitalWrite(PIN_LED_RED, redBeat ? HIGH : LOW);

    uint8_t satellites = satsInView();
    if (satellites > 12) satellites = 12;
    if (satellites == 0) { digitalWrite(PIN_LED_GREEN, HIGH); return; }

    // Against amber a green blink would be invisible, so the count shows as
    // short dark gaps instead. Both colours go out for a gap: with only the
    // green one off, each gap would read as a red flash.
    const uint8_t pulses = uint8_t(satellites * 2);
    if (step < pulses) {
      const bool gap = (step % 2) == 0;
      digitalWrite(PIN_LED_GREEN, gap ? LOW : HIGH);
      if (gap) digitalWrite(PIN_LED_RED, LOW);
      if (now - phaseMs >= (gap ? 110u : 190u)) { phaseMs = now; ++step; }
    } else {
      digitalWrite(PIN_LED_GREEN, HIGH);
      if (now - phaseMs >= 1500u) { phaseMs = now; step = 0; }
    }
    return;
  }

  digitalWrite(PIN_LED_RED, LOW);
  step = 0;
  // A real transmission is a longer flash; while the device lies still, a very
  // short blink every second keeps it visibly alive between the ten-second
  // idle transmissions.
  const bool txGlow   = txFlashMs && (now - txFlashMs) < 150;
  const bool idleBeat = !motionActive() && ((now % 1000) < 25);
  digitalWrite(PIN_LED_GREEN, (txGlow || idleBeat) ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
//  Button
// ---------------------------------------------------------------------------
//  Short press reports status. Holding three seconds plays the shutdown
//  melody; releasing after it powers the device off. Holding on only
//  postpones that.
constexpr uint32_t kShutdownHoldMs = 3000;

void handleButton() {
  static bool     previous = false;
  static uint32_t pressedAt = 0;
  static bool     armed = false;
  static bool     released = false;
  static bool     usedForPairing = false;

  const bool now = buttonPressed();

  // After a confirmed wake the button is still held, so ignore everything
  // until it has been released once.
  if (!released) {
    if (!now) { released = true; previous = false; }
    return;
  }

  if (now && !previous) {
    pressedAt = millis();
    armed = false;
    usedForPairing = false;

    if (bleAuthPending()) {
      // A client waiting for confirmation is answered on the press itself,
      // not on the release: the click says at once that the button was hit,
      // and the melody follows as soon as the client has its answer. The
      // whole point of the confirmation is that it happens here rather than
      // over the air, so while one is waiting the press means this and
      // nothing else.
      usedForPairing = true;
      beep(2200, 25);
      if (bleAuthConfirm()) melodyPaired();
      else                  melodyRefuse();
    }
  }

  if (now && !armed && millis() - pressedAt >= kShutdownHoldMs) {
    armed = true;
    shutdownIndication();
  }

  if (!now && previous) {
    if (armed) {
      enterSystemOff();
    } else if (!usedForPairing && millis() - pressedAt < 1000) {
      beep(1500, 80);
      bleSendStatus();
    }
  }
  previous = now;
}

// ADS-L uses its own category list, so map it onto the one the NMEA output
// expects. Several ADS-L categories have no counterpart and become unknown.
// The ADS-L address mapping table is wider than the three identifier types the
// data port sentence knows, so anything without a counterpart is reported as an
// unassigned address rather than claimed to be something it is not.
uint8_t adslToNmeaAddressType(uint8_t table) {
  switch (table) {
    case 5:  return 1;    // ICAO
    case 6:  return 2;    // FLARM
    default: return 0;    // OGN, FANET, manufacturer pages: no equivalent
  }
}

AircraftType adslToNmeaAircraft(uint8_t category) {
  switch (category) {
    case 1:  return AircraftType::Powered;        // light fixed wing
    case 2:  return AircraftType::Jet;            // small to heavy fixed wing
    case 3:  return AircraftType::Helicopter;
    case 4:  return AircraftType::Glider;
    case 5:  return AircraftType::Balloon;        // lighter than air
    case 6:  return AircraftType::Powered;        // ultralight
    case 7:  return AircraftType::Paraglider;     // hang glider or paraglider
    case 8:  return AircraftType::Parachute;
    case 10: return AircraftType::Helicopter;     // gyrocopter
    case 11:
    case 12:
    case 13: return AircraftType::Uav;
    default: return AircraftType::Unknown;
  }
}

// ---------------------------------------------------------------------------
//  Receiving other aircraft
// ---------------------------------------------------------------------------
//  Only one protocol can be received at a time, because each uses a different
//  modem and there is one radio. Reception is off unless asked for: listening
//  keeps the receiver powered continuously and is the largest single draw on
//  this board.
//
//  Decoded aircraft are reported as the FLARM data port sentences, which is
//  what navigation applications understand regardless of how the frame
//  arrived. Weather stations have no standard sentence and use our own.

constexpr size_t   kMaxTargets      = 16;
constexpr uint32_t kTargetTimeoutMs = 20000;   // drop a target after silence

struct Target {
  uint32_t address  = 0;
  uint32_t lastSeen = 0;
  bool     used     = false;
};

Target   targets[kMaxTargets];

// The targets above are only addresses and timestamps, enough to count what is
// in range for the status sentence. This keeps the reports themselves, so a
// client that connects a minute later starts with what is in the air rather
// than with an empty list.
rx::Cache rxCache;
uint32_t lastStatusSentenceMs = 0;
uint32_t rxFrameCount   = 0;
uint32_t rxDecodedCount = 0;

Radio::Mode wantedRxMode() {
  if (!cfg.rxOn) return Radio::Mode::None;
  switch (cfg.rxProto) {
    case 1:  return Radio::Mode::Fanet;
    case 2:  return Radio::Mode::Adsl;
    default: return Radio::Mode::Flarm;
  }
}

// Remembers a target and reports how many are currently being tracked.
unsigned noteTarget(uint32_t address) {
  const uint32_t now = millis();
  int free = -1;

  for (size_t i = 0; i < kMaxTargets; ++i) {
    if (targets[i].used && now - targets[i].lastSeen > kTargetTimeoutMs) {
      targets[i].used = false;
    }
    if (targets[i].used && targets[i].address == address) {
      targets[i].lastSeen = now;
      address = 0;                    // already counted
    } else if (!targets[i].used && free < 0) {
      free = int(i);
    }
  }

  if (address != 0 && free >= 0) {
    targets[free] = Target{address, now, true};
  }

  unsigned count = 0;
  for (const auto& t : targets) if (t.used) ++count;
  return count;
}

unsigned trackedCount() {
  const uint32_t now = millis();
  unsigned count = 0;
  for (auto& t : targets) {
    if (t.used && now - t.lastSeen > kTargetTimeoutMs) t.used = false;
    if (t.used) ++count;
  }
  return count;
}

nmea::Own ownPosition() {
  nmea::Own own;
  own.valid = haveFix();
  if (own.valid) {
    const gnss::Position& fix = gnssFilter.position();
    own.latitude  = fix.latitude;
    own.longitude = fix.longitude;
    own.altitude  = fix.altitude;
    own.airborne  = fix.groundSpeed > 2.0f;
  }
  return own;
}

void emitSentence(const char* line, size_t len) {
  if (len) bleWriteNmea(line, len);
}

void reportTraffic(const Traffic& t) {
  const unsigned tracked = noteTarget(t.address);
  ++rxDecodedCount;
  rxCache.note(millis(), t);

  char line[128];
  emitSentence(line, nmea::buildTraffic(line, sizeof(line), t, ownPosition()));
  bleSendTraffic(t, tracked);
}

void reportWeather(const Weather& w) {
  ++rxDecodedCount;
  rxCache.note(millis(), w);
  char line[160];
  emitSentence(line, nmea::buildWeather(line, sizeof(line), w));
  bleSendWeather(w);
}

// Everything heard in the last few minutes, sent to a client that has just
// arrived. Each report carries its real age, so nothing old is shown as if it
// had just come in.
//
// The cache holds more than the link takes in one go, so it is handed over a
// few entries at a time as the queue empties, see continueReplay().
unsigned replayNext      = rx::kMaxEntries;   // past the end: nothing to do
uint32_t replayStartedMs = 0;

void replayReceptions() {
  rxCache.expire(millis());
  replayNext      = 0;
  replayStartedMs = millis();
}

void continueReplay() {
  if (replayNext >= rxCache.capacity()) return;

  static uint32_t lastTryMs = 0;
  const uint32_t now = millis();
  if (now - lastTryMs < 20) return;
  lastTryMs = now;

  // A client that left in the middle has no queue to fill, which looks the
  // same as a full one. Given up on rather than retried forever.
  if (now - replayStartedMs > 5000) { replayNext = rxCache.capacity(); return; }

  // The most one entry produces: a position, a weather report and a name.
  constexpr size_t kEntryRoom = 512;
  const unsigned tracked = trackedCount();

  while (replayNext < rxCache.capacity() && bleEventRoom() >= kEntryRoom) {
    const rx::Entry& e = rxCache.at(replayNext++);
    if (!e.used) continue;
    const uint32_t age = (now - e.lastSeen) / 1000U;

    if (e.hasTraffic) bleSendTraffic(e.traffic, tracked, age);
    if (e.hasWeather) bleSendWeather(e.weather, age);
    // The name last: it is the one that means nothing on its own, and this way
    // it lands on an entry the client already has.
    if (e.hasName)    bleSendName(e.name, age);
  }
}

void pollRadio() {
  const Radio::Mode wanted = wantedRxMode();
  if (radio.listening() != wanted) {
    radio.listen(wanted);
    if (wanted == Radio::Mode::None) {
      for (auto& t : targets) t.used = false;
      rxCache.clear();
    }
  }
  if (wanted == Radio::Mode::None) return;

  uint8_t frame[64];
  const size_t len = radio.poll(frame, sizeof(frame));
  if (len == 0) return;
  ++rxFrameCount;

  switch (wanted) {
    case Radio::Mode::Flarm: {
      if (len < proto::flarm::kFrameBytes) return;
      proto::flarm::Reference ref;
      ref.latitude   = gps.location.lat();
      ref.longitude  = gps.location.lng();
      ref.utcSeconds = gnssUnixTime();
      // Positions are sent relative to the receiver, and the clock keys the
      // cipher, so nothing can be decoded before we have a fix ourselves.
      if (!haveFix() || ref.utcSeconds == 0) return;

      Traffic t;
      if (proto::flarm::decodeFrame(frame, ref, t)) {
        t.rssi = radio.lastRssi();
        reportTraffic(t);
      }
      break;
    }

    case Radio::Mode::Fanet: {
      Report r;
      if (!proto::fanet::decodeFrame(frame, len, r)) return;
      if (r.kind == ReportKind::Traffic) {
        r.traffic.rssi = radio.lastRssi();
        reportTraffic(r.traffic);
      } else if (r.kind == ReportKind::Weather) {
        r.weather.rssi = radio.lastRssi();
        reportWeather(r.weather);
      } else if (r.kind == ReportKind::Name) {
        rxCache.note(millis(), r.name);
        bleSendName(r.name);
      }
      break;
    }

    case Radio::Mode::Adsl: {
      // The frame is Manchester encoded on air and has to be halved first.
      uint8_t body[proto::ads_l::kFrameBytes];
      if (len < 2 * sizeof(body)) return;
      if (!proto::manchesterDecode(frame, sizeof(body), body)) return;

      proto::ads_l::Decoded d;
      if (!proto::ads_l::decodeFrame(body, d)) return;

      Traffic t;
      t.source      = Source::Adsl;
      t.address     = d.address;
      t.addressType = adslToNmeaAddressType(d.addressTable);
      t.latitude    = d.latitude;
      t.longitude   = d.longitude;
      t.altitude    = d.altitude;
      t.groundSpeed = d.groundSpeed;
      t.climbRate   = d.climbRate;
      t.track       = d.track;
      t.aircraft    = adslToNmeaAircraft(d.aircraft);
      t.hasPosition = true;
      t.hasVelocity = true;
      t.rssi        = radio.lastRssi();
      reportTraffic(t);
      break;
    }

    default:
      break;
  }
}

// The heartbeat sentence is what a consumer uses to detect that a compatible
// stream is present at all, so it goes out every second while receiving.
void sendRxStatus() {
  if (!cfg.rxOn) return;
  if (millis() - lastStatusSentenceMs < 1000) return;
  lastStatusSentenceMs = millis();

  const bool transmitting = cfg.fanetOn || cfg.flarmOn || cfg.adslOn;
  char line[96];
  emitSentence(line, nmea::buildStatus(line, sizeof(line), trackedCount(),
                                       transmitting, ownPosition()));
}

// ---------------------------------------------------------------------------
//  Callbacks for the BLE layer
// ---------------------------------------------------------------------------
void fillStatus(LiveStatus& st) {
  st.fix        = haveFix();
  st.sats       = satsInView();
  const gnss::Position& fix = gnssFilter.position();
  st.lat        = fix.latitude;
  st.lon        = fix.longitude;
  st.altM       = fix.altitude;
  st.speedKmh   = fix.groundSpeed * 3.6f;
  st.climbMs    = fix.climbRate;
  st.headingDeg = fix.track;
  st.battV      = batteryVolts;
  st.battPct    = batteryPercent(st.battV);
  st.charging   = usbPresent();
  st.fanetTx    = radio.fanetCount();
  st.flarmTx    = radio.flarmCount();
  st.adslTx     = radio.adslCount();
  st.txSkipped  = radio.skippedCount();
  st.txBusy     = radio.busyCount();
  st.flarmState = radio.lastStatus();
  st.flarmV7    = lastFrameWasV7;
  st.motionOk   = accel.ok();
  st.noMotionS  = (millis() - lastMotionMs) / 1000UL;
  st.radioOk    = radio.ready();
}

// A client that has not been confirmed is asking to configure the device.
// This has to be noticeable without looking at the device, and it repeats: the
// prompt fires once per connection, so a pilot who missed it simply reconnects.
void onAuthPrompt() {
  // A client that keeps trying would otherwise have the device beeping
  // continuously. The prompt is rate limited; the LED keeps signalling for as
  // long as one is waiting.
  static uint32_t lastPromptMs = 0;
  const uint32_t now = millis();
  if (lastPromptMs && now - lastPromptMs < 10000) return;
  lastPromptMs = now;
  melodyAskPairing();
}

void onConfigChanged() {
  radio.setTxPower(cfg.txPower);
  radio.setAccess(cfg.lbt, cfg.flarmOn, cfg.adslOn);
  radio.listen(wantedRxMode());
}
void onBeepRequest(uint8_t volume) { beepAt(1400, 180, volume); }

}  // namespace

// ===========================================================================
void setup() {
  boardInit();
  watchdogStart();

  cfgSetDefaults();
  cfgLoad();

  // A power-off that did not take effect on the last run, before anything else
  // gets a chance to enable the SoftDevice again.
  if (NRF_POWER->GPREGRET == kPowerOffRetryMagic) {
    NRF_POWER->GPREGRET = 0;
    retryPowerOff();
  }

  Serial.begin(115200);
  buzzerInit();

  // Devices whose clocks agree must not pick the same moment in a slot, so the
  // sequence starts somewhere else on every unit.
  randomSeed(NRF_FICR->DEVICEID[0] ^ NRF_FICR->DEVICEID[1] ^ micros());

  // Refuse to start on an empty battery, before asking for the wake gesture.
  measureBattery();
  if (batteryBelowCutoff()) {
    melodyRefuse();
    enterSystemOff();
  }

  // Whatever restarted the chip, the device comes back in the state it was
  // in. Only the button switches it on.
  const uint32_t reason = readResetReason();
  if (reason & POWER_RESETREAS_OFF_Msk) {
    // The button woke it from system-off.
    confirmWake();
  } else if (reason & POWER_RESETREAS_VBUS_Msk) {
    // A cable was plugged into a device that was switched off.
    chargingIdle(true);
  } else if (!cfgPowerIntent()) {
    // Off before the restart. A reset with no reason recorded is a power-up,
    // which is how plugging in or pulling out a cable can arrive; any other
    // reason is the firmware's own, such as switching off while charging.
    if (usbPresent()) chargingIdle(reason == 0);
    else              enterSystemOff();
  }
  // On from here, until switched off.
  cfgSetPowerIntent(true);

  gnssStart();

  Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.begin();
  i2cUp = true;
  const bool motionOk = accel.begin();
  lastMotionMs = millis();

  const bool radioOk = radio.begin(cfg.txPower);
  radio.setAccess(cfg.lbt, cfg.flarmOn, cfg.adslOn);

  // The name lives in the scan response, which has its own 31 byte budget, so
  // 21 characters plus the two byte header fit comfortably.
  char name[32];
  snprintf(name, sizeof(name), "BreezedudeBeacon-%04X", cfg.deviceId);
  bleBegin(name);
  bleSetStatusProvider(fillStatus);
  bleSetConfigChangedCb(onConfigChanged);
  bleSetAuthPromptCb(onAuthPrompt);
  bleSetReplayCb(replayReceptions);
  bleSetBeepCb(onBeepRequest);

  if (!wakeConfirmed) {
    digitalWrite(PIN_LED_GREEN, HIGH);
    melodyOn();
  }
  digitalWrite(PIN_LED_GREEN, LOW);

  Serial.printf("\r\n=== Breezedude Beacon %s ===\r\n", FW_VERSION);
  Serial.printf("BLE name : %s\r\n", name);
  Serial.printf("Radio    : %s (status %d)\r\n",
                radioOk ? "ok" : "FAILED", radio.lastStatus());
  Serial.printf("Motion   : %s, auto-off %s, cutoff %u mV\r\n",
                motionOk ? "ok" : "NOT FOUND (motion auto-off disabled)",
                (cfg.autoOffMin && motionOk) ? "on" : "off", cfg.lowVoltMv);
}

// ---------------------------------------------------------------------------
void loop() {
  watchdogFeed();
  pollGnss();
  gnssTick();
  batteryTick();
  blePoll();
  continueReplay();
  handleButton();
  sendFanet();
  sendFlarm();
  sendAdsl();
  pollRadio();
  sendRxStatus();
  updateLeds();

  if (millis() - lastStatusMs >= 1000) {
    lastStatusMs = millis();
    bleSetBattery(batteryPercent(batteryVolts));
    bleSendStatus();
    checkAutoOff();

    // Changes are written once they have settled rather than only at the end
    // of the session, and the result is reported so a failure is visible in
    // the configuration page instead of being discovered at the next start.
    const CfgSaveResult saved = cfgSaveIfSettled();
    if (saved != CfgSaveResult::None) bleSendSaved(saved == CfgSaveResult::Ok);
  }

  // Let the processor sleep until the next tick. Without this the loop spins
  // flat out, which costs several milliamps for nothing: everything it polls
  // is either buffered or slower than a millisecond.
  delay(1);
}
