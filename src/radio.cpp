#include "radio.h"

#include <nrf_gpio.h>

#include "pins_t1000e.h"
#include "proto/bits.h"
#include "proto/ads_l.h"
#include "proto/flarm.h"

// Restores the old behaviour of resetting the chip before every frame. It
// costs a third of a second per frame and makes slot timing impossible, so it
// is only here as a way back should a chip turn out to need it.
#ifndef RADIO_REINIT_EACH_FRAME
#define RADIO_REINIT_EACH_FRAME 0
#endif

namespace {

// ---- board ----------------------------------------------------------------
// The crystal oscillator is supplied from the chip's DIO3.
constexpr float kTcxoVoltage = 1.6f;

// The antenna switch is driven by four of the chip's own pins and has to be
// told what to do in every mode. Left unconfigured the pins float, the switch
// connects nothing in particular, and the radio transmits and receives through
// whatever leaks across it: enough for a bench test next to a ground station,
// and a fraction of the range everywhere else.
const uint32_t kRfSwitchPins[Module::RFSWITCH_MAX_PINS] = {
    RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
    RADIOLIB_LR11X0_DIO7, RADIOLIB_LR11X0_DIO8, RADIOLIB_NC};

const Module::RfSwitchMode_t kRfSwitchTable[] = {
    // mode               DIO5  DIO6  DIO7  DIO8
    {LR11x0::MODE_STBY,  {LOW,  LOW,  LOW,  LOW}},
    {LR11x0::MODE_RX,    {HIGH, LOW,  LOW,  HIGH}},
    {LR11x0::MODE_TX,    {HIGH, HIGH, LOW,  HIGH}},   // low power amplifier
    {LR11x0::MODE_TX_HP, {LOW,  HIGH, LOW,  HIGH}},   // high power amplifier
    {LR11x0::MODE_TX_HF, {LOW,  LOW,  LOW,  LOW}},
    {LR11x0::MODE_GNSS,  {LOW,  LOW,  HIGH, LOW}},
    {LR11x0::MODE_WIFI,  {LOW,  LOW,  LOW,  LOW}},
    END_OF_MODE_TABLE,
};

// ---- FANET: LoRa ------------------------------------------------------------
constexpr float   kFanetFrequency  = 868.2f;  // MHz
constexpr float   kFanetBandwidth  = 250.0f;  // kHz
constexpr uint8_t kFanetSpreading  = 7;
constexpr uint8_t kFanetCodingRate = 5;       // 4/5
constexpr uint8_t kFanetSyncWord   = 0xF1;
constexpr uint16_t kFanetPreamble  = 12;

// ---- FLARM and ADS-L: 2-FSK, 100 kbit/s, Manchester in software ------------
constexpr float    kLowerChannel  = 868.2f;   // MHz
constexpr float    kUpperChannel  = 868.4f;   // MHz
constexpr float    kFskBitrate    = 100.0f;   // kbit/s
constexpr float    kFskDeviation  = 50.0f;    // kHz
// Receive side only. The signal is 200 kHz wide by Carson's rule, so the next
// narrower filter setting would cut into it.
constexpr float    kFskBandwidth  = 234.3f;   // kHz
constexpr uint16_t kFskPreamble   = 8;        // bits

constexpr uint8_t kFlarmSyncWord[] = {0x55, 0x99, 0xA5, 0xA9,
                                      0x55, 0x66, 0x65, 0x96};

// The fixed-key plaintext sync word (Manchester of 0xF5 0x72 0x4B 0x18 with
// the length byte forming the tail), same as every reference receiver.
constexpr uint8_t kAdslSyncWord[] = {0x55, 0x99, 0x95, 0xA6,
                                     0x9A, 0x65, 0xA9, 0x6A};

constexpr size_t kFlarmAirBytes = proto::flarm::kFrameBytes;
constexpr size_t kAdslAirBytes  = 2 * proto::ads_l::kFrameBytes;

// ---- sharing the band, see airtime.h ----------------------------------------
// Time on air of the slotted frames, which have a fixed length.
const uint32_t kFlarmCostUs = airtime::fskUs(kFlarmAirBytes, kFskPreamble, 8, 100000);
const uint32_t kAdslCostUs  = airtime::fskUs(kAdslAirBytes, kFskPreamble, 8, 100000);

uint32_t fanetCostUs(size_t len) {
  return airtime::loraUs(len, kFanetSpreading, 250000, kFanetCodingRate, kFanetPreamble);
}

// What FANET leaves in the account for each second of FLARM and ADS-L, so
// that the slotted protocols are not the ones that go short.
constexpr uint32_t kReserveSeconds = 2;

// Clear channel assessment. The standard measures its threshold at the
// antenna: 15 dB above the sensitivity it requires, which for 200 to 250 kHz
// of channel comes to -79 and -78 dBm, for an antenna with no gain over a
// dipole. This antenna has less, which would allow a higher figure at the
// chip; -85 dBm stays clear of that and well above the noise floor.
constexpr float    kCcaThresholdDbm = -85.0f;
constexpr uint32_t kCcaSettleUs     = 300;    // receiver up and RSSI valid
constexpr uint8_t  kCcaSamples      = 8;      // 2 ms in all; the minimum is 0.16 ms
constexpr uint32_t kCcaStepUs       = 250;

// Random pause after a busy channel, before listening again.
constexpr uint32_t kBackoffMinMs  = 5;
constexpr uint32_t kBackoffSpanMs = 20;

unsigned channelOf(float frequencyMhz) { return frequencyMhz > 868.3f ? 1U : 0U; }

}  // namespace

Radio::Radio()
    : spi_(NRF_SPIM2, PIN_LR_MISO, PIN_LR_SCK, PIN_LR_MOSI),
      radio_(new Module(PIN_LR_NSS, PIN_LR_IRQ, PIN_LR_RST, PIN_LR_BUSY, spi_)) {}

// ---------------------------------------------------------------------------
bool Radio::begin(int8_t txPower) {
  power_ = txPower;
  spi_.begin();
  ready_ = initChip();
  return ready_;
}

void Radio::setTxPower(int8_t txPower) {
  power_ = txPower;
  mode_  = Mode::None;               // force a reconfiguration
}

// Resets and calibrates the chip and leaves it configured for FANET. This is
// the slow part, a third of a second, and happens once rather than per frame.
bool Radio::initChip() {
  chipUp_ = false;
  mode_   = Mode::None;

  status_ = radio_.begin(kFanetFrequency, kFanetBandwidth, kFanetSpreading,
                         kFanetCodingRate, kFanetSyncWord, power_,
                         kFanetPreamble, kTcxoVoltage);
  if (status_ != RADIOLIB_ERR_NONE) return false;

  radio_.setRfSwitchTable(kRfSwitchPins, kRfSwitchTable);

  // The board carries the inductor for the chip's switching regulator. The
  // library selects the linear one, which roughly doubles the receive current.
  status_ = radio_.setRegulatorDCDC();
  if (status_ != RADIOLIB_ERR_NONE) return false;

  status_ = radio_.setCRC(1);
  if (status_ != RADIOLIB_ERR_NONE) return false;

  chipUp_    = true;
  mode_      = Mode::Fanet;
  frequency_ = kFanetFrequency;
  return true;
}

// ---------------------------------------------------------------------------
//  Modem configuration
// ---------------------------------------------------------------------------
//  Every setting is sent again on each switch rather than only the ones that
//  differ. Changing the packet type resets the modulation and packet
//  parameters inside the chip, and the handful of commands takes a few
//  milliseconds.
bool Radio::applyLora(float frequencyMhz) {
  int16_t s = radio_.standby();
  if (s == RADIOLIB_ERR_NONE) s = radio_.setPacketType(RADIOLIB_LR11X0_PACKET_TYPE_LORA);
  if (s == RADIOLIB_ERR_NONE) s = radio_.setBandwidth(kFanetBandwidth);
  if (s == RADIOLIB_ERR_NONE) s = radio_.setSpreadingFactor(kFanetSpreading);
  if (s == RADIOLIB_ERR_NONE) s = radio_.setCodingRate(kFanetCodingRate);
  if (s == RADIOLIB_ERR_NONE) s = radio_.setSyncWord(kFanetSyncWord);
  if (s == RADIOLIB_ERR_NONE) s = radio_.setPreambleLength(kFanetPreamble);
  if (s == RADIOLIB_ERR_NONE) s = radio_.explicitHeader();
  if (s == RADIOLIB_ERR_NONE) s = radio_.setCRC(1);
  if (s == RADIOLIB_ERR_NONE) s = radio_.invertIQ(false);
  if (s == RADIOLIB_ERR_NONE) s = radio_.setFrequency(frequencyMhz, true);
  if (s == RADIOLIB_ERR_NONE) s = radio_.setOutputPower(power_);
  status_ = s;
  return s == RADIOLIB_ERR_NONE;
}

bool Radio::applyFsk(const uint8_t* syncWord, size_t airBytes, float frequencyMhz) {
  int16_t s = radio_.standby();
  if (s == RADIOLIB_ERR_NONE) s = radio_.setPacketType(RADIOLIB_LR11X0_PACKET_TYPE_GFSK);
  if (s == RADIOLIB_ERR_NONE) s = radio_.setBitRate(kFskBitrate);
  if (s == RADIOLIB_ERR_NONE) s = radio_.setFrequencyDeviation(kFskDeviation);
  if (s == RADIOLIB_ERR_NONE) s = radio_.setRxBandwidth(kFskBandwidth);
  if (s == RADIOLIB_ERR_NONE) s = radio_.setPreambleLength(kFskPreamble);
  if (s == RADIOLIB_ERR_NONE) s = radio_.setSyncWord(const_cast<uint8_t*>(syncWord), 8);
  if (s == RADIOLIB_ERR_NONE) s = radio_.setDataShaping(RADIOLIB_SHAPING_0_5);
  if (s == RADIOLIB_ERR_NONE) s = radio_.setCRC(0);            // the checksum is part of the payload
  if (s == RADIOLIB_ERR_NONE) s = radio_.setWhitening(false);  // Manchester is applied in software
  if (s == RADIOLIB_ERR_NONE) s = radio_.disableAddressFiltering();

  // Fixed length framing is mandatory, and this is easy to get wrong: in
  // variable length mode the radio prepends a length byte, which shifts the
  // whole frame. The transmission is perfectly measurable on a spectrum
  // analyser, yet no receiver will accept it. Established by comparing
  // transmitted frames against recordings from a receiving ground station.
  if (s == RADIOLIB_ERR_NONE) s = radio_.fixedPacketLengthMode(uint8_t(airBytes));

  // Both channels lie well inside the band the image rejection was calibrated
  // for when the chip was started, so that step is skipped here.
  if (s == RADIOLIB_ERR_NONE) s = radio_.setFrequency(frequencyMhz, true);
  if (s == RADIOLIB_ERR_NONE) s = radio_.setOutputPower(power_);
  status_ = s;
  return s == RADIOLIB_ERR_NONE;
}

bool Radio::apply(Mode mode, float frequencyMhz) {
  switch (mode) {
    case Mode::Fanet: return applyLora(frequencyMhz);
    case Mode::Flarm: return applyFsk(kFlarmSyncWord, kFlarmAirBytes, frequencyMhz);
    case Mode::Adsl:  return applyFsk(kAdslSyncWord, kAdslAirBytes, frequencyMhz);
    default:          return false;
  }
}

bool Radio::select(Mode mode, float frequencyMhz) {
  if (RADIO_REINIT_EACH_FRAME && mode != mode_) chipUp_ = false;
  if (!chipUp_ && !initChip()) return false;

  if (mode == mode_ && frequencyMhz == frequency_) return true;

  if (!apply(mode, frequencyMhz)) {
    // The chip refused something. Rather than give up on the frame, start it
    // from scratch once; if that does not help either, the caller is told.
    if (!initChip() || !apply(mode, frequencyMhz)) {
      mode_ = Mode::None;
      return false;
    }
  }
  mode_      = mode;
  frequency_ = frequencyMhz;
  return true;
}

// ===========================================================================
//  Transmit
// ===========================================================================
void Radio::setAccess(bool lbt, bool flarmOn, bool adslOn) {
  lbt_ = lbt;
  budget_.setRule(lbt ? airtime::kPolite : airtime::kDutyCycle, millis());
  reserveUs_ = kReserveSeconds * ((flarmOn ? kFlarmCostUs : 0) + (adslOn ? kAdslCostUs : 0));
}

// Listens on the frequency the modem is set to. The radio is in receive for
// a little over two milliseconds and back in standby afterwards.
bool Radio::channelClear() {
  receiving_ = false;
  // A radio that cannot listen cannot say the channel is free.
  if (radio_.startReceive() != RADIOLIB_ERR_NONE) return false;

  delayMicroseconds(kCcaSettleUs);
  float loudest = -200.0f;
  for (uint8_t i = 0; i < kCcaSamples; ++i) {
    const float rssi = radio_.getRSSI(false, true);   // instantaneous, stay in receive
    if (rssi > loudest) loudest = rssi;
    delayMicroseconds(kCcaStepUs);
  }
  radio_.standby();
  return loudest < kCcaThresholdDbm;
}

// Everything every frame goes through: the account, the pause on the
// frequency, the modem, listening, and the transmission itself.
Radio::Result Radio::send(Mode mode, float frequencyMhz, const uint8_t* data, size_t len,
                          uint32_t costUs, uint32_t reserveUs) {
  if (!ready_) return Result::Failed;

  const uint32_t now     = millis();
  const unsigned channel = channelOf(frequencyMhz);

  if (!budget_.affordable(now, costUs, reserveUs)) {
    ++skipped_;
    return Result::NoBudget;
  }
  if (lbt_) {
    const uint32_t wait = offTime_.waitMs(now, channel);
    if (wait) {
      retryMs_ = wait;
      return Result::Busy;
    }
  }

  if (!select(mode, frequencyMhz)) return Result::Failed;

  if (lbt_ && !channelClear()) {
    ++busy_;
    retryMs_ = kBackoffMinMs + uint32_t(random(kBackoffSpanMs));
    resumeReceive();
    return Result::Busy;
  }

  receiving_ = false;
  const uint32_t startUs = micros();
  status_ = radio_.transmit(const_cast<uint8_t*>(data), len);
  const uint32_t tookUs = micros() - startUs;

  // Booked whether or not the radio reported success, and at the longer of
  // estimate and measurement, which includes the time it took to start.
  const uint32_t endMs = millis();
  budget_.charge(endMs, tookUs > costUs ? tookUs : costUs);
  offTime_.transmitted(endMs, channel);

  return status_ == RADIOLIB_ERR_NONE ? Result::Sent : Result::Failed;
}

Radio::Result Radio::sendFanet(const uint8_t* data, size_t len) {
  // FANET is the one that gives way: it leaves room for the slotted frames.
  const Result r = send(Mode::Fanet, kFanetFrequency, data, len, fanetCostUs(len), reserveUs_);
  if (r == Result::Sent) ++fanetCount_;
  resumeReceive();
  return r;
}

Radio::Result Radio::sendFlarm(const uint8_t* data, size_t len) {
  const Result r = send(Mode::Flarm, kLowerChannel, data, len, kFlarmCostUs, 0);

  // The FSK settings are sent afresh for the next frame. A run of consecutive
  // frames on one configuration was once seen to stop being received after
  // the first, and re-sending costs a few milliseconds.
  mode_ = Mode::None;

  if (r == Result::Sent) ++flarmCount_;
  resumeReceive();
  return r;
}

Radio::Result Radio::sendAdsl(const uint8_t* data, size_t len, bool upperChannel) {
  uint8_t air[kAdslAirBytes];
  proto::manchesterEncode(data, len, air);
  const Result r = send(Mode::Adsl, upperChannel ? kUpperChannel : kLowerChannel,
                        air, sizeof(air), kAdslCostUs, 0);

  mode_ = Mode::None;

  if (r == Result::Sent) ++adslCount_;
  resumeReceive();
  return r;
}

// ---------------------------------------------------------------------------
//  Following the reference firmware for this board: back to standby, release
//  the oscillator supply, which also resets the chip into a state that sleep
//  can always be entered from, then sleep without retaining anything.
void Radio::shutdown() {
  // A chip that was never started in this session may be sitting in standby
  // from its own power-on, which draws a milliamp for as long as the battery
  // lasts. It has to be brought up once to be told to sleep.
  if (!chipUp_) {
    spi_.begin();
    initChip();
  }
  if (chipUp_) {
    radio_.standby();
    radio_.setTCXO(0);
    radio_.sleep(false, 0);
  }
  spi_.end();

  // A falling edge on chip select is what wakes the radio, so that line is
  // held high, by a pull-up rather than driven so that it rides through the
  // wake-up reset untouched. Reset stays released. The remaining lines are
  // parked so that nothing floats into an input buffer on either side.
  nrf_gpio_cfg_input(g_ADigitalPinMap[PIN_LR_NSS], NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(g_ADigitalPinMap[PIN_LR_SCK], NRF_GPIO_PIN_PULLDOWN);
  nrf_gpio_cfg_input(g_ADigitalPinMap[PIN_LR_MOSI], NRF_GPIO_PIN_PULLDOWN);
  nrf_gpio_cfg_default(g_ADigitalPinMap[PIN_LR_MISO]);
  nrf_gpio_cfg_default(g_ADigitalPinMap[PIN_LR_BUSY]);
  nrf_gpio_cfg_default(g_ADigitalPinMap[PIN_LR_IRQ]);

  chipUp_    = false;
  ready_     = false;
  receiving_ = false;
  mode_      = Mode::None;
}

// ===========================================================================
//  Receive
// ===========================================================================
//  Only one protocol can be received at a time: each uses a different modem,
//  and the radio has one. Listening is therefore an explicit choice, and
//  Mode::None leaves the chip idle rather than burning current in a receive
//  state nobody reads.
bool Radio::listen(Mode mode) {
  rxMode_ = mode;
  if (!ready_ || mode == Mode::None) {
    receiving_ = false;
    if (ready_) radio_.standby();
    return true;
  }
  return resumeReceive();
}

bool Radio::resumeReceive() {
  if (!ready_ || rxMode_ == Mode::None) return false;
  if (!select(rxMode_, kLowerChannel)) return false;

  status_ = radio_.startReceive();
  receiving_ = (status_ == RADIOLIB_ERR_NONE);
  return receiving_;
}

size_t Radio::poll(uint8_t* buf, size_t cap) {
  if (!receiving_) {
    // Transmitting takes the radio away from us, so pick listening back up.
    if (rxMode_ != Mode::None) resumeReceive();
    return 0;
  }

  // The chip raises its interrupt line when a frame is complete. Asking it
  // over the bus on every pass instead would keep the bus and the processor
  // busy for nothing.
  if (digitalRead(PIN_LR_IRQ) == LOW) return 0;

  const uint32_t flags = radio_.getIrqFlags();
  if (!(flags & RADIOLIB_LR11X0_IRQ_RX_DONE)) {
    radio_.clearIrqFlags(RADIOLIB_LR11X0_IRQ_ALL);
    resumeReceive();
    return 0;
  }

  size_t len = radio_.getPacketLength();
  if (len == 0 || len > cap) {
    radio_.clearIrqFlags(RADIOLIB_LR11X0_IRQ_ALL);
    resumeReceive();
    return 0;
  }

  const int16_t state = radio_.readData(buf, len);
  rssi_ = int16_t(radio_.getRSSI());
  radio_.clearIrqFlags(RADIOLIB_LR11X0_IRQ_ALL);

  // readData leaves the chip in standby, so listening has to be restarted for
  // the next frame.
  resumeReceive();
  return (state == RADIOLIB_ERR_NONE) ? len : 0;
}
