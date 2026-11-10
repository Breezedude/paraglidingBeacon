#pragma once
// ===========================================================================
//  LR1110 transceiver
// ---------------------------------------------------------------------------
//  One radio serves all three protocols, so the modem is switched between LoRa
//  and FSK many times a second. This class owns that state and takes finished
//  frames from the protocol layer; it does not know how they were built, nor
//  when they are due.
// ===========================================================================

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#include "airtime.h"

// The library changes modem by resetting the chip, which costs a third of a
// second each time. Switching the packet type is a single command, but one the
// library keeps to itself, so it is made reachable here.
class Lr1110Modem : public LR1110 {
 public:
  using LR1110::LR1110;
  using LR11x0::setPacketType;
};

class Radio {
 public:
  enum class Mode : uint8_t { None, Fanet, Flarm, Adsl };

  // What became of a frame.
  enum class Result : uint8_t {
    Sent,
    NoBudget,   // the airtime account is spent; try again in a later second
    Busy,       // the channel is in use or has to rest; retryDelayMs() says when
    Failed,     // the radio refused
  };

  Radio();

  bool begin(int8_t txPower);
  void setTxPower(int8_t txPower);

  // How the band is shared, see airtime.h. With `lbt` the radio listens
  // before every frame and the polite limits apply, without it the 1 % duty
  // cycle. The switched on slotted protocols are what FANET gives way to.
  void setAccess(bool lbt, bool flarmOn, bool adslOn);

  Result sendFanet(const uint8_t* data, size_t len);

  // Always on the lower channel, which is where receivers listen for FLARM in
  // the first slot of the second.
  Result sendFlarm(const uint8_t* data, size_t len);

  // ADS-L uses both channels; which one is right depends on the slot the
  // caller is transmitting in.
  Result sendAdsl(const uint8_t* data, size_t len, bool upperChannel);

  // After Result::Busy: how long to wait before trying again.
  uint32_t retryDelayMs() const { return retryMs_; }

  // ---- receive ----------------------------------------------------------
  // Selects the protocol to listen for. Mode::None leaves the radio idle,
  // which is what keeps the current draw down when reception is switched off.
  bool listen(Mode mode);
  Mode listening() const { return rxMode_; }

  // Returns the length of a frame that arrived since the last call, or 0.
  size_t poll(uint8_t* buf, size_t cap);
  int16_t lastRssi() const { return rssi_; }

  // Puts the chip into its lowest power state and lets go of the bus, for
  // switching the device off. begin() is needed again afterwards.
  void shutdown();

  bool     ready()       const { return ready_; }
  int      lastStatus()  const { return status_; }
  uint32_t fanetCount()  const { return fanetCount_; }
  uint32_t flarmCount()  const { return flarmCount_; }
  uint32_t adslCount()   const { return adslCount_; }
  uint32_t skippedCount() const { return skipped_; }   // frames over budget
  uint32_t busyCount()    const { return busy_; }      // channel found in use

 private:
  bool initChip();
  bool select(Mode mode, float frequencyMhz);
  bool apply(Mode mode, float frequencyMhz);
  bool applyLora(float frequencyMhz);
  bool applyFsk(const uint8_t* syncWord, size_t airBytes, float frequencyMhz);
  Result send(Mode mode, float frequencyMhz, const uint8_t* data, size_t len,
              uint32_t costUs, uint32_t reserveUs);
  bool channelClear();
  bool resumeReceive();

  SPIClass    spi_;
  Lr1110Modem radio_;
  Mode     mode_       = Mode::None;   // what the modem is configured for
  float    frequency_  = 0.0f;
  int8_t   power_      = 14;
  bool     chipUp_     = false;        // reset and calibrated
  bool     ready_      = false;
  int      status_     = 0;
  uint32_t fanetCount_ = 0;
  uint32_t flarmCount_ = 0;
  uint32_t adslCount_  = 0;
  Mode     rxMode_     = Mode::None;
  bool     receiving_  = false;
  int16_t  rssi_       = 0;

  airtime::Budget  budget_;
  airtime::OffTime offTime_;
  bool     lbt_        = false;
  uint32_t reserveUs_  = 0;      // what FANET leaves for FLARM and ADS-L
  uint32_t retryMs_    = 0;
  uint32_t skipped_    = 0;
  uint32_t busy_       = 0;
};
