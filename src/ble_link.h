#pragma once
// ===========================================================================
//  Bluetooth LE link
// ---------------------------------------------------------------------------
//  Two services, so that navigation software and the configuration tool do
//  not have to share a channel, and up to two centrals at the same time:
//
//    Nordic UART Service      NMEA sentences, open to anyone who connects.
//                             This is what cockpit applications expect.
//
//    Configuration service    one JSON object per line in both directions.
//                             A client has to show a key, which it is given
//                             after a press on the device's button.
//
//  A central receives a stream only while it is subscribed to it, so nothing
//  is sent that nobody reads.
//
//  Protocol version 1. Before a client is let in, only two commands count:
//    {"cmd":"auth","mac":"<hex>"}     HMAC-SHA-256 of the nonce under its key
//    {"cmd":"pair"}                   asks for a key; needs the button
//  and anything else is answered with
//    {"r":"auth","ok":false,"nonce":"<hex>","dev":"<hex>"}
//
//  Afterwards, each optionally with an "id" that is echoed in the reply:
//    {"cmd":"hello","v":1}            opens the session
//    {"cmd":"get"}                    whole configuration
//    {"cmd":"set","cfg":{...}}        partial update
//    {"cmd":"status"}                 live status, once
//    {"cmd":"beep","vol":0..100}      preview tone, changes nothing
//    {"cmd":"reboot"}
//    {"cmd":"bootloader"}             enter the UF2 bootloader for updates
//
//  Replies:
//    {"r":"hello","v":1,"fw":"...","mtu":244,"mot":true,"radio":true,"caps":[...]}
//    {"r":"cfg","cfg":{...}}
//    {"r":"ok","cmd":"<cmd>"}
//    {"r":"err","cmd":"<cmd>","msg":"..."}
//
//  Events, sent without being asked for:
//    {"r":"auth","ok":false,"wait":true}   waiting for the button
//    {"r":"auth","ok":true[,"key":"<hex>"]} let in; the key only when new
//    {"r":"status",...}               once a second
//    {"r":"traffic"|"weather"|"name",...}
//    {"r":"saved","ok":true|false}    outcome of a deferred configuration write
//
//  Unknown commands get an error, unknown fields are ignored, and a client is
//  expected to ignore events it does not know. That is what lets either side
//  grow without breaking the other.
// ===========================================================================

#include <Arduino.h>
#include <ArduinoJson.h>

#include "proto/report.h"

// Live values, filled in by the application.
struct LiveStatus {
  bool     fix;
  uint8_t  sats;
  double   lat, lon;
  float    altM, speedKmh, climbMs, headingDeg;
  float    battV;
  uint8_t  battPct;
  bool     charging;        // USB VBUS present
  uint32_t fanetTx;
  uint32_t flarmTx;
  uint32_t adslTx;
  uint32_t txSkipped;       // frames held back by the airtime account
  uint32_t txBusy;          // times the channel was found in use
  int      flarmState;      // last radio status code, 0 is success
  bool     flarmV7;         // whether the last frame used the newer format
  bool     motionOk;        // accelerometer answers
  uint32_t noMotionS;       // seconds since movement was last detected
  bool     radioOk;
};

void bleBegin(const char *devName);

// Called from the main loop. Moves queued data out and handles what came in.
// Nothing in this module waits for the radio: what does not fit into a queue
// is dropped, so a slow or absent central cannot hold up the transmit timing.
void blePoll();

// Forwards a sentence to every central subscribed to the NMEA stream.
void bleWriteNmea(const char *line, size_t len);

void bleSetBattery(uint8_t percent);

// ---- configuration clients -------------------------------------------------
// All of these go to confirmed clients only.

void bleSendStatus();

// Tells the client whether a deferred configuration write succeeded.
void bleSendSaved(bool ok);

// Decoded receptions. ageSec is how long ago the report was heard: zero for
// something arriving now, and the real age when the cache is replayed to a
// client that has just connected. Without it a client would show minute old
// traffic as current.
void bleSendTraffic(const proto::Traffic &t, unsigned tracked, uint32_t ageSec = 0);
void bleSendWeather(const proto::Weather &w, uint32_t ageSec = 0);
void bleSendName(const proto::Name &n, uint32_t ageSec = 0);

// Bytes that can still be queued for the configuration clients, zero when
// there is none. Lets a long replay pace itself instead of overflowing.
size_t bleEventRoom();

// Called when a client opens a session, so that what was heard before it
// connected can be handed over.
typedef void (*ReplayCb)();
void bleSetReplayCb(ReplayCb fn);

// ---- confirmation at the button --------------------------------------------
// True while a client is waiting to be confirmed. The confirmation is a short
// press on the device itself, which is the point: it cannot be given remotely.
bool bleAuthPending();

// Confirms the waiting client and hands it its key. Returns false when there
// was none to confirm, or more than one: a press cannot say which of two it
// means, so both are turned away rather than letting a stranger in alongside
// the owner.
bool bleAuthConfirm();

// Called when a client asks for a key, so the device can ask for attention.
typedef void (*AuthPromptCb)();
void bleSetAuthPromptCb(AuthPromptCb fn);

using StatusProvider = void (*)(LiveStatus &);
void bleSetStatusProvider(StatusProvider fn);

// Called after a configuration change, for instance to re-apply transmit power.
using ConfigChangedCb = void (*)();
void bleSetConfigChangedCb(ConfigChangedCb fn);

// Preview tone for the volume slider; does not touch the configuration.
using BeepCb = void (*)(uint8_t volume);
void bleSetBeepCb(BeepCb fn);
