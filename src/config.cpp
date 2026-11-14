#include "config.h"

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

#define CFG_PATH     "/config.json"
#define CFG_TMP_PATH "/config.new"
#define POWER_PATH   "/power"

Config cfg;

// Set whenever the in-memory configuration diverges from the file system.
// Writing is deferred to the end of the session (BLE disconnect or shutdown)
// so repeated save clicks do not wear the flash.
static bool     s_dirty   = false;
static uint32_t s_dirtyMs = 0;

// How long the configuration has to stay unchanged before it is written.
// Generous, because a write costs flash wear and this device runs for days on
// a charge: it only has to beat the user switching off, and switching off saves
// as well.
static const uint32_t kSaveDelayMs = 15000;

// ---------------------------------------------------------------------------
//  The device number comes from the factory programmed serial number in FICR,
//  which is read only and survives reflashing. That makes every unit distinct
//  out of the box, so no one ends up transmitting a shared firmware default
//  because they never opened the settings.
//
//  All 64 bits are folded into the 16 available, and the two values FANET
//  reserves are moved out of the way. The chance of two units colliding is the
//  birthday problem over 65534 values, which is what a 16 bit field allows.
void cfgResetIdentity() {
  cfg.vendorId = FANET_VENDOR_ID;

  const uint32_t a = NRF_FICR->DEVICEID[0];
  const uint32_t b = NRF_FICR->DEVICEID[1];
  const uint32_t folded = a ^ b ^ (a >> 16) ^ (b >> 16);

  cfg.deviceId = uint16_t(folded);
  if (cfg.deviceId == 0x0000) cfg.deviceId = 0x0001;   // reserved
  if (cfg.deviceId == 0xFFFF) cfg.deviceId = 0xFFFE;   // broadcast

  // FLARM and ADS-L are configurable separately, but default to the same
  // number so that a receiver hearing the device on two protocols sees one
  // aircraft instead of two.
  cfg.flarmVendor = cfg.vendorId;
  cfg.flarmDevId  = cfg.deviceId;
  cfg.adslAddr    = 0;          // 0 = derive from the FLARM identity
}

// ---------------------------------------------------------------------------
void cfgSetDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.version  = CFG_VERSION;
  cfgResetIdentity();

  // A name every unit has in common would be shown on everyone's display for
  // everyone who never opens the settings. This one is the device's own, the
  // same four characters as in its Bluetooth name.
  snprintf(cfg.pilot, PILOT_NAME_MAX, "Beacon-%04X", cfg.deviceId);

  // Alternate the two frame formats so receivers that only understand the
  // older one are still served.
  cfg.flarmProto  = 0;

  cfg.buzzerVol  = 80;
  cfg.lowVoltMv  = 3100;
  cfg.autoOffMin = 30;

  cfg.acType  = 1;        // paraglider
  cfg.online  = true;
  cfg.fanetOn = true;
  cfg.flarmOn = false;
  cfg.adslOn  = false;
  cfg.adslTable = 7;      // OGN address table
  cfg.gnssVrtc = true;    // keep the GNSS RTC alive in system-off: the
                          // ephemeris is lost on power-off either way
  cfg.trackMs = 2000;
  cfg.nameMs  = 60000;
  cfg.txPower = 14;

  // Listening before each transmission costs a few milliseconds of receiver
  // current per frame, and in return the airtime limit is 100 s an hour
  // rather than 36 s. Without it FANET has to be thinned out to stay in 1 %.
  cfg.lbt     = true;

  // Reception is off by default. Listening keeps the radio powered
  // continuously, which is the single largest draw on this board.
  cfg.rxOn    = false;
  cfg.rxProto = 0;
}

// ---------------------------------------------------------------------------
void cfgToJson(JsonObject o) {
  o["version"]  = cfg.version;
  o["pilot"]    = cfg.pilot;
  o["vendor"]   = cfg.vendorId;
  o["devId"]    = cfg.deviceId;
  o["fVendor"]  = cfg.flarmVendor;
  o["fDevId"]   = cfg.flarmDevId;
  o["fProto"]   = cfg.flarmProto;
  o["buzzer"]   = cfg.buzzerVol;
  o["lowVoltMv"]= cfg.lowVoltMv;
  o["autoOffMin"]= cfg.autoOffMin;
  o["acType"]   = cfg.acType;
  o["online"]   = cfg.online;
  o["fanet"]    = cfg.fanetOn;
  o["flarm"]    = cfg.flarmOn;
  o["adsl"]     = cfg.adslOn;
  o["adslAddr"] = cfg.adslAddr;
  o["adslTable"]= cfg.adslTable;
  o["gnssVrtc"] = cfg.gnssVrtc;
  o["trackMs"]  = cfg.trackMs;
  o["nameMs"]   = cfg.nameMs;
  o["txPower"]  = cfg.txPower;
  o["lbt"]      = cfg.lbt;
  o["rxOn"]     = cfg.rxOn;
  o["rxProto"]  = cfg.rxProto;
}

// The client keys, as hex strings. Only the file sees these two: a client
// that could read a key could pass for its owner, and one that could write
// one could let itself in.
static void clientsToJson(JsonObject o) {
  JsonArray clients = o["clients"].to<JsonArray>();
  for (uint8_t i = 0; i < cfg.clientCount; ++i) {
    char hex[2 * CLIENT_KEY_LEN + 1];
    for (uint8_t b = 0; b < CLIENT_KEY_LEN; ++b)
      snprintf(hex + 2 * b, 3, "%02X", cfg.clientKeys[i][b]);
    clients.add(hex);
  }
}

static void clientsFromJson(JsonObjectConst o) {
  if (!o["clients"].is<JsonArrayConst>()) return;

  cfg.clientCount = 0;
  for (JsonVariantConst v : o["clients"].as<JsonArrayConst>()) {
    const char *hex = v.as<const char *>();
    if (!hex || strlen(hex) != 2 * CLIENT_KEY_LEN) continue;
    if (cfg.clientCount >= CLIENT_MAX) break;

    bool valid = true;
    for (uint8_t b = 0; b < CLIENT_KEY_LEN; ++b) {
      unsigned byte = 0;
      if (sscanf(hex + 2 * b, "%2x", &byte) != 1) valid = false;
      cfg.clientKeys[cfg.clientCount][b] = uint8_t(byte);
    }
    // A damaged entry is dropped rather than kept as a key of zeros.
    if (valid) ++cfg.clientCount;
  }
}

// Accepts booleans, 0/1 and the strings "true"/"false"/"on"/"off" so a
// sloppy client can never silently drop a toggle.
static bool jsonBool(JsonVariantConst v) {
  if (v.is<bool>()) return v.as<bool>();
  if (v.is<int>())  return v.as<int>() != 0;
  if (v.is<const char *>()) {
    const char *s = v.as<const char *>();
    return !strcmp(s, "true") || !strcmp(s, "1") || !strcmp(s, "on");
  }
  return false;
}

void cfgApplyJson(JsonObjectConst o) {
  if (o["pilot"].is<const char *>()) {
    strncpy(cfg.pilot, o["pilot"], PILOT_NAME_MAX - 1);
    cfg.pilot[PILOT_NAME_MAX - 1] = '\0';
  }
  if (o["vendor"].is<int>())   cfg.vendorId = (uint8_t)o["vendor"];
  if (o["devId"].is<int>())    cfg.deviceId    = (uint16_t)o["devId"];
  if (o["fVendor"].is<int>())  cfg.flarmVendor = (uint8_t)o["fVendor"];
  if (o["fDevId"].is<int>())   cfg.flarmDevId  = (uint16_t)o["fDevId"];
  if (o["fProto"].is<int>()) {
    int v = o["fProto"];
    cfg.flarmProto = (v == 6 || v == 7) ? (uint8_t)v : 0;
  }
  if (o["buzzer"].is<int>())   cfg.buzzerVol   = constrain((int)o["buzzer"], 0, 100);
  if (o["lowVoltMv"].is<int>()) {
    int v = o["lowVoltMv"];
    cfg.lowVoltMv = (v == 0) ? 0 : (uint16_t)constrain(v, 2800, 4200);
  }
  if (o["autoOffMin"].is<int>())
    cfg.autoOffMin = (uint16_t)constrain((int)o["autoOffMin"], 0, 120);
  if (o["acType"].is<int>())   cfg.acType   = constrain((int)o["acType"], 0, 7);
  if (o["online"].is<bool>())  cfg.online   = jsonBool(o["online"]);
  if (o["fanet"].is<bool>())   cfg.fanetOn  = jsonBool(o["fanet"]);
  if (o["flarm"].is<bool>())   cfg.flarmOn  = jsonBool(o["flarm"]);
  if (o["adsl"].is<bool>())    cfg.adslOn   = jsonBool(o["adsl"]);
  if (o["adslAddr"].is<int>()) {
    int64_t v = o["adslAddr"].as<int64_t>();
    cfg.adslAddr = uint32_t(v < 0 ? 0 : (v > 0xFFFFFF ? 0xFFFFFF : v));
  }
  if (o["adslTable"].is<int>()) cfg.adslTable = constrain((int)o["adslTable"], 0, 63);
  if (o["gnssVrtc"].is<bool>()) cfg.gnssVrtc = jsonBool(o["gnssVrtc"]);
  if (o["trackMs"].is<int>())  cfg.trackMs  = constrain((int)o["trackMs"], 1000, 60000);
  if (o["nameMs"].is<int>())   cfg.nameMs   = constrain((int)o["nameMs"], 0, 2000000);
  if (o["txPower"].is<int>())  cfg.txPower  = constrain((int)o["txPower"], -9, 22);
  if (!o["rxOn"].isNull())     cfg.rxOn     = jsonBool(o["rxOn"]);
  if (!o["lbt"].isNull())      cfg.lbt      = jsonBool(o["lbt"]);
  if (o["rxProto"].is<int>())  cfg.rxProto  = (uint8_t)constrain((int)o["rxProto"], 0, 2);

  s_dirty   = true;
  s_dirtyMs = millis();
}

// ---------------------------------------------------------------------------
bool cfgLoad() {
  InternalFS.begin();
  File f(InternalFS);
  if (!f.open(CFG_PATH, FILE_O_READ)) {
    // No configuration means this firmware has not run on the device before,
    // and whatever the file system holds belongs to its predecessor. It is 28
    // KB in all, so what another firmware left behind can be enough to make
    // every later attempt to store the settings fail. Starting clean costs
    // nothing: there is nothing of ours in it yet.
    InternalFS.format();
    return false;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;

  // Defaults are already in place, so absent fields keep their value.
  const int stored = doc["version"].is<int>() ? doc["version"].as<int>() : 0;
  cfgApplyJson(doc.as<JsonObjectConst>());
  clientsFromJson(doc.as<JsonObjectConst>());

  if (stored < CFG_VERSION) {
    // Files written before the identity was derived from the chip carry a
    // fixed address that every unit of that build shared. Take the derived one
    // and keep the rest of the settings.
    cfgResetIdentity();
    cfg.version = CFG_VERSION;
    s_dirty   = true;       // persisted by the next settled save
    s_dirtyMs = millis();
    return true;
  }

  s_dirty = false;          // what was loaded is what is stored
  return true;
}

// Writes the whole configuration to one path, replacing whatever was there.
// FILE_O_WRITE seeks to the end rather than truncating, so an existing file has
// to be removed first: appending would leave two JSON objects in the file, and
// a parser reading the first one would keep restoring the older settings.
static bool writeConfigTo(const char *path) {
  InternalFS.remove(path);

  File f(InternalFS);
  if (!f.open(path, FILE_O_WRITE)) return false;

  JsonDocument doc;
  cfgToJson(doc.to<JsonObject>());
  clientsToJson(doc.as<JsonObject>());
  const size_t written = serializeJson(doc, f);
  f.close();
  return written > 0;
}

// Reads a file back and checks that it parses. A write that reported success
// but produced something unreadable is worse than a failed write, because it
// is only discovered at the next start.
static bool configReadsBack(const char *path) {
  File f(InternalFS);
  if (!f.open(path, FILE_O_READ)) return false;

  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, f);
  f.close();
  return !err && doc["version"].is<int>();
}

bool cfgSave() {
  InternalFS.begin();

  // Written beside the real file and moved into place once it is known to be
  // good, so an interrupted write cannot destroy the previous configuration.
  if (!writeConfigTo(CFG_TMP_PATH))  { Serial.println("config: write failed"); return false; }
  if (!configReadsBack(CFG_TMP_PATH)) {
    Serial.println("config: verify failed");
    InternalFS.remove(CFG_TMP_PATH);
    return false;
  }

  InternalFS.remove(CFG_PATH);
  if (!InternalFS.rename(CFG_TMP_PATH, CFG_PATH)) {
    Serial.println("config: rename failed");
    return false;
  }

  s_dirty = false;
  return true;
}

bool cfgSaveIfDirty() {
  if (!s_dirty) return false;
  return cfgSave();
}

// ---------------------------------------------------------------------------
//  Power intent
// ---------------------------------------------------------------------------
//  One byte, '1' or '0'. Written when the device is switched on or off, which
//  is a few times a day at most.
static int8_t s_powerIntent = -1;            // not read yet

bool cfgPowerIntent() {
  if (s_powerIntent < 0) {
    InternalFS.begin();
    s_powerIntent = 1;
    File f(InternalFS);
    if (f.open(POWER_PATH, FILE_O_READ)) {
      if (f.read() == '0') s_powerIntent = 0;
      f.close();
    }
  }
  return s_powerIntent != 0;
}

void cfgSetPowerIntent(bool on) {
  if (cfgPowerIntent() == on) return;

  // Removed first: writing appends rather than replacing.
  InternalFS.remove(POWER_PATH);
  File f(InternalFS);
  if (f.open(POWER_PATH, FILE_O_WRITE)) {
    f.write(uint8_t(on ? '1' : '0'));
    f.close();
  }
  s_powerIntent = on ? 1 : 0;
}

// ---------------------------------------------------------------------------
//  Confirmed clients
// ---------------------------------------------------------------------------
void cfgClientAdd(const uint8_t key[CLIENT_KEY_LEN]) {
  if (cfg.clientCount < CLIENT_MAX) {
    memcpy(cfg.clientKeys[cfg.clientCount++], key, CLIENT_KEY_LEN);
  } else {
    // Full: the oldest entry goes. A client that is still in use will be
    // confirmed again, one that is not disappears on its own.
    memmove(cfg.clientKeys[0], cfg.clientKeys[1], (CLIENT_MAX - 1) * CLIENT_KEY_LEN);
    memcpy(cfg.clientKeys[CLIENT_MAX - 1], key, CLIENT_KEY_LEN);
  }

  s_dirty   = true;
  s_dirtyMs = millis();
}

// Saving once the changes have settled, rather than only at the end of the
// session: a flat battery, a crash or a client that never disconnects cleanly
// must not cost the configuration.
CfgSaveResult cfgSaveIfSettled() {
  if (!s_dirty) return CfgSaveResult::None;
  if (millis() - s_dirtyMs < kSaveDelayMs) return CfgSaveResult::None;
  return cfgSave() ? CfgSaveResult::Ok : CfgSaveResult::Failed;
}
