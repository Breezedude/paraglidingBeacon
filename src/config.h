#pragma once
// ===========================================================================
//  Configuration: readable and writable as JSON over BLE, persisted in the
//  internal filesystem.
//
//  To extend, add the field to Config, cfgToJson() and cfgApplyJson(). Unknown
//  JSON fields are ignored and absent ones keep their value, so older clients
//  keep working as the schema grows.
// ===========================================================================

#include <Arduino.h>
#include <ArduinoJson.h>

#define CFG_VERSION      2
#define PILOT_NAME_MAX   46

// Clients that may configure the device. Each was confirmed at the button and
// handed a key, which it shows it holds on every later connection. Four covers
// a phone, a tablet and a laptop with one to spare; beyond that the oldest is
// dropped.
#define CLIENT_MAX       4
#define CLIENT_KEY_LEN   32

// FANET manufacturer id assigned to this project. The lower two bytes of the
// address are the device number and come from the chip, see cfgResetIdentity().
#define FANET_VENDOR_ID  0xBD

// Firmware version, reported to configuration clients. Raise on every release.
#define FW_VERSION       "0.3.0"

struct Config {
  uint8_t  version;
  char     pilot[PILOT_NAME_MAX];
  uint8_t  vendorId;     // FANET manufacturer id
  uint16_t deviceId;     // FANET device number, derived from the chip
  uint8_t  flarmVendor;  // FLARM identity, independent of FANET
  uint16_t flarmDevId;
  uint8_t  flarmProto;   // 6 = V6 only, 7 = V7 only, 0 = alternate
  uint32_t adslAddr;     // ADS-L device address, 0 = derive from FLARM identity
  uint8_t  adslTable;    // ADS-L address mapping table, 6 bits (5 ICAO, 6 FLARM, 7 OGN, 8 FANET)
  bool     gnssVrtc;     // keep the GNSS backup domain powered after system-off
  uint8_t  buzzerVol;    // 0..100 %
  uint16_t lowVoltMv;    // low voltage cutoff, 0 disables
  uint16_t autoOffMin;   // motion timeout in minutes, 0 disables
  uint8_t  acType;       // aircraft type 0..7
  bool     online;       // online tracking flag
  bool     fanetOn;
  bool     flarmOn;
  bool     adslOn;
  uint16_t trackMs;      // tracking interval
  uint16_t nameMs;       // name interval
  int8_t   txPower;      // dBm
  bool     lbt;          // listen before talk, see airtime.h
  bool     rxOn;         // listen for other aircraft, off saves current
  uint8_t  rxProto;
  uint8_t  clientCount;
  uint8_t  clientKeys[CLIENT_MAX][CLIENT_KEY_LEN];   // confirmed clients
};

extern Config cfg;

// Outcome of a deferred save, so a failure can be reported rather than lost.
enum class CfgSaveResult { None, Ok, Failed };

void cfgSetDefaults();

// Restores the identity of all three protocols to the value derived from the
// chip. Called for a fresh configuration and when migrating an older one.
void cfgResetIdentity();
bool cfgLoad();
bool cfgSave();

// True once cfgApplyJson() changed anything. Flash is only written when the
// session ends (BLE disconnect or power-off) to keep the write wear low.
bool cfgSaveIfDirty();

// Called from the main loop: writes the configuration once it has been
// unchanged for a moment. Returns None while there is nothing to do.
CfgSaveResult cfgSaveIfSettled();

// Stores the key of a client that the user confirmed at the button.
void cfgClientAdd(const uint8_t key[CLIENT_KEY_LEN]);

// Serialises the settings into an existing JsonObject. The client keys are
// not part of it: they are stored with the settings but are nobody's to read
// or write over the air.
void cfgToJson(JsonObject o);
// Applies only the fields present in the object.
void cfgApplyJson(JsonObjectConst o);
