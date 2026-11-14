#include "ble_link.h"

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <bluefruit.h>

#include "byte_ring.h"
#include "config.h"
#include "sha256.h"

namespace {

constexpr uint8_t kProtocolVersion = 1;

// A navigation app and the configuration page at the same time.
constexpr uint8_t kMaxLinks = 2;

// SoftDevice sizing. The event length is what each connection is given per
// interval; thirty milliseconds carries several full packets and leaves room
// for the second link, where the library's "maximum" preset would have the two
// compete for the same time.
constexpr uint16_t kMaxMtu      = 247;
constexpr uint16_t kMaxPayload  = kMaxMtu - 3;
constexpr uint16_t kEventLength = 24;      // 1.25 ms units
constexpr uint8_t  kNotifyQueue = 3;       // notifications the stack holds per link

// Asked for a few seconds after connecting, once discovery is through. Slow
// enough to let the radio rest between events, fast enough that a client
// limited to twenty byte packets still gets the whole NMEA stream.
constexpr uint16_t kIntervalMin        = 48;    // 60 ms, 1.25 ms units
constexpr uint16_t kIntervalMax        = 72;    // 90 ms
constexpr uint16_t kSupervisionTimeout = 400;   // 4 s, 10 ms units
constexpr uint32_t kParamDelayMs       = 5000;

// Advertising: brisk while someone is likely to be looking for the device,
// then about once a second, which is still found within a few seconds.
constexpr uint16_t kAdvFast        = 244;       // 152.5 ms, 0.625 ms units
constexpr uint16_t kAdvSlow        = 1636;      // 1022.5 ms
constexpr uint16_t kAdvFastSeconds = 30;

// How long a client waits for the button, and how often it is told that it
// is waiting. Repeated rather than sent once, so a missed notification does
// not leave the client guessing.
constexpr uint32_t kAuthTimeoutMs = 30000;
constexpr uint32_t kAuthNoticeMs  = 1500;

constexpr size_t  kNonceLength = 16;
constexpr size_t  kAddressLength = 6;
constexpr uint8_t kMaxBonds    = 8;

// 88C00001-13A4-4AAB-998F-26EE246F7E7B and its two characteristics, least
// significant byte first as the stack wants them.
const uint8_t kConfigServiceUuid[16] = {0x7B, 0x7E, 0x6F, 0x24, 0xEE, 0x26, 0x8F, 0x99,
                                        0xAB, 0x4A, 0xA4, 0x13, 0x01, 0x00, 0xC0, 0x88};
const uint8_t kConfigRxUuid[16]      = {0x7B, 0x7E, 0x6F, 0x24, 0xEE, 0x26, 0x8F, 0x99,
                                        0xAB, 0x4A, 0xA4, 0x13, 0x02, 0x00, 0xC0, 0x88};
const uint8_t kConfigTxUuid[16]      = {0x7B, 0x7E, 0x6F, 0x24, 0xEE, 0x26, 0x8F, 0x99,
                                        0xAB, 0x4A, 0xA4, 0x13, 0x03, 0x00, 0xC0, 0x88};

BLEDis deviceInfo;
BLEBas batteryService;

BLEService        nmeaService(BLEUART_UUID_SERVICE);
BLECharacteristic nmeaTx(BLEUART_UUID_CHR_TXD);
BLECharacteristic nmeaRx(BLEUART_UUID_CHR_RXD);

BLEService        configService(kConfigServiceUuid);
BLECharacteristic configRx(kConfigRxUuid);
BLECharacteristic configTx(kConfigTxUuid);

StatusProvider  s_status     = nullptr;
ConfigChangedCb s_cfgCb      = nullptr;
BeepCb          s_beep       = nullptr;
ReplayCb        s_replay     = nullptr;
AuthPromptCb    s_authPrompt = nullptr;

// The device's own Bluetooth address in hex. A client files its key under it.
char s_deviceId[2 * kAddressLength + 1] = "";

// ---------------------------------------------------------------------------
//  One connection. The stack numbers its connections from zero, so with two
//  links the handle is the index.
// ---------------------------------------------------------------------------
struct Link {
  // Written by the Bluetooth task as events arrive, read by the main loop.
  volatile bool     up         = false;
  volatile uint8_t  generation = 0;       // counts connections on this handle
  volatile uint32_t notifyDone = 0;       // notifications the stack has sent
  ByteRing<1024>    rx;                   // configuration bytes as they arrive

  // Main loop only.
  bool     active         = false;
  uint8_t  seenGeneration = 0;
  bool     pending        = false;        // waiting for the button
  bool     open           = false;        // has shown a key, the session is live
  bool     paramsAsked    = false;
  bool     haveNonce      = false;
  uint8_t  nonce[kNonceLength];
  uint32_t connectedMs    = 0;
  uint32_t pendingSinceMs = 0;
  uint32_t noticeMs       = 0;
  uint32_t notifySent     = 0;
  ByteRing<2048> json;                    // waiting to go out as configuration
  ByteRing<2048> nmea;                    // waiting to go out as NMEA
  char     line[768];
  uint16_t lineLength   = 0;
  bool     lineOverflow = false;
};

Link s_links[kMaxLinks];

// Scratch space for one outgoing message. Static rather than on the stack of
// the loop task, which has little to spare.
uint8_t s_text[768];

uint16_t payloadOf(uint16_t handle) {
  BLEConnection *conn = Bluefruit.Connection(handle);
  return conn ? uint16_t(conn->getMtu() - 3) : uint16_t(BLE_GATT_ATT_MTU_DEFAULT - 3);
}

// ---------------------------------------------------------------------------
//  Small helpers
// ---------------------------------------------------------------------------
void toHex(const uint8_t *data, size_t length, char *out) {
  for (size_t i = 0; i < length; ++i) snprintf(out + 2 * i, 3, "%02x", data[i]);
}

bool fromHex(const char *text, uint8_t *out, size_t length) {
  if (!text || strlen(text) != 2 * length) return false;
  for (size_t i = 0; i < length; ++i) {
    unsigned value = 0;
    if (sscanf(text + 2 * i, "%2x", &value) != 1) return false;
    out[i] = uint8_t(value);
  }
  return true;
}

// From the stack's hardware generator. It hands out what it has collected,
// which after a quiet spell is everything asked for here; the wait covers the
// case that it is not.
bool fillRandom(uint8_t *out, size_t length) {
  const uint32_t start = millis();
  size_t done = 0;
  while (done < length) {
    uint8_t available = 0;
    sd_rand_application_bytes_available_get(&available);
    if (available) {
      const size_t  left = length - done;
      const uint8_t take = available < left ? available : uint8_t(left);
      if (sd_rand_application_vector_get(out + done, take) == NRF_SUCCESS) done += take;
    } else {
      if (millis() - start > 200) return false;
      delay(1);
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
//  Events, in the Bluetooth task
// ---------------------------------------------------------------------------
//  Only flags and counters are touched here. Everything that has consequences
//  happens in the main loop, where it cannot interleave with the transmitter.
void onBleEvent(ble_evt_t *evt) {
  const uint16_t handle = evt->evt.common_evt.conn_handle;
  if (handle >= kMaxLinks) return;
  Link &link = s_links[handle];

  switch (evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
      link.notifyDone = 0;
      link.generation = link.generation + 1;
      link.up         = true;
      break;

    case BLE_GAP_EVT_DISCONNECTED:
      link.up = false;
      break;

    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
      link.notifyDone = link.notifyDone + evt->evt.gatts_evt.params.hvn_tx_complete.count;
      break;

    default:
      break;
  }
}

// Also in the Bluetooth task. A chunk that does not fit is dropped whole; the
// line it belonged to then fails to parse and is reported as such.
void onConfigWrite(uint16_t handle, BLECharacteristic *, uint8_t *data, uint16_t length) {
  if (handle < kMaxLinks) s_links[handle].rx.push(data, length);
}

// ---------------------------------------------------------------------------
//  Sending
// ---------------------------------------------------------------------------
//  Data waits in a queue per link and stream and is handed to the stack only
//  when the stack has room, which is known from counting what went in and what
//  it reported as sent. The library call would otherwise wait for room itself,
//  up to a tenth of a second per packet, in the loop that also has to hit the
//  transmit slots of the radio.
void drain(uint16_t handle, Link &link) {
  static uint8_t chunk[kMaxPayload];

  while (int32_t(link.notifySent - link.notifyDone) < kNotifyQueue) {
    // Replies and status before the NMEA stream: they are what a person is
    // waiting for, and they are short.
    const bool fromJson = !link.json.empty();
    if (!fromJson && link.nmea.empty()) return;

    ByteRing<2048>    &queue = fromJson ? link.json : link.nmea;
    BLECharacteristic &chr   = fromJson ? configTx : nmeaTx;

    const size_t payload = payloadOf(handle);
    const size_t count   = queue.peek(chunk, payload < sizeof(chunk) ? payload : sizeof(chunk));

    if (!chr.notify(handle, chunk, uint16_t(count))) {
      // Unsubscribed in the meantime, or the link is going down.
      queue.clear();
      continue;
    }
    queue.drop(count);
    ++link.notifySent;
  }
}

// Serialises into s_text with a closing newline. Zero when it does not fit,
// which is said out loud: a message cut short would run into the next one.
size_t render(const JsonDocument &doc) {
  if (measureJson(doc) + 1 > sizeof(s_text)) {
    Serial.println("ble: message too long, dropped");
    return 0;
  }
  size_t length = serializeJson(doc, reinterpret_cast<char *>(s_text), sizeof(s_text));
  s_text[length++] = '\n';
  return length;
}

// To one link, whatever its state: the caller decides whether it may hear it.
void sendTo(uint16_t handle, Link &link, const JsonDocument &doc) {
  if (!link.active || !configTx.notifyEnabled(handle)) return;
  const size_t length = render(doc);
  if (length) link.json.push(s_text, length);
}

bool listening(uint16_t handle, const Link &link) {
  return link.active && link.open && configTx.notifyEnabled(handle);
}

bool anyListening() {
  for (uint16_t h = 0; h < kMaxLinks; ++h)
    if (listening(h, s_links[h])) return true;
  return false;
}

// To every confirmed client that is subscribed.
void broadcast(const JsonDocument &doc) {
  const size_t length = render(doc);
  if (!length) return;
  for (uint16_t h = 0; h < kMaxLinks; ++h)
    if (listening(h, s_links[h])) s_links[h].json.push(s_text, length);
}

// Pushes out what is queued before the processor is reset under it.
void flush(uint16_t handle, Link &link) {
  const uint32_t start = millis();
  while (millis() - start < 300) {
    drain(handle, link);
    if (link.json.empty() && int32_t(link.notifySent - link.notifyDone) <= 0) break;
    delay(5);
  }
}

// ---------------------------------------------------------------------------
//  Who may configure
// ---------------------------------------------------------------------------
//  Reading the position stream is open. Changing the device is not.
//
//  A client is let in once it has shown that it holds a key this device handed
//  out, and a key is handed out only after a press on the button. The key
//  never travels again after that: on each connection the device sends a
//  number it has just made up, and the client answers with the HMAC of that
//  number under its key. Someone listening to a later session therefore learns
//  nothing they could replay.
//
//  This is done here rather than with Bluetooth's own pairing because the page
//  runs in a browser, and browsers on Windows and Linux do not pair on their
//  own. The price is that the link itself is not encrypted: the one moment a
//  key is exposed is when it is handed over, to someone already recording that
//  connection. Plain Bluetooth pairing without a passkey has the same gap.

// Everything an unconfirmed client is told: that it has to prove itself, and
// the number to do it with.
void sendChallenge(uint16_t handle, Link &link) {
  if (!link.haveNonce) {
    if (!fillRandom(link.nonce, sizeof(link.nonce))) return;
    link.haveNonce = true;
  }
  char nonce[2 * kNonceLength + 1];
  toHex(link.nonce, sizeof(link.nonce), nonce);

  JsonDocument doc;
  doc["r"]     = "auth";
  doc["ok"]    = false;
  doc["nonce"] = nonce;
  doc["dev"]   = s_deviceId;
  sendTo(handle, link, doc);
}

void sendWaiting(uint16_t handle, Link &link) {
  JsonDocument doc;
  doc["r"]    = "auth";
  doc["ok"]   = false;
  doc["wait"] = true;
  sendTo(handle, link, doc);
}

bool answersChallenge(const Link &link, const char *macHex) {
  uint8_t mac[crypto::kSha256Length];
  if (!link.haveNonce || !fromHex(macHex, mac, sizeof(mac))) return false;

  bool match = false;
  for (uint8_t i = 0; i < cfg.clientCount; ++i) {
    uint8_t expected[crypto::kSha256Length];
    crypto::hmacSha256(cfg.clientKeys[i], CLIENT_KEY_LEN, link.nonce, sizeof(link.nonce), expected);
    if (crypto::equalConstantTime(expected, mac, sizeof(mac))) match = true;
  }
  return match;
}

// Commands from a client that has not been let in. Only two mean anything.
void handleLocked(uint16_t handle, Link &link, JsonDocument &in) {
  const char *cmd = in["cmd"] | "";

  if (!strcmp(cmd, "auth")) {
    const bool ok = answersChallenge(link, in["mac"] | "");
    link.haveNonce = false;               // a number is good for one answer
    if (!ok) {
      sendChallenge(handle, link);
      return;
    }
    link.open    = true;
    link.pending = false;
    JsonDocument doc;
    doc["r"]  = "auth";
    doc["ok"] = true;
    sendTo(handle, link, doc);

  } else if (!strcmp(cmd, "pair")) {
    if (!link.pending) {
      link.pending        = true;
      link.pendingSinceMs = millis();
      if (s_authPrompt) s_authPrompt();
    }
    link.noticeMs = millis();
    sendWaiting(handle, link);

  } else {
    sendChallenge(handle, link);
  }
}

// The library accepts a pairing request from anyone and stores a file for
// each, in the file system the configuration lives in. Nothing here relies on
// those bonds, so they are thrown out when they pile up.
void limitBonds() {
  using namespace Adafruit_LittleFS_Namespace;

  uint8_t count = 0;
  File dir(BOND_DIR_PRPH, FILE_O_READ, InternalFS);
  File file(InternalFS);
  while ((file = dir.openNextFile(FILE_O_READ))) {
    ++count;
    file.close();
  }
  dir.close();

  if (count > kMaxBonds) Bluefruit.Periph.clearBonds();
}

// ---------------------------------------------------------------------------
//  Commands
// ---------------------------------------------------------------------------
void fillStatus(JsonDocument &doc) {
  LiveStatus st{};
  s_status(st);

  doc["r"]   = "status";
  doc["fix"] = st.fix;
  doc["sat"] = st.sats;
  if (st.fix) {
    doc["lat"] = serialized(String(st.lat, 6));
    doc["lon"] = serialized(String(st.lon, 6));
    doc["alt"] = (int)st.altM;
    doc["spd"] = serialized(String(st.speedKmh, 1));
    doc["clb"] = serialized(String(st.climbMs, 1));
    doc["hdg"] = (int)st.headingDeg;
  }
  doc["bat"]   = serialized(String(st.battV, 2));
  doc["batP"]  = st.battPct;
  doc["chg"]   = st.charging;
  doc["tx"]    = st.fanetTx;      // FANET frame count
  doc["txF"]   = st.flarmTx;      // FLARM frame count
  doc["txA"]   = st.adslTx;       // ADS-L frame count
  doc["skip"]  = st.txSkipped;    // held back by the airtime account
  doc["busy"]  = st.txBusy;       // channel found in use
  doc["fSt"]   = st.flarmState;   // last radio status code
  doc["fV7"]   = st.flarmV7;      // last frame used the newer format
  doc["noMot"] = st.noMotionS;    // seconds since movement
}

void handleCommand(uint16_t handle, Link &link, JsonDocument &in) {
  JsonDocument out;

  const char *cmd = in["cmd"] | "";
  if (!in["id"].isNull()) out["id"] = in["id"];

  if (!strcmp(cmd, "hello")) {
    // What does not change during a session is said once, here, instead of
    // being repeated in every status message.
    out["r"]   = "hello";
    out["v"]   = kProtocolVersion;
    out["fw"]  = FW_VERSION;
    out["mtu"] = payloadOf(handle);    // largest write the client may send
    if (s_status) {
      LiveStatus st{};
      s_status(st);
      out["mot"]   = st.motionOk;
      out["radio"] = st.radioOk;
    }
    JsonArray caps = out["caps"].to<JsonArray>();
    caps.add("fanet");
    caps.add("flarm");
    caps.add("adsl");
    caps.add("rx");
    sendTo(handle, link, out);
    if (s_replay) s_replay();

  } else if (!strcmp(cmd, "get")) {
    out["r"] = "cfg";
    cfgToJson(out["cfg"].to<JsonObject>());
    sendTo(handle, link, out);

  } else if (!strcmp(cmd, "set")) {
    if (!in["cfg"].is<JsonObjectConst>()) {
      out["r"]   = "err";
      out["cmd"] = cmd;
      out["msg"] = "missing cfg";
    } else {
      cfgApplyJson(in["cfg"].as<JsonObjectConst>());
      if (s_cfgCb) s_cfgCb();
      out["r"]   = "ok";
      out["cmd"] = cmd;
    }
    sendTo(handle, link, out);

  } else if (!strcmp(cmd, "status")) {
    if (s_status) {
      fillStatus(out);
      sendTo(handle, link, out);
    }

  } else if (!strcmp(cmd, "beep")) {
    // Deliberately does not use the stored volume, so the slider can be
    // auditioned before it is applied.
    const uint8_t volume = (uint8_t)constrain((int)(in["vol"] | (int)cfg.buzzerVol), 0, 100);
    out["r"]   = "ok";
    out["cmd"] = cmd;
    sendTo(handle, link, out);
    if (s_beep) s_beep(volume);

  } else if (!strcmp(cmd, "reboot") || !strcmp(cmd, "bootloader")) {
    cfgSaveIfDirty();                 // do not lose the session changes
    out["r"]   = "ok";
    out["cmd"] = cmd;
    sendTo(handle, link, out);
    flush(handle, link);
    // The bootloader shows up as a USB drive, ready for a new firmware.uf2.
    if (!strcmp(cmd, "bootloader")) enterUf2Dfu();
    NVIC_SystemReset();

  } else {
    out["r"]   = "err";
    out["cmd"] = cmd;
    out["msg"] = "unknown command";
    sendTo(handle, link, out);
  }
}

void handleLine(uint16_t handle, Link &link, char *text) {
  while (*text == ' ' || *text == '\t') ++text;
  if (!*text) return;

  JsonDocument in;
  if (deserializeJson(in, text)) {
    // Said to anyone: it gives nothing away and tells a client that is still
    // being written what went wrong.
    JsonDocument out;
    out["r"]   = "err";
    out["msg"] = "not valid JSON";
    sendTo(handle, link, out);
    return;
  }

  if (link.open) handleCommand(handle, link, in);
  else           handleLocked(handle, link, in);
}

void readCommands(uint16_t handle, Link &link) {
  int byte;
  while ((byte = link.rx.pop()) >= 0) {
    if (byte == '\r' || byte == '\n') {
      if (link.lineLength && !link.lineOverflow) {
        link.line[link.lineLength] = '\0';
        handleLine(handle, link, link.line);
      }
      link.lineLength   = 0;
      link.lineOverflow = false;
    } else if (link.lineLength < sizeof(link.line) - 1) {
      link.line[link.lineLength++] = char(byte);
    } else {
      link.lineOverflow = true;          // the rest of this line is discarded
    }
  }
}

// ---------------------------------------------------------------------------
//  Link housekeeping
// ---------------------------------------------------------------------------
void openLink(Link &link) {
  link.active         = true;
  link.seenGeneration = link.generation;
  link.pending        = false;
  link.open           = false;
  link.paramsAsked    = false;
  link.haveNonce      = false;
  link.connectedMs    = millis();
  link.notifySent     = 0;
  link.lineLength     = 0;
  link.lineOverflow   = false;
  link.rx.clear();
  link.json.clear();
  link.nmea.clear();
}

void closeLink(Link &link) {
  // The client went away: write what it changed now rather than waiting for
  // the changes to settle.
  if (link.open) cfgSaveIfDirty();
  link.active  = false;
  link.open    = false;
  link.pending = false;
  link.json.clear();
  link.nmea.clear();
}

void requestSlowerInterval(uint16_t handle) {
  BLEConnection *conn = Bluefruit.Connection(handle);
  if (!conn || conn->getConnectionInterval() >= kIntervalMin) return;

  const ble_gap_conn_params_t params = {
      .min_conn_interval = kIntervalMin,
      .max_conn_interval = kIntervalMax,
      .slave_latency     = 0,
      .conn_sup_timeout  = kSupervisionTimeout,
  };
  // The central is free to refuse; nothing depends on it.
  (void)sd_ble_gap_conn_param_update(handle, &params);
}

// Advertising stops with every connection. It is started again here for as
// long as a link is free, so the second client can still find the device.
void keepAdvertising() {
  if (Bluefruit.Periph.connected() < kMaxLinks && !Bluefruit.Advertising.isRunning())
    Bluefruit.Advertising.start(0);
}

}  // namespace

// ---------------------------------------------------------------------------
void bleSetAuthPromptCb(AuthPromptCb fn)       { s_authPrompt = fn; }
void bleSetStatusProvider(StatusProvider fn)   { s_status = fn; }
void bleSetConfigChangedCb(ConfigChangedCb fn) { s_cfgCb = fn; }
void bleSetBeepCb(BeepCb fn)                   { s_beep = fn; }
void bleSetReplayCb(ReplayCb fn)               { s_replay = fn; }
void bleSetBattery(uint8_t percent)            { batteryService.write(percent); }

void blePoll() {
  static uint32_t lastAdvertisingCheckMs = 0;
  const uint32_t  now     = millis();
  bool            changed = false;

  for (uint16_t h = 0; h < kMaxLinks; ++h) {
    Link &link = s_links[h];

    // A link that dropped and came back between two passes is a new one.
    if (link.active && (!link.up || link.generation != link.seenGeneration)) {
      closeLink(link);
      changed = true;
    }
    if (link.up && !link.active) {
      openLink(link);
      changed = true;
    }
    if (!link.active) continue;

    readCommands(h, link);

    if (link.pending) {
      if (now - link.noticeMs >= kAuthNoticeMs) {
        link.noticeMs = now;
        sendWaiting(h, link);
      }
      // A client that is never confirmed does not get to hold the link open.
      if (now - link.pendingSinceMs > kAuthTimeoutMs) {
        link.pending = false;
        BLEConnection *conn = Bluefruit.Connection(h);
        if (conn) conn->disconnect();
      }
    }

    if (!link.paramsAsked && now - link.connectedMs > kParamDelayMs) {
      link.paramsAsked = true;
      requestSlowerInterval(h);
    }

    drain(h, link);
  }

  if (changed || now - lastAdvertisingCheckMs > 2000) {
    lastAdvertisingCheckMs = now;
    keepAdvertising();
  }
}

// ---------------------------------------------------------------------------
void bleWriteNmea(const char *line, size_t len) {
  for (uint16_t h = 0; h < kMaxLinks; ++h) {
    Link &link = s_links[h];
    // A sentence that does not fit is dropped whole, which a parser shrugs
    // off; half of one would take the following sentence with it.
    if (link.active && nmeaTx.notifyEnabled(h))
      link.nmea.push(reinterpret_cast<const uint8_t *>(line), len);
  }
}

// ---------------------------------------------------------------------------
bool bleAuthPending() {
  for (const Link &link : s_links)
    if (link.active && link.pending) return true;
  return false;
}

bool bleAuthConfirm() {
  uint8_t waiting = 0;
  for (const Link &link : s_links)
    if (link.active && link.pending) ++waiting;
  if (waiting == 0) return false;

  if (waiting > 1) {
    for (uint16_t h = 0; h < kMaxLinks; ++h) {
      Link &link = s_links[h];
      if (!link.active || !link.pending) continue;
      link.pending = false;
      BLEConnection *conn = Bluefruit.Connection(h);
      if (conn) conn->disconnect();
    }
    return false;
  }

  for (uint16_t h = 0; h < kMaxLinks; ++h) {
    Link &link = s_links[h];
    if (!link.active || !link.pending) continue;

    uint8_t key[CLIENT_KEY_LEN];
    if (!fillRandom(key, sizeof(key))) return false;

    cfgClientAdd(key);
    link.pending = false;
    link.open    = true;

    char hex[2 * CLIENT_KEY_LEN + 1];
    toHex(key, sizeof(key), hex);
    JsonDocument doc;
    doc["r"]   = "auth";
    doc["ok"]  = true;
    doc["key"] = hex;
    sendTo(h, link, doc);
    drain(h, link);

    // Written at once, but after the answer is on its way: writing the file
    // takes a good part of a second, and the client need not wait for it. Were
    // the write lost, the client would be asked for the button once more,
    // nothing worse.
    cfgSaveIfDirty();
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
void bleSendStatus() {
  // Checked here rather than at the call sites: this is the message that
  // carries the position, and a caller that forgets would leak it.
  if (!s_status || !anyListening()) return;
  JsonDocument doc;
  fillStatus(doc);
  broadcast(doc);
}

// Reports the outcome of a deferred configuration write. A client that never
// hears this can assume nothing was written.
void bleSendSaved(bool ok) {
  if (!anyListening()) return;
  JsonDocument doc;
  doc["r"]  = "saved";
  doc["ok"] = ok;
  broadcast(doc);
}

size_t bleEventRoom() {
  size_t room  = 0;
  bool   found = false;
  for (uint16_t h = 0; h < kMaxLinks; ++h) {
    if (!listening(h, s_links[h])) continue;
    const size_t here = s_links[h].json.room();
    room  = (found && room < here) ? room : here;
    found = true;
  }
  return room;
}

// ---------------------------------------------------------------------------
//  Received reports
// ---------------------------------------------------------------------------
//  These are events rather than replies: they arrive whenever a frame is
//  decoded. The NMEA stream carries the same information in its own form.
void bleSendTraffic(const proto::Traffic &t, unsigned tracked, uint32_t ageSec) {
  if (!anyListening()) return;

  JsonDocument doc;
  doc["r"]    = "traffic";
  if (ageSec) doc["age"] = ageSec;
  doc["src"]  = (t.source == proto::Source::Flarm) ? "FLARM"
              : (t.source == proto::Source::Fanet) ? "FANET" : "ADS-L";
  doc["id"]   = t.address;
  doc["type"] = uint8_t(t.aircraft);
  doc["n"]    = tracked;
  if (t.hasPosition) {
    doc["lat"] = serialized(String(t.latitude, 6));
    doc["lon"] = serialized(String(t.longitude, 6));
    doc["alt"] = (int)t.altitude;
  }
  if (t.hasVelocity) {
    doc["spd"] = (int)(t.groundSpeed * 3.6f);      // km/h for display
    doc["clb"] = serialized(String(t.climbRate, 1));
    doc["trk"] = (int)t.track;
  }
  if (t.rssi) doc["rssi"] = t.rssi;
  broadcast(doc);
}

void bleSendWeather(const proto::Weather &w, uint32_t ageSec) {
  if (!anyListening()) return;

  JsonDocument doc;
  doc["r"]  = "weather";
  if (ageSec) doc["age"] = ageSec;
  doc["id"] = w.address;
  if (w.hasPosition) {
    doc["lat"] = serialized(String(w.latitude, 6));
    doc["lon"] = serialized(String(w.longitude, 6));
  }
  if (w.hasTemperature) doc["temp"] = serialized(String(w.temperature, 1));
  if (w.hasWind) {
    doc["wdir"]  = (int)w.windDir;
    doc["wind"]  = serialized(String(w.windSpeed, 1));
    doc["gust"]  = serialized(String(w.windGust, 1));
  }
  if (w.hasHumidity) doc["hum"]   = (int)w.humidity;
  if (w.hasPressure) doc["press"] = serialized(String(w.pressure, 1));
  if (w.hasBattery)  doc["bat"]   = w.battery;
  if (w.internetGateway) doc["gw"] = true;
  if (w.rssi) doc["rssi"] = w.rssi;
  broadcast(doc);
}

void bleSendName(const proto::Name &n, uint32_t ageSec) {
  if (!anyListening()) return;

  JsonDocument doc;
  doc["r"]    = "name";
  if (ageSec) doc["age"] = ageSec;
  doc["id"]   = n.address;
  doc["name"] = n.text;
  broadcast(doc);
}

// ---------------------------------------------------------------------------
void bleBegin(const char *devName) {
  // Has to happen before begin(); afterwards the configuration is fixed.
  Bluefruit.configPrphConn(kMaxMtu, kEventLength, kNotifyQueue,
                           BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);

  if (!Bluefruit.begin(kMaxLinks, 0)) {
    Serial.println("ble: stack did not start");
    return;
  }
  // The library has a connection indicator of its own: it toggles LED_BLUE
  // from a timer every 250 ms while advertising and lights it while
  // connected. This board has no blue LED, and the variant maps that name to
  // the red one. The main loop rewrites both LEDs on every pass, which hides
  // the toggling, but not while the loop is held up: during the melodies, and
  // above all during the second or so it takes to send the GNSS receiver into
  // backup mode on the way to switching off, the red LED flashed by itself.
  Bluefruit.autoConnLed(false);

  Bluefruit.setTxPower(4);
  Bluefruit.setName(devName);
  Bluefruit.setEventCallback(onBleEvent);

  uint8_t address[kAddressLength];
  Bluefruit.getAddr(address);
  toHex(address, sizeof(address), s_deviceId);

  limitBonds();

  // Published as the preferred parameters as well, for centrals that read
  // them instead of waiting to be asked.
  Bluefruit.Periph.setConnInterval(kIntervalMin, kIntervalMax);

  deviceInfo.setManufacturer("Breezedude");
  deviceInfo.setModel("T1000-E Beacon");
  deviceInfo.begin();

  batteryService.begin();
  batteryService.write(100);

  // The serial service is put together by hand instead of using the library's
  // class: that one sends through a call that waits, and keeps a receive
  // buffer nothing here would read.
  nmeaService.begin();
  nmeaTx.setProperties(CHR_PROPS_NOTIFY);
  nmeaTx.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  nmeaTx.setMaxLen(kMaxPayload);
  nmeaTx.begin();
  // Present because applications look for it, and otherwise ignored.
  nmeaRx.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  nmeaRx.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  nmeaRx.setMaxLen(kMaxPayload);
  nmeaRx.begin();

  // Open at the Bluetooth level; who may use it is decided by handleLocked().
  configService.begin();
  configTx.setProperties(CHR_PROPS_NOTIFY);
  configTx.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  configTx.setMaxLen(kMaxPayload);
  configTx.begin();
  configRx.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  configRx.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  configRx.setMaxLen(kMaxPayload);
  configRx.setWriteCallback(onConfigWrite, false);
  configRx.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(nmeaService);
  Bluefruit.ScanResponse.addName();

  // Restarted from blePoll(), which also covers the case the library does
  // not: one link taken and one still free.
  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.setInterval(kAdvFast, kAdvSlow);
  Bluefruit.Advertising.setFastTimeout(kAdvFastSeconds);
  Bluefruit.Advertising.start(0);
}
