# Development notes

Technical documentation for the Breezedude Beacon firmware. For installation and
day to day use see [README.md](README.md).

---

## Feature status

| | Status |
|---|---|
| FANET tracking (type 1 in the air, type 7 on the ground) | received by a ground station |
| FANET name (type 2) | implemented |
| FLARM legacy V6 and V7 | frame format accepted by a ground station; not yet confirmed with the transmit slots |
| ADS-L (ED 2022/024/R) | golden vector verified on host; not yet confirmed on air |
| Receive path | implemented, one protocol at a time, off by default |
| NMEA over Bluetooth, configuration page | in use with a Chromium browser on Windows |
| Build for SoftDevice S140 7.3.0, the one the device is delivered with | runs on hardware, installed with `firmware-full.uf2`; a factory-fresh device with `firmware.uf2` alone not yet tried |
| Listen before talk, airtime account | host tested; **not yet run on a device** |
| Current while switched off | reasoned from the circuit, not measured |

The FLARM check was made on the frame format alone. The frames have not changed
since, but they are now sent in their time slots, and reception by a station
that hops with those slots is still to be confirmed. Until it is, treat FLARM
and ADS-L as experimental.

All three protocols share the single LR1110 transceiver, so the modem is
switched between LoRa and 2-FSK several times a second. The same constraint
applies to reception: only one protocol can be listened for at a time, and
listening keeps the receiver powered continuously, so it is off unless asked
for. FLARM alternates between V6 and V7 by default; either format can be
selected exclusively. FLARM and ADS-L are transmitted in their time slots, see
[Timing and channels](#timing-and-channels).

---

## Building

[PlatformIO](https://platformio.org/) fetches the toolchain, the Adafruit nRF52
core and the libraries on the first run.

```bash
pio run -e t1000e          # firmware, also produces the two .uf2 files
pio test -e native         # host tests, needs a C++17 compiler on the host
```

The results are in `.pio/build/t1000e/`: `firmware.uf2`, the application alone,
and `firmware-full.uf2`, the application together with the SoftDevice it runs
on, see [Bootloader and SoftDevice](#bootloader-and-softdevice). Install either as
described in the [README](README.md#installing-the-firmware): put the device
into its bootloader and copy the file onto the drive that appears.

Target: nRF52840 with SoftDevice S140 7.3.0, application start `0x27000`, UF2
family `0xADA52840`. The build uses C++17.

For a release, raise `FW_VERSION` in `src/config.h`, build, and attach both
`.uf2` files. The configuration page is published separately, see
[Configuration page](#configuration-page).

### Project layout

```
src/
  proto/            frame encoders, standard library only
    bits.h          bit packing, CRC, Manchester, endian-safe word access
    xxtea.h         XXTEA block cipher
    report.h        decoded reports, shared by all three protocols
    fanet.*         FANET tracking, ground tracking, name and weather frames
    flarm.*         FLARM legacy V6 and V7 frames
    ads_l.*         ADS-L (ED 2022/024/R) frames
    nmea.*          $PFLAU, $PFLAA and the weather sentence
  radio.*           LR1110 driver, modem switching, transmission
  slot_clock.*      UTC second estimate and transmit slot timing
  airtime.*         time on air, the airtime account, the pause per frequency
  gnss_filter.*     plausibility check and smoothing of the own position
  flight_state.*    walking or flying
  rx_cache.*        stations heard in the last few minutes
  battery.*         state of charge from the cell voltage
  main.cpp          scheduling, LED and buzzer, power management, auto-off
  config.*          configuration, JSON, persistence
  ble_link.*        BLE services, configuration protocol, who may configure
  byte_ring.h       queue behind the BLE link
  sha256.*          SHA-256 and HMAC for the client key check
  accel.*           QMA6100P, motion detection
  pins_t1000e.h     pin assignments
boards/t1000e.json  board definition: SoftDevice version, linker script, variant
linker/             memory layout under SoftDevice 7.3.0
variants/T1000-E/   board variant with an identity pin map
test/               host tests, one directory per module above
tools/index.html    Web Bluetooth configuration interface
uf2_postbuild.py    produces the .uf2 files after each build
licenses/           licence of the SoftDevice that firmware-full.uf2 contains
```

The encoders under `src/proto/` depend only on the standard library. They take a
position and return the bytes that go on the air, and know nothing about any
radio, which keeps the wire formats testable without hardware. The same holds
for the position filter, the flight state, the slot timing, the battery curve,
the station cache, the byte queue and the hash: whatever can be decided without
the board is kept free of it and has a test directory of its own.

Those tests cover the properties that are easy to get wrong and impossible to
observe from the transmitting side: bit ordering, Manchester polarity, the
checksum seed and the field scaling. Expectations are taken from decoded
recordings of real traffic, so a regression in the frame layout fails on the host
rather than silently going out over the air.

Bit fields are assembled explicitly rather than through packed structs, so the
layout does not depend on how a particular compiler orders them, and 32 bit words
are loaded and stored byte by byte so the cipher stages are independent of host
endianness.

### The board variant is required

The Adafruit nRF52 core translates every Arduino pin number through
`g_ADigitalPinMap[]` and silently discards anything at or above `PINS_COUNT` in
`digitalWrite()` and `analogRead()`. With a stock variant the raw port numbers
used here land on the wrong pins or nowhere at all, with no diagnostic. The
variant in `variants/T1000-E/` uses an identity map, which makes
`pins_t1000e.h` correct by construction.

---

## Hardware

Pin assignments were established on the device and agree with the board
manufacturer's own firmware, which is the reference for how the board is meant
to be driven:
<https://github.com/Seeed-Studio/Seeed-Tracker-T1000-E-for-LoRaWAN-dev-board>.
No schematic is published. The Arduino pin number is `32 × port + bit`.

| Function | Port | Pin | Note |
|---|---|---|---|
| LR1110 NSS / SCK / MISO / MOSI | P0.12 / P0.11 / P1.08 / P1.09 | 12 / 11 / 40 / 41 | |
| LR1110 RESET / BUSY / IRQ | P1.10 / P0.07 / P1.01 | 42 / 7 / 33 | TCXO on DIO3, antenna switch on DIO5 to DIO8 |
| GNSS receive | P0.14 | 14 | 115200 baud |
| GNSS transmit | P0.13 | 13 | |
| GNSS EN / VRTC / RST / SINT / RINT | P1.11 / P0.08 / P1.15 / P1.12 / P0.15 | 43 / 8 / 47 / 44 / 15 | **RST stays low**; a high pulse on RINT leaves backup mode |
| GNSS RESETB | P1.14 | 46 | driven by the receiver |
| Button | P0.06 | 6 | **active high**, needs a pull-down |
| LED green / red | P0.24 / P0.03 | 24 / 3 | |
| Buzzer / buzzer enable | P0.25 / P1.05 | 25 / 37 | |
| I²C SDA / SCL | P0.26 / P0.27 | 26 / 27 | QMA6100P at `0x12` |
| Accelerometer supply | P1.07 | 39 | |
| Sensor supply | P1.06 | 38 | thermistor, light sensor and battery divider; on only while sampling |
| Battery ADC | P0.02 | 2 | 1:2 divider, fed from the sensor supply |
| Flash supply | P1.13 | 45 | QSPI flash, unused, held off |

Identified parts: LR1110 `HW 0x22, dev 0x01, fw 0x0307`, QMA6100P
`WHO_AM_I 0x90`. A further I²C device answers at `0x7E` and is not identified.

### Traps worth knowing about

**`GNSS_RST` must be held low.** Driven high, the receiver sits in permanent
reset and emits nothing at all, regardless of baud rate or pin assignment.

**P1.06 is not a general 3.3 V rail.** It switches a small regulator for the
thermistor, the light sensor and the battery divider, and the manufacturer's
firmware turns it on for the length of a measurement only. The button does not
depend on it, tempting as that guess is on a board without a schematic. Left
on while the device is switched off, it drains the battery for nothing.

**The antenna switch needs configuring.** It hangs on four of the LR1110's own
pins. Without the switch table the radio still transmits and receives through
what leaks across the switch, which is enough to pass a test on the bench next
to a ground station and nowhere near enough in the air.

**Every modem change through the library resets the chip.** `begin()` and
`beginGFSK()` contain a fixed 300 ms wait. Called per frame, as the obvious
implementation does, three protocols keep the main loop blocked for most of
every second. See [Radio driver](#radio-driver).

**The core's UART receive buffer is 64 bytes**, under six milliseconds at
115200 baud, while the receiver's report is around a kilobyte once a second.
Anything that holds the main loop for longer cuts sentences out of it, and the
symptom is a fix that keeps dropping out under an open sky. The buffer is
enlarged with `SERIAL_BUFFER_SIZE` in `platformio.ini`.

**The Bluetooth library drives the red LED.** Bluefruit expects a blue LED as
its connection indicator, blinks it from a timer while advertising and lights
it while connected. The variant has to define `LED_BLUE` for the library to
build and maps it to the red LED, so the indicator is switched off with
`Bluefruit.autoConnLed(false)`. Left on, the main loop hides it by rewriting
the LEDs on every pass, except while the loop is held up: then the red LED
flashes by itself, most visibly during the GNSS shutdown on the way off.

**The core sets every output latch high on start-up**, and pin configuration
survives the reset that wakes the device from system-off. A supply enable left
as a low output while off therefore pulses high on every wake, before
`setup()` runs. Lines that have to hold a level while off are handed to the
internal pull resistors instead, which the latch does not affect.

---

## Bluetooth LE interface

Two services, so that a navigation app and the configuration page do not have
to share a channel, and up to two centrals at the same time:

| Service | UUID | Who may use it | Carries |
|---|---|---|---|
| Nordic UART | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | anyone who connects | NMEA sentences |
| Configuration | `88C00001-13A4-4AAB-998F-26EE246F7E7B` | clients holding a key handed out at the button | one JSON object per line, both ways |

Both have the same shape: a characteristic ending in `0002` that the client
writes to, and one ending in `0003` that notifies. The serial service accepts
writes because applications look for that characteristic, and ignores them.
Device Information and Battery are there as well.

A central is sent a stream only while it is subscribed to it: a navigation app
subscribes to the serial service and gets NMEA, the configuration page
subscribes to its own and gets JSON, and one that wants both subscribes to
both. There is no mode to switch.

Anyone within Bluetooth range can read the position from the NMEA stream. That
is deliberate: the beacon broadcasts the same position over the air on purpose.

### Sending never waits

Outgoing data goes into a queue per connection and stream (`byte_ring.h`) and
is handed to the stack from `blePoll()` only when the stack has room. A message
enters a queue whole or not at all, so a full queue costs a sentence or a
status report, never half of one. Host tests are in `test/test_ring`.

The reason is the radio. The library's own send call waits for room, up to a
tenth of a second per packet, and that wait would sit in the loop that has to
hit the FLARM and ADS-L time slots. Room is known without asking: the code
counts the notifications it handed over and the ones the stack reported as
sent (`BLE_GATTS_EVT_HVN_TX_COMPLETE`), and never has more outstanding than the
stack was configured to hold.

Packets are as large as the negotiated MTU allows, up to 244 bytes, instead of
a fixed twenty. Replies go before NMEA when both are waiting.

Incoming bytes are collected in the Bluetooth task and parsed in the main loop,
so a command never runs in the middle of a transmission.

### Protocol, version 1

Before a client is let in, only two commands count, see *Who may configure*:

```json
{"cmd":"auth","mac":"<64 hex>"}        HMAC-SHA-256 of the nonce under the client's key
{"cmd":"pair"}                         asks for a key; needs the button
```

Afterwards. Any of these may carry an `id`, which is echoed in the reply:

```json
{"cmd":"hello","v":1}                  opens the session
{"cmd":"get"}                          whole configuration
{"cmd":"set","cfg":{"pilot":"Flo"}}    partial update
{"cmd":"status"}                       live status, once
{"cmd":"beep","vol":60}                preview tone, changes nothing
{"cmd":"reboot"}
{"cmd":"bootloader"}                   enter the UF2 bootloader
```

Replies:

```json
{"r":"hello","v":1,"fw":"0.3.0","mtu":244,"mot":true,"radio":true,"caps":["fanet","flarm","adsl","rx"]}
{"r":"cfg","cfg":{...}}
{"r":"ok","cmd":"set"}
{"r":"err","cmd":"set","msg":"missing cfg"}
```

Events, sent without being asked for:

```json
{"r":"auth","ok":false,"nonce":"<32 hex>","dev":"<12 hex>"}   prove yourself
{"r":"auth","ok":false,"wait":true}    waiting for the button
{"r":"auth","ok":true}                 let in; with "key":"<64 hex>" when the key is new
{"r":"status",...}                     once a second
{"r":"traffic"|"weather"|"name",...}   see Receiving
{"r":"saved","ok":true}                outcome of a deferred configuration write
```

`hello` carries what does not change during a session, so the status message
does not repeat it every second:

| Field | Meaning |
|---|---|
| `v` | protocol version of the firmware |
| `fw` | firmware version string |
| `mtu` | largest write the client may send in one piece, in bytes |
| `mot` | accelerometer present |
| `radio` | radio initialised |
| `caps` | what this build can do, for clients that want to hide the rest |

The rules that let either side grow without breaking the other: unknown
commands get an error, unknown fields are ignored, absent fields keep their
value, and a client ignores events it does not know. A change that cannot be
expressed that way raises `v`.

On the client side, register the notification listener *before* enabling
notifications, decode with a streaming decoder, because a character can be
split across two packets, and run one GATT operation at a time: some platforms
refuse a second one while the first is in the air.

### Configuration fields

| Field | Meaning |
|---|---|
| `pilot` | name transmitted in FANET type 2 |
| `vendor`, `devId` | FANET identity |
| `fVendor`, `fDevId` | FLARM identity, independent of FANET |
| `fProto` | FLARM format: `0` alternate, `6` V6 only, `7` V7 only |
| `acType` | aircraft type 0–7 |
| `online`, `fanet`, `flarm`, `adsl` | feature switches |
| `rxOn` | listen for other aircraft; off leaves the receiver idle |
| `rxProto` | protocol to listen for: `0` FLARM, `1` FANET, `2` ADS-L |
| `adslAddr` | ADS-L device address (24 bit), `0` derives it from the FLARM identity |
| `adslTable` | ADS-L address mapping table, six bits: `5` ICAO, `6` FLARM, `7` OGN (default), `8` FANET |
| `gnssVrtc` | keep the GNSS backup supply on while the device is off, so the receiver keeps its clock and satellite data and the next fix is quick |
| `trackMs`, `nameMs` | transmission intervals |
| `txPower` | −9 … +22 dBm; the band allows 25 mW e.r.p., see [Sharing the band](#sharing-the-band) |
| `lbt` | listen before talk; off keeps to the 1 % duty cycle |
| `buzzer` | 0–100 % |
| `lowVoltMv` | cutoff voltage, `0` disables |
| `autoOffMin` | motion timeout in minutes, `0` disables |

The identity is one 24 bit number shared by all three protocols, so a receiver
hearing the device twice sees one aircraft rather than two. The upper byte is
the FANET manufacturer id `0xBD`, held by this project; the lower two bytes are
the device number and are derived from the factory serial number in `FICR`
(`cfgResetIdentity()` in `config.cpp`).

Deriving it rather than shipping a constant means every unit is distinct out of
the box, which matters because most users never open the settings: a fixed
default would put every device on the same address. `FICR` is read only, so the
number survives reflashing and a configuration reset.

Because the number is self assigned, the transmitted address type says so:
FLARM frames use tag `3` (self assigned, OGN) rather than `2` (issued by
FLARM), and ADS-L uses mapping table `7` for the same reason. Register the
address at [ddb.glidernet.org](https://ddb.glidernet.org/) as device type OGN
to be identified by name on tracking sites.

Changing the identity is a migration concern: a stored configuration wins over
the defaults, so `cfgLoad()` compares `CFG_VERSION` against the file and
re-derives the identity when the file is older, keeping every other setting.

### Status fields

| Field | Meaning |
|---|---|
| `fix`, `sat` | fix state and satellites in view (GSV, all constellations) |
| `lat`, `lon`, `alt` | position, only present with a fix |
| `spd`, `clb`, `hdg` | speed in km/h, climb in m/s, track in degrees |
| `bat`, `batP` | battery volts and percent |
| `chg` | USB VBUS present, i.e. charging |
| `tx`, `txF`, `txA` | FANET, FLARM and ADS-L frame counts |
| `skip` | frames held back by the airtime account |
| `busy` | times the channel was found in use before a frame |
| `fV7` | whether the last FLARM frame used the newer format |
| `fSt` | last radio status code, `0` is success |
| `noMot` | seconds since movement |

### Who may configure

Both configuration characteristics are open at the Bluetooth level. Access is
decided above that, by a key.

A client is let in once it has shown that it holds a key this device handed
out, and a key is handed out only after a short press on the button. The
confirmation is physical on purpose: an attacker in radio range has the radio,
but the button is on the device.

```
client                                   device
  hello                          ->
                                 <-      auth ok:false, nonce, dev
  auth mac=HMAC(key, nonce)      ->                         (has a key for dev)
                                 <-      auth ok:true
  hello                          ->
                                 <-      hello ...

  pair                           ->                (no key, or key refused)
                                 <-      auth ok:false, wait      (repeated)
                                          ... button pressed ...
                                 <-      auth ok:true, key
  hello                          ->
```

- The key is 32 random bytes from the SoftDevice's hardware generator. The
  device keeps the last four in `/config.json` beside the settings; they are
  not part of what `get` returns or `set` accepts. The client files its key
  under `dev`, the device's own Bluetooth address.
- The key goes over the air once, when it is handed out. On every later
  connection the device sends a fresh 16 byte nonce and the client answers
  with its HMAC-SHA-256 under the key, so a recorded session holds nothing
  that could be replayed. A nonce is good for one answer.
- While a client waits for the button, the LED alternates green and red and a
  two-tone figure plays that is deliberately unlike the fix melodies. Thirty
  seconds without a press drops the connection.
- The press is acted on as it goes down, not on release, with a click at once.
  The key is sent before it is written to flash, which takes a good part of a
  second; were that write lost, the client would only be asked for the button
  again.
- Two clients waiting at once are both turned away by the press: it cannot say
  which of the two it means, and the other may be a stranger who was waiting
  for exactly this moment.
- A client that has not been let in is told nothing but the challenge. No
  status, no settings, no events.

`sha256.*` is plain C++ and is checked against the FIPS 180-4 and RFC 4231
vectors in `test/test_sha256`; the page uses the browser's WebCrypto for the
same computation.

**Why not Bluetooth's own pairing.** The obvious design is to make the
characteristics require an encrypted link and let bonding identify the client.
That works on Android and macOS, where the system pairs on its own when a
protected characteristic is touched. Chrome and Edge on Windows and Linux do
not: pairing without a passkey is behind a browser flag there
(`enable-web-bluetooth-confirm-pairing-support`), and without it the first
subscription simply fails. A configuration page that needs a flag set is not
one to publish.

**What this does not cover.** The link is not encrypted. Someone recording the
connection at the moment a key is handed out has the key; pairing without a
passkey has the same gap in its older form. Someone recording a later session
sees the settings and the status, which is the position the beacon broadcasts
anyway, and gains nothing that lets them in.

The library still accepts a pairing request from anyone and stores a file per
bond. Nothing here relies on those bonds, so `limitBonds()` clears them at
start-up once there are more than eight, before they crowd the file system the
configuration lives in.

### Connection parameters and advertising

Five seconds after a central connects, once discovery is through, the device
asks for an interval of 60 to 90 ms; the same range is published as
the preferred parameters. The central is free to refuse. Slow enough to let the
radio rest between events, fast enough that a client limited to twenty byte
packets still gets the whole NMEA stream.

Advertising runs at 152.5 ms for thirty seconds after start-up or a
disconnect, then at about one second. It continues while one of the two links
is free; the library on its own restarts it only once every link is gone.

Both are small savings next to the GNSS receiver, which is what the battery
really goes on. They are taken because they cost nothing.

Two links cost SoftDevice memory, which comes out of a reserve at the bottom
of RAM. The linker script sets it to 32 KB, eight more than the core's
default, because how much the stack needs for a given configuration can only
be found out on the running device. If the connection settings (`kMaxMtu`,
`kEventLength`, `kNotifyQueue` in `ble_link.cpp`) are raised beyond what fits,
the stack refuses to start and the serial log says `ble: stack did not start`.

### Persistence

The configuration is written to `/config.json` in the internal filesystem
fifteen seconds after the last change, and again on disconnect and on shutdown.
A device without that file has not run this firmware before, and its file
system is formatted on that first start, see
[Bootloader and SoftDevice](#the-first-start-clears-the-file-system).
Waiting for the end of the session alone is not enough: a flat battery or a
client that never disconnects cleanly would cost the changes. Fifteen seconds
is long enough that dragging a slider is one write rather than fifty, and the
device runs for days on a charge, so there is no hurry.

Writing is atomic and verified. `FILE_O_WRITE` in Adafruit_LittleFS seeks to the
end of the file rather than truncating, so the old file has to be removed
first — appending would leave two JSON objects in the file, and a parser reading
the first one would faithfully restore the older settings on every start. The
new configuration is therefore written to `/config.new`, read back and parsed,
and only then moved into place. The outcome is reported to the client as
`{"r":"saved","ok":true|false}`, so a failure shows up in the configuration page
rather than at the next start.

### NMEA output

Limited to `GGA` and `RMC`. The full GNSS stream is more than a client limited
to twenty byte packets can take. `GSV` is consumed on the device to count
satellites in view, but not forwarded.

---

## Bootloader and SoftDevice

The firmware is built for the device as it is delivered, so that installing it
is copying one file and going back to other firmware is copying another.

| From | What | Installed by |
|---|---|---|
| `0x00000` | MBR | the factory; never written here |
| `0x01000` | SoftDevice S140 **7.3.0**, Nordic's Bluetooth stack | the factory, or `firmware-full.uf2` |
| `0x27000` | the application | `firmware.uf2` or `firmware-full.uf2` |
| `0xED000` | internal file system, 28 KB | |
| `0xF4000` | UF2 bootloader | the factory; see below |

`INFO_UF2.TXT` on the bootloader drive says what a device carries. As
delivered:

```
UF2 Bootloader 0.9.1-5-g488711a ...
Model: Seeed T1000-E for Meshtastic
Board-ID: nRF52840-T1000-E-v1
SoftDevice: S140 7.3.0
```

### Building for 7.3.0

An application is linked against one SoftDevice version and starts where that
version ends: `0x26000` under 6.1.1, `0x27000` under 7.3.0. On the wrong one
it installs without complaint and never starts.

The Arduino core builds for this chip against 6.1.1 and has no linker script
for anything else, although it carries the 7.3.0 headers. Three things in this
repository change that:

| | |
|---|---|
| `boards/t1000e.json` | selects `sd_version` 7.3.0 and its firmware id `0x0123`, which is also what the DFU package the build produces asks for |
| `linker/nrf52840_s140_v7.ld` | the core's script with the application moved up one page and a larger RAM reserve for the SoftDevice |
| `uf2_postbuild.py` | checks that the SoftDevice it packs is the one the application was built for |

### When the SoftDevice is a different one

Other firmware for this board runs on 6.1.1 and replaces the SoftDevice when
it is installed. For a device in that state there is `firmware-full.uf2`,
which carries S140 7.3.0 in front of the application.

That is something the bootloader provides for: it counts the SoftDevice as
part of the application space, writes everything from the end of the MBR up to
its own start, skips blocks addressed to the MBR, and finds the application by
asking the SoftDevice in flash for its size rather than assuming one. One file
thus replaces the SoftDevice and installs the application, and neither the MBR
nor the bootloader is touched. If the copy is interrupted, the bootloader is
still there and the file can be copied again.

`uf2_postbuild.py` takes the SoftDevice out of a bootloader image that ships
with the Arduino core and puts the application behind it. The image is one for
the nRF52833, the board the core does build against 7.3.0; the SoftDevice is
the same file for both chips, and nothing else is taken from it. The result
has been compared byte for byte with the SoftDevice file Nordic publishes.

### The first start clears the file system

The internal file system is 28 KB and shared by whatever firmware is on the
device. What a predecessor left in it can be enough to make every attempt to
store the settings fail. `cfgLoad()` therefore formats it when it finds no
configuration of its own, which is exactly the case in which nothing of this
firmware's is in it yet. The other firmware's settings are gone after that; it
starts with its defaults when it is installed again.

### The bootloader

Leave it alone unless there is a reason. It is the one thing that cannot be
repaired by copying a file: the device is sealed and has no debug connector on
the outside, so a bootloader that does not start stays that way.

The bootloader is the
[Adafruit nRF52 bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader),
which knows this board as `t1000_e`. What the device is delivered with is that
project at the commit which added the board, a few days before version 0.9.2.
Its [releases](https://github.com/adafruit/Adafruit_nRF52_Bootloader/releases)
hold two files for the board, and only files with that board name may be used:

| File | Contains | How |
|---|---|---|
| `update-t1000_e_bootloader-<version>_nosd.uf2` | the bootloader alone | copy onto the bootloader drive, like a firmware |
| `t1000_e_bootloader-<version>_s140_7.3.0.zip` | bootloader and SoftDevice 7.3.0 | `adafruit-nrfutil`, below |

```bash
pip3 install --user adafruit-nrfutil
adafruit-nrfutil --verbose dfu serial --package t1000_e_bootloader-<version>_s140_7.3.0.zip -p <port> -b 115200 --singlebank --touch 1200
```

`<port>` is the serial port the device shows up as. `--touch 1200` asks a
running firmware to enter the bootloader; leave it out when the device is in
the bootloader already. Seeed describes the same procedure, with the package
the device was delivered with, in the
[T1000-E guide](https://wiki.seeedstudio.com/sensecap_t1000_e/). It is also
the way to bring a device back to the delivered state from anything else.

Versions after 0.9.2 reworked large parts of the bootloader. What this
firmware relies on, see below and under [Power](#power), has been checked
against 0.9.2 only.

Devices sold with Seeed's LoRaWAN firmware carry a different bootloader. Whether
that one behaves the same has not been tried; installing the package above
first puts the device into the state described here.

### Licences

The project is under CC BY-NC 4.0, see [LICENSE.txt](LICENSE.txt).

The SoftDevice is Nordic Semiconductor's and is distributed in binary form
under [their licence](licenses/nordic-softdevice.txt): redistribution is
allowed with the notice attached, for use on Nordic chips only, unmodified.
That is why `firmware-full.uf2` can be published, and why the licence text is
in this repository. The SoftDevice itself is not: the build takes it from the
Arduino core. The bootloader is MIT licensed and is not redistributed here.

The Arduino core and TinyGPSPlus are under the LGPL 2.1, which asks that
whoever receives the firmware can rebuild it with a changed library. With the
complete source and build in this repository, that holds.

### Entering the bootloader from the configuration page

`{"cmd":"bootloader"}` resets into the UF2 bootloader, where the device appears
as a mass storage volume. Bluetooth is not available there: UF2 mode brings up
USB, not the radio.

**Triggering it away from a computer is harmless.** `enterUf2Dfu()` sets
`GPREGRET` to `DFU_MAGIC_UF2_RESET`, and the Adafruit bootloader treats that
path as `bootloader_dfu_start(false, 3000, true)`: if the host has not
enumerated USB within three seconds, it gives up and starts the application
again. Checked against the source of version 0.9.2, which is the delivered
bootloader plus one change for another board. The bootloader also returns to the application when a cable that was
enumerated is unplugged again.

The application then comes up running rather than switched off, because the
wake gesture is only required when the reset reason is `RESETREAS.OFF`. A reset
out of the bootloader is a software reset, so `confirmWake()` is skipped.

### No update over Bluetooth

Updating without a cable would be convenient and is not offered. Each of the
three ways to do it runs into something:

| Route | What stops it |
|---|---|
| From the configuration page | The bootloader's update service (`00001530-1212-EFDE-1523-785FEABCD123`) is on the Web Bluetooth blocklist, so no page can open it |
| Through the configuration channel, with the application receiving the image | The bootloader keeps a single copy of the application. It cannot take over an image the application has stored, and the application cannot overwrite itself |
| From a phone, with a DFU app and the `firmware.zip` the build also produces | Works: `enterOTADfu()` starts it. But that mode has no timeout. Entered by mistake away from a computer, the device stays in the bootloader until an update completes or it is reset |

The third is the one that could be added, behind a confirmation that makes the
consequence clear.

---

## GNSS

AG3335 on UART at 115200 baud, running continuously at 1 Hz.

The part is an **Airoha** design and takes the `$PAIR` command set. It accepts
some of the older MediaTek `$PMTK` sentences and silently ignores the rest,
which is a good way to end up with a receiver in a mode nobody asked for:
duty-cycling it with `$PMTK225`, for one, leaves it not updating after the
first fix. There is no duty cycling at all.
Reception is what the device exists for, and the battery is looked after by
switching off after a period without motion instead.

### Starting and stopping

The sequence is the one in the manufacturer's driver (`ag3335.c`):

| | Step | Why |
|---|---|---|
| Start | main supply on, wait 50 ms | |
| | 3 ms high pulse on `RINT` | brings the receiver out of backup mode |
| | `$PAIR382,1`, 25 times at 40 ms | stops it from dozing between fixes; repeated because the first ones are lost while it is still asleep |
| | the settings below | only now is it certain to be listening |
| Stop | `$PAIR382,0`, 4 times | allow sleep again |
| | `$PAIR650,0`, 25 times | enter backup mode until woken by pin |
| | wait 50 ms, main supply off | |

Backup mode is what makes the next start quick. With the backup supply on, the
receiver keeps its clock and what it knows about the satellites, and comes back
with a hot or warm start instead of searching the sky from nothing. Every step
matters. Cutting the main supply without asking for backup mode, or starting
without the pulse on `RINT`, keeps the backup supply on to no reliable effect,
and the fix takes as long as a cold start. Settings sent once, a second after
power-up, go to a receiver that may not be awake to hear them.

The start is stepped from the main loop (`gnssTick()`), because it is two
seconds of mostly waiting. The stop is blocking; it happens on the way to
switching off.

There is no supply for an antenna amplifier to switch, and no other enable
line: `EN`, `VRTC`, `RST`, `SINT`, `RINT` and `RESETB` are all there is.

### Settings

Applied on every start and deliberately **not** written to the receiver's own
memory (`$PAIR513`), so the module is in a known state whatever a previous
firmware left behind:

| Command | Purpose |
|---|---|
| `$PAIR066,1,1,1,1,0,0` | GPS, GLONASS, Galileo and BeiDou. Four constellations rather than GPS alone is the single biggest lever on time to first fix and on holding a fix under a wing |
| `$PAIR062,<id>,<on>` | `GGA` and `RMC` for position, `GSV` for the satellite display; everything else off |

What remains outside the firmware's reach is the antenna: a small one, in a
case that also holds a transmitter. A cold start, after the backup supply was
off or the device lay unused for hours, takes as long as the sky allows. An
A-GNSS aid would shorten that and is not implemented.

### Position filter

Nothing transmits straight from the parser. `gnss_filter.*` holds a small
filter that every fix passes through, and the transmit paths, the NMEA output
and the status all read its result. It is plain arithmetic with no Arduino
dependency, so it is covered by host tests in `test/test_gnss`.

| Rule | Reason |
|---|---|
| At least four satellites | Fewer is not a three dimensional solution |
| Step under 100 m/s | Catches receiver artefacts, not flying |
| Altitude step under 30 m/s | Altitude is the noisiest quantity a GNSS produces |
| Re-seed after four rejections | A fix regained far from the last one is real; insisting on a stale reference would reject the truth forever |
| Exponential smoothing of altitude, climb, speed and heading | Time constant based, so the result does not depend on how often the receiver reports |
| Heading held below 1 m/s | A stationary receiver reports meaningless headings |

Climb rate comes from the smoothed altitude; the board has no barometer, so
this is as good as it gets and is not comparable to a variometer.

### Walking or flying

`flight_state.*` decides which, because the protocols ask outright: FANET has
two different frame types and ADS-L a flight state flag. Host tests are in
`test/test_flight`.

Ground speed on its own does not answer it. A paraglider into wind stands still
over the ground for minutes, and walking to a launch in the mountains gains
altitude the whole way, so neither speed nor a changing altitude decides
anything alone.

| | Evidence | Held for |
|---|---|---|
| Takeoff | speed ≥ 15 km/h **or** climb ≥ 1.0 m/s | 5 s |
| Landing | speed ≤ 12 km/h **and** climb ≤ 0.5 m/s **and** altitude within 15 m over the whole period | 45 s |

The asymmetry is deliberate. Being shown as a walker while airborne is the
worse error, so a takeoff is accepted quickly; a moment of hanging still in the
air is not a landing, so the other direction takes a long, uninterrupted stretch
of evidence. The altitude range is what separates a glider parked in ridge lift
from someone standing in a field: both have almost no speed and gentle vertical
movement, but only one of them keeps drifting. A brisk walk uphill is about
0.3 m/s, comfortably below the 1.0 m/s that counts as flying.

The detector reads the **filtered** speed and altitude; raw values jitter enough
on the ground to change its mind. A lost fix resets the evidence but not the
state: what happened while nothing was received cannot count towards either.

**A known limitation:** driving to a launch reads as flying, because it is fast
and the rule says fast is flying. FANET has a `Vehicle` ground status that would
cover it, but nothing on this board can tell a car from an aircraft.

### FANET frame selection

| State | Frame | Contents |
|---|---|---|
| Airborne | type 1, tracking | position, altitude, speed, climb, heading, aircraft type |
| On the ground | type 7, ground tracking | position and a status, here `Walking` |

Ground tracking carries no altitude, velocity or aircraft category, which is
the point: on the ground there is nothing to report about any of them, and
receivers show a walker rather than an aircraft at zero speed. Sending type 1
regardless would show a pilot on launch as an aircraft standing still in the
air.

**Transmission requires a live fix.** `haveFix()` demands that the filter holds
a position, that the satellite count is sufficient, and that the fix is younger
than four seconds. TinyGPS++ marks a location valid from the first fix onwards
and never takes it back, so the age check is what separates a live fix from the
memory of one — without it the device keeps transmitting a position it no
longer has.

Transmission stops the moment the fix goes stale, but the **tones wait ten
seconds**. A dropout of a second or two is ordinary and announcing it would
mean a beacon that chirps continuously under a wing. A loss that outlasts the
grace period plays a falling two note melody, the inverse of the one that
announces a fix, and resets the filter; regaining the fix is only announced if
the loss was.

---

## Power

### While switched off

The processor is in system-off, a few microamps, and wakes on the button
through GPIO sense, or when a cable is plugged in. That says nothing about the rest of the board. Pin
configuration is retained in system-off, and every line that still reaches a
part without supply feeds it through that part's protection diodes. The states
below are taken from the manufacturer's power-off routine; held with the
internal pull resistors rather than driven, for the reason given under
[Traps](#traps-worth-knowing-about).

| Line | State while off | What goes wrong otherwise |
|---|---|---|
| GNSS `EN`, `RST`, `RINT`, `SINT` | pulled low | `SINT` driven high feeds an unpowered receiver |
| GNSS `VRTC` | pulled high, or low with `gnssVrtc` off | |
| GNSS `RESETB` | disconnected | 13 kΩ pull-up into the receiver's output |
| GNSS UART | stopped, pins disconnected | |
| I²C SDA, SCL | disconnected | `Wire.begin()` enables both pull-ups, which keep the accelerometer half alive through its inputs |
| Accelerometer supply | pulled low | |
| Sensor supply (P1.06) | pulled low | left on, the thermistor, light sensor and divider draw several hundred microamps |
| Flash supply | pulled low | left floating, the flash may be half powered |
| LR1110 | cold sleep, NSS pulled high, SCK and MOSI pulled low | |
| Buzzer, LEDs | pulled low | |

Before the receiver's supply goes, it is sent into backup mode, see
[Starting and stopping](#starting-and-stopping). The radio is taken to standby,
its oscillator supply released, which resets it, and then put to sleep without
retention; the reset is what guarantees it is in a state sleep can be entered
from.

None of this has been measured. The board has no accessible test point in its
sealed case, so the figures come from what each path can carry, not from an
ammeter. What can be checked without one is the battery after a few days off.

### Switching off must not fail

`sd_power_system_off()` is documented to return only when the chip did not
power off, and writing `SYSTEMOFF` directly is ignored while the SoftDevice
owns the POWER peripheral. A device that falls through both sits in system-on
with the processor parked, where neither the button nor plugging in USB can
reach it: both only reset the chip out of system-off. `powerOffNow()` therefore
ends in system-off or in a reset, and after the reset the attempt is repeated
before the SoftDevice is started, where it cannot be refused.

The retained register is set to the value that tells the bootloader to start
the application directly.

### Watchdog

Sixty seconds, fed from the main loop. It is the only thing that gets the
device out of a hang, for the same reason as above: the button is read by the
loop that would be stuck. The library's start-up, for one, waits for the
radio's busy line with no timeout.

It stops by itself in system-off, and the bootloader feeds it while a firmware
file is being copied, so it interferes with neither. It cannot be stopped or
reconfigured once started and survives a soft reset.

### While running

- The main loop yields for a tick on every pass. Spinning flat out costs
  several milliamps and gains nothing: everything it polls is buffered or
  slower than a millisecond.
- The radio is asked for received frames when its interrupt line says there is
  one, not over the bus on every pass.
- The LR1110 runs from its switching regulator; the library selects the linear
  one, which roughly doubles the receive current.
- The battery is sampled every ten seconds with the sensor supply on for 30 ms,
  and everything else reads the stored value. The percentage follows the
  discharge curve of the cell rather than a straight line.
- Lying still, the device transmits less. Fifteen seconds without a movement
  report from the accelerometer drop the tracking interval to ten seconds
  (`kIdleTrackingMs`); any movement restores the configured one. A device whose
  accelerometer does not answer counts as moving.

### Switching on and off

| | Behaviour | Why |
|---|---|---|
| Wake | The press that woke the processor, a release, then a second press that is held (`confirmWake()`); anything else goes back to system-off | A button squeezed in a rucksack must not switch the device on |
| Start with an empty battery | Refused, with a low double tone | Starting the radio and the receiver would pull the cell further down |
| Hold for three seconds | Melody, then off on release | |
| No movement for `autoOffMin` | Off | Only while the accelerometer answers: a silent sensor looks like no movement and must never switch the device off by itself |
| Below `lowVoltMv` | Off after three samples in a row, ten seconds apart | The supply dips during a transmission, and one reading must not end the flight |
| Cable in or out | No change; off stays off, on stays on | Charging is not a reason to fly, and pulling a cable out is not a reason to start |

Whether the device should be running is kept in a one-byte file, `/power`
(`cfgPowerIntent()`), because nothing else survives everything that restarts
the chip: plugging in a cable wakes it from system-off, and pulling one out can
reset it outright as the supply changes over. `setup()` decides from the reset
reason and that file:

| Restarted by | Was | Then |
|---|---|---|
| The button, out of system-off | off | the wake gesture; failing it goes back to off |
| A cable, out of system-off | off | the charging state, with two rising tones |
| Anything else: power-up, watchdog, the firmware's own reset | on | runs straight away |
| | off | the charging state if a cable is in, otherwise system-off |

The charging state (`chargingIdle()`) starts nothing: no GNSS, radio or
Bluetooth, and the LEDs stay dark. The processor stays awake on USB power and
watches for the cable to come out, which ends in system-off with two falling
tones, and for the wake gesture, which switches the device on.

Switching off with a cable in leads there too. System-off would not notice the
cable coming out, so `enterSystemOff()` shuts the board down as usual and then
restarts into the charging state instead.

---

## Receiving

Reception decodes into one common shape (`proto/report.h`) so that everything
downstream works on `Traffic`, `Weather` and `Name` alone and never branches per
protocol.

What each protocol yields:

| Protocol | Decoded |
|---|---|
| FLARM | aircraft position, velocity and category, both frame formats |
| FANET | aircraft (type 1), ground objects (type 7), pilot names (type 2) and weather stations (type 4) |
| ADS-L | aircraft position, velocity and category |

FLARM is the awkward one. Positions are transmitted with their most significant
bits removed and are only meaningful against a nearby reference, and the
timestamp keys the cipher, so nothing can be decoded before the receiver has its
own fix and a correct clock. A clock more than a second out simply fails to
decrypt, which looks identical to no traffic at all.

FANET and ADS-L carry absolute positions and decode on their own.

The ADS-L address mapping table is wider than the data port's `<IDType>`, which
only distinguishes unassigned, ICAO and FLARM addresses. ICAO and FLARM map
across; OGN, FANET and the manufacturer pages have no counterpart and are
reported as unassigned rather than misattributed.

### NMEA output

Navigation applications understand the FLARM data port sentences, so every
protocol is reported through those regardless of how the frame arrived. The
field layout follows the FLARM Data Port Interface Control Document (FTD-012).

```
$PFLAU,<RX>,<TX>,<GPS>,<Power>,<AlarmLevel>,<RelativeBearing>,<AlarmType>,
       <RelativeVertical>,<RelativeDistance>[,<ID>]

$PFLAA,<AlarmLevel>,<RelativeNorth>,<RelativeEast>,<RelativeVertical>,<IDType>,
       <ID>,<Track>,<TurnRate>,<GroundSpeed>,<ClimbRate>,<AcftType>
```

`$PFLAU` goes out once a second while receiving and is what a consumer uses to
detect that a compatible stream exists at all. `$PFLAA` follows for each
aircraft. Positions are expressed relative to our own, so both a fix and a
decoded target position are required; without either, no sentence is produced.

No collision computation happens on the device, so `<AlarmLevel>` is always `0`
and the alarm fields stay empty. Consumers read that as traffic information
rather than a warning, which is the intent: this is a beacon, and any warning
logic belongs to the application.

Aircraft categories use the data port enumeration (`1` glider, `7` paraglider,
`8` powered and so on). FANET and ADS-L categories are mapped onto it; several
have no counterpart and become `A`, unknown.

Weather stations have no standard sentence, so they use a proprietary one.
Empty fields mean the station did not report that value:

```
$PBDWX,<id>,<lat>,<lon>,<temp C>,<dir deg>,<wind km/h>,<gust km/h>,
       <humidity %>,<pressure hPa>,<battery %>
```

### Status events

Each decoded frame is also pushed to configuration clients as an event, which
is what the configuration page lists:

```json
{"r":"traffic","src":"FANET","id":12388875,"type":7,"n":3,
 "lat":47.891290,"lon":12.194160,"alt":1200,"spd":36,"clb":-2.5,"trk":90,"rssi":-78}
{"r":"weather","id":12388875,"temp":21.0,"wdir":90,"wind":10.0,"gust":16.0,
 "hum":50,"press":1013.2,"bat":100}
{"r":"name","id":12388875,"name":"Test Pilot"}
```

`age`, in seconds, appears on reports replayed from the cache when a session
opens and is absent on live ones.

Clients ignore any `r` they do not recognise, so these can be extended without
breaking older ones.

### The station cache

Sent straight to whoever is connected and then forgotten, decoded reports
would leave a client that connects a minute later with an empty list until the
next frame happens to arrive. `rx_cache.*` therefore keeps the last five
minutes: one
entry per address, holding its position, its name and, for a weather station,
its measurements, because those arrive in separate frames that belong together.
Twenty-four entries, oldest evicted when full, cleared when reception is
switched off. Host tests are in `test/test_rxcache`.

It is handed over when a client opens its session with `hello`, a few entries
per pass as the queue empties (`continueReplay()` in `main.cpp`), because the
whole cache is larger than the queue. Each replayed report carries an `age` in
seconds, so nothing minutes old is shown as if it had just come in; live
reports omit the field.

The cache lives in RAM only. Switching the device off loses it, which is
correct: what was in the air five minutes and one power cycle ago is not
information.

---

## Configuration page

`tools/index.html` is one file with no dependencies. Styles, script and the
logo are inline, and it loads nothing from anywhere. It is served at
<https://beacon.breezedude.de/>, and publishing a new version is copying the
file there. A local copy works as well, from a web server on
`localhost` or opened directly; Web Bluetooth needs a secure origin, which
excludes plain `http` from anywhere else. It needs Web Bluetooth, which means a Chromium browser on
Windows, macOS, Linux, ChromeOS or Android. Firefox and Safari, and with them
every browser on iOS, do not have it.

| | |
|---|---|
| Language | `TEXT` holds one dictionary per language. Static text is filled in through `data-i18n` attributes, the rest through `t(key)`. The browser's preference decides, the button in the header overrides it. Adding a language is a new dictionary and one more entry in the toggle |
| Theme | Follows the system; the button overrides it. Every colour is a token defined for both themes |
| Key | The key from the beacon is kept in `localStorage`, filed under the beacon's address. Storage belongs to the origin the page was opened from, so a copy opened from somewhere else asks for the button once more |
| Settings | Every change is sent at once; typing is held back until it settles. There is no save button to forget |
| Received stations | Kept as received and put into words when drawn, so the list follows a change of language |

### Debug console

Hidden behind an **Enable debug** button in the footer, off by default and
remembered. It shows the raw configuration traffic in both directions with a
timestamp, one switch per direction, and a line cap of 300. Autoscroll sticks
to the bottom only while the reader is already there, so scrolling back through
the log is not fought by every arriving line.

**Capture NMEA** subscribes the page to the serial service, exactly as a
navigation app does, so what the console then shows is what such an app
receives. Nothing changes on the device, and the subscription ends with the
connection.

A failed connection names the step it stopped at, in brackets after the
message: `gatt`, `service`, `characteristic`, `notify` or `hello`, and the
error's name.

A configuration service that is not found, on a beacon that has one, means the
phone or computer answered from the service list it kept from an earlier
firmware. Android keeps one for every device it is paired with, Windows for
devices it has seen. The page says so and what to do: unpair or remove the
beacon in the Bluetooth settings, switch Bluetooth off and on, connect again.
A change to the services in a firmware update brings this on for anyone paired,
which is a reason to keep the service layout stable.

### Working on it without a beacon

The connection can be exercised without hardware by replacing
`navigator.bluetooth` from the browser console with an object whose
`requestDevice()` returns a stand-in: a `gatt` with `connect()` and
`getPrimaryService()`, and characteristics that answer the commands of the
protocol above. That is enough to walk through the first connection, the
button, a reconnect with a stored key and a failure.

---

## Radio driver

The chip is reset and calibrated once, in `Radio::initChip()`. After that a
change of modem is the packet type command followed by every setting of the
new modem, a few milliseconds in all. The library keeps the packet type command
to itself, because its own way of changing modem is `begin()`; `Lr1110Modem`
makes it reachable.

FSK settings are sent afresh before every FLARM and ADS-L frame rather than
only when the mode changes. A run of frames on one configuration was once seen
to stop being received after the first; that was observed together with several
other faults and never isolated, so the cheap precaution stays. Should a chip
turn out to need the full reset after all, building with
`-DRADIO_REINIT_EACH_FRAME=1` restores it, at the price of the slot timing.

Board specifics, all three from the manufacturer's board support code:

| | |
|---|---|
| Antenna switch | DIO5 and DIO8 high to receive; DIO5, DIO6 and DIO8 to transmit on the low power amplifier; DIO6 and DIO8 on the high power one |
| Regulator | DC-DC |
| TCXO | supplied from DIO3 |

The FSK receive bandwidth is 234 kHz. The signal is 200 kHz wide, 100 kbit/s
with 50 kHz deviation, and the next narrower setting cuts into it.

## Timing and channels

FLARM and ADS-L divide each UTC second into two slots, and in Europe each slot
belongs to one of two frequencies:

| | Slot 0, 400 to 800 ms | Slot 1, 800 to 1200 ms |
|---|---|---|
| FLARM | 868.2 MHz | 868.4 MHz |
| ADS-L, OGN | 868.4 MHz | 868.2 MHz |

Receivers hop between the two in step with this. A frame on the right
frequency in the wrong slot is not heard, and a ground station that listens on
one frequency all the time does not show the problem, which is how an
unsynchronised transmitter passes a bench test.

What is sent when:

| | Frequency | Nominal start, ms into the second |
|---|---|---|
| FANET | 868.2 MHz | 20 to 300, when due |
| FLARM | 868.2 MHz | 470 to 540, every second |
| ADS-L, upper channel | 868.4 MHz | 600 to 660, every other frame |
| ADS-L, lower channel | 868.2 MHz | 870 to 980, every other frame |

The moment inside each window is random per second, so devices whose clocks
agree do not transmit on top of each other. FLARM stays in slot 0 so that a
receiver on 868.2 MHz alone hears every frame as well. LoRa is not slotted, but
it occupies the radio for tens of milliseconds and is kept clear of the rest.

### Where the second starts

The board has no pulse-per-second line, so the start of the second is taken
from the arrival of the `GGA` sentence, which leads each report, minus an
assumed latency of 90 ms (`slot_clock.*`). A report can be seen late, when the
loop was busy, but never early, so the earliest of the last five is used; a
lasting shift, such as the receiver aligning its output to UTC after the first
fix, is followed within five seconds.

The latency is an assumption, not a measurement. The windows are placed so that
an error of −60 to +120 ms still lands every frame inside its slot, and a host
test holds them to that. A frame that cannot start before its deadline is
dropped for that second rather than sent in the wrong slot.

Reception does not hop yet: it stays on 868.2 MHz and so hears FLARM from slot
0 and ADS-L from slot 1 only.

## Sharing the band

All three protocols transmit between 868.0 and 868.6 MHz. ETSI EN 300 220-2
(V3.3.1, 2025) calls that band M and allows two ways of using it:

| | Duty cycle | Polite spectrum access |
|---|---|---|
| Clause | 4.4.3 | 4.6, Tables 17 and 18 |
| Before each transmission | nothing | listen at least 160 µs; defer while the channel is above the CCA threshold |
| Time on air | at most 1 % of an hour, 36 s, across the whole band | at most 100 s per hour for each 200 kHz |
| Pause on one frequency | none | at least 100 ms between two transmissions |
| Single transmission | | at most 1 s |
| Power | 25 mW e.r.p. | 25 mW e.r.p. |

Polite access may add frequency agility, moving to another channel when one is
busy, but does not have to. These protocols cannot: their frequencies are fixed.

The `lbt` setting chooses between the two, and is on by default.

### The account

`airtime.*` keeps a bucket of microseconds that fills at a steady rate and is
emptied by every transmission. Whatever the pattern of transmissions, no hour
can hold more than the capacity plus an hour of refill, and both are chosen to
stay under the limit:

| | Refill | Capacity | Worst hour | Limit |
|---|---|---|---|---|
| Duty cycle | 9 ms per second | 3 s | 35.4 s | 36 s |
| Polite | 25 ms per second | 9 s | 99 s | 100 s |

The polite limit applies per 200 kHz, so 868.2 and 868.4 MHz could each have
their own. One account for both is stricter than required and simpler. The
host tests in `test/test_airtime` drive the account with a sender that takes
every chance it gets, for hours and across the millisecond counter wrapping,
and check the worst hour.

A frame is admitted on its computed time on air (`airtime::loraUs`,
`airtime::fskUs`) and booked at the longer of that and the measured time the
transmit call took, which includes starting the transmitter.

| Frame | Time on air |
|---|---|
| FANET tracking, 15 bytes | 25.2 ms |
| FANET ground tracking, 11 bytes | 22.7 ms |
| FANET name, up to 28 bytes | up to 35.5 ms |
| FLARM | 4.9 ms |
| ADS-L | 4.6 ms |

### Who gives way

FLARM and ADS-L are sent once a second in fixed slots, and a frame that misses
its slot is lost. FANET is not slotted. So FANET is the one that gives way: it
is only sent while the account holds, besides the frame itself, two seconds'
worth of whichever slotted protocols are switched on. A FANET frame that does
not fit stays due and is tried again in the next second.

What that means in practice, while moving:

| Switched on | With `lbt` | Without |
|---|---|---|
| FANET, every 2 s | every 2 s | about every 2.8 s |
| FANET, every 1 s | nearly every second | about every 2.8 s |
| FANET every 2 s, FLARM, ADS-L | all as set | FLARM and ADS-L most seconds, FANET hardly at all |

Lying still, every protocol drops to one frame in ten seconds, which fits
either way.

### Listening

Before each frame the radio is switched to receive on that frequency, with the
modem set for the protocol, and the received level is sampled eight times over
two milliseconds (`Radio::channelClear()`). If the loudest sample is at or
above −85 dBm the channel counts as busy, and the frame is tried again after a
random 5 to 25 ms, still inside its window; past the window's deadline it is
dropped for that second. The 100 ms pause per frequency is handled the same
way.

The standard measures the threshold at the antenna: 15 dB above the
sensitivity it requires, −79 dBm for 200 kHz and −78 dBm for 250 kHz of
channel, assuming an antenna with no gain over a dipole. This one has less,
which would allow a higher figure at the chip. −85 dBm stays clear of that and
far above the noise floor.

The cost is a little over two milliseconds of receive current per frame. With
one to three frames a second that averages out well under 0.1 mA.

The pause per frequency interacts with the windows. FANET and FLARM share
868.2 MHz, and a long FANET frame late in its window can push FLARM towards the
end of its own. FLARM then goes out later in the slot, or not at all in that
second.

### Transmit power

The band allows 25 mW e.r.p., about 14 dBm, which is the default. The setting
goes up to the chip's 22 dBm, and the firmware does not cap it: the antenna in
the case is small, and what it radiates is left to the user's judgement.

## Radio formats

### FANET

FANET is openly documented by its author:
<https://github.com/3s1d/fanet-stm32/blob/master/Src/fanet/radio/protocol.txt>

The implementation follows that document. Parameters for the European band:

```
868.2 MHz, 250 kHz bandwidth, SF7, coding rate 4/5, sync word 0xF1, preamble 12
coordinates: latitude × 93206, longitude × 46603, 24 bit little endian
header: type:6 | forward:1 | extended:1, vendor:8, address:16
altitude 11 bit metres with a 4× scale bit, speed in 0.5 km/h with a 5× scale
bit, climb in 0.1 m/s with a 5× scale bit, heading as 360/256 degrees
```

### FLARM legacy

FLARM is a proprietary air interface without a public specification. The field
layouts, scaling rules, key derivation and checksum parameters used here are
protocol facts taken from reverse-engineering notes circulated publicly without
any author, copyright or licence statement.

Physical layer: 2-FSK, 100 kbit/s, 50 kHz deviation, Gaussian shaping BT 0.5,
sync word `55 99 A5 A9 55 66 65 96`, preamble 8, fixed frame length.

Frame construction: a 24 byte payload, checksum appended, then the whole thing
Manchester encoded to 52 bytes.

Several details in this implementation differ from what the available
descriptions imply. Each was established by comparing transmitted frames against
recordings from a receiving ground station, and each is documented at the point
in the code where it matters and covered by a host test:

| | | Where |
|---|---|---|
| Framing | Fixed length is mandatory. In variable length mode the radio prepends a length byte and shifts the whole frame: measurable on a spectrum analyser, accepted by no receiver | `radio.cpp` |
| Manchester | 1 → 01, 0 → 10. The other way round, the receiver synchronises and decodes the exact bitwise complement of every byte | `proto/bits.h` |
| Payload | Not inverted. Inverting it and using the wrong Manchester polarity cancel out, which is how both mistakes survive together | `proto/flarm.cpp` |
| Checksum | CRC-16 with seed `0x051E`, not `0xFFFF`. Found by trying all 65536 seeds against three frames a receiver had accepted | `proto/flarm.cpp` |
| Time | The timestamp keys the cipher and has to be exact UTC. `mktime()` reads its input as local time and is wrong everywhere outside UTC | `main.cpp` |

The two frame formats differ substantially:

| | V6 | V7 |
|---|---|---|
| Header type | 0 | 2 |
| Coordinates | `>> 7` | `/ 52`, longitude divisor varies with latitude |
| Altitude, speed, climb | linear with scale bits | pseudo floating point |
| Encryption | XXTEA over words 1–5 | XOR with a derived key, then XXTEA over words 2–5 |
| Parity | dedicated bit | none |

Key mixing uses XXTEA by Needham and Wheeler, whose reference implementation is
in the public domain.

### ADS-L

ADS-L is the position reporting protocol defined by ED 2022/024/R for light
aircraft (SRD-860, 868 MHz band). There is no public specification; the field
layouts, scaling rules and checksum parameters used here were derived from the
reference decoder and are pinned by a golden vector in the host tests.

Physical layer: 2-FSK, 100 kchip/s, 50 kHz deviation, Gaussian shaping BT 0.5,
sync word `55 99 95 A6 9A 65 A9 6A`, preamble 8, fixed frame length. The two
channels 868.2 MHz and 868.4 MHz (spec C.2) are alternated for every
transmission, reconfiguring the modem each time.

Frame construction: a 1 byte version plus 20 byte body plus CRC-24
(`0xFFFA0480`, MSB first, single bit error correction by syndrome), scrambled
with XXTEA over the first 21 bytes using key index 0 (5 words, 6 rounds), then
Manchester encoded to 48 bytes on the air. The version byte encodes the key
index: `0x00` is key 0, `0x03` is plaintext. The decoder accepts both and
rejects frames using any derived key. The body carries:

```
type:8 = 0x02, address:32 (bits 0-5 hold the address mapping table, default 7 = OGN),
time stamp:6 (seconds, wraps at 60), flight state:2 (1 ground, 2 airborne),
aircraft category:5 (EASA G.1.3), emergency:3 (1 = no emergency),
latitude x 93206 and longitude x 46603, 24 bit each,
speed in 0.25 m/s (unsVR6), altitude in metres + 320 (12 bit, saturated),
climb in 0.125 m/s (signVR6), track as 360/512 degrees
```

---

## Known limitations

- FLARM and ADS-L are not yet confirmed on air with their time slots, see
  [Feature status](#feature-status).
- Listening before each frame is not yet confirmed on hardware, and the
  threshold has not been checked against a calibrated signal.
- Without `lbt`, FLARM and ADS-L together use nearly the whole 1 % on their
  own, and FANET is then sent rarely, see [Who gives way](#who-gives-way).
- The configuration link is not encrypted, see
  [Who may configure](#who-may-configure).
- The configuration page has only been used with a Chromium browser on Windows.
- There is no firmware update over Bluetooth, see
  [Bootloader](#no-update-over-bluetooth).
- LFCLK runs on the internal RC oscillator. Whether a 32.768 kHz crystal is
  fitted was never established; RC is the safe choice but costs current.
- The current while switched off has not been measured, see [Power](#power).
- The latency of the GNSS report, which the transmit slots are timed from, is
  assumed rather than measured.
- Reception stays on one frequency and therefore misses the slot in which the
  protocol being listened for uses the other one.
- The motion threshold for the auto-off is set conservatively and has not been
  validated in flight.
- The I²C device at `0x7E` is unidentified.
- Turn rate is not derived and is transmitted as zero in FLARM V7.
- Reception handles one protocol at a time, which is a property of having one
  radio rather than a software limitation.
- FANET weather wind and gust scaling follows the published convention of a
  seven bit value in 0.2 km/h with a five times scale bit; it has not been
  checked against a station of every make.
