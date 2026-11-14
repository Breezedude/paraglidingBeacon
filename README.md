# Breezedude Beacon

A position beacon for free flight, running on the **Seeed SenseCAP T1000-E**.
It transmits on FANET, FLARM and ADS-L, feeds your navigation app over
Bluetooth, and is set up from a web page in your browser.

Built to be easy to live with: install by drag and drop, configure without an
app, and it looks after its own battery.

It is firmware for the European 868 MHz band, and a hobby project: see
[Before you fly with it](#before-you-fly-with-it).

---

## What it does

**Knows whether you are walking or flying**, and says so: on the ground you
appear as a walker rather than as an aircraft standing still in the air. It
works this out from your movement, and is built so that soaring into wind with
no ground speed does not read as a walk.

**Sends your position on three protocols at once.** FANET reaches other pilots
and the OGN ground station network. FLARM reaches gliders and towplanes. ADS-L
reaches air traffic and light aircraft (ED 2022/024/R, 868 MHz). All three run
from the one radio in the device, so you appear on all of them.

**Configured from your browser.** Open
[beacon.breezedude.de](https://beacon.breezedude.de/), press connect, change
your name and settings. Nothing to install.

**Can listen as well.** Switched on, it passes other aircraft to your
navigation app as standard traffic sentences, weather stations included.

**Looks after the battery.** It switches itself off after a set time without
movement, so a forgotten device in your rucksack does not run flat. It also
switches off before the battery is damaged by running empty.

**Tells you what it is doing** through the LED and short melodies, without you
having to look at a screen.

---

## Installing the firmware

The firmware is a `.uf2` file, and installing it is a drag and drop with no
tools required. Take it from the
[releases](https://github.com/Breezedude/paraglidingBeacon/releases), or build
it yourself as described in [DEVELOPMENT.md](DEVELOPMENT.md#building).

### Which file

| File | When |
|---|---|
| `firmware.uf2` | A T1000-E as delivered, or one that has only run Meshtastic. Also for every later update |
| `firmware-full.uf2` | A device on which other firmware has replaced the Bluetooth stack |

If in doubt, open `INFO_UF2.TXT` on the drive the device shows in step 2. A
line `SoftDevice: S140 7.3.0` means the first file is the right one. Anything
else there means the second, which brings that version along. Neither file
touches the bootloader, so a copy that goes wrong can simply be repeated.

The first start clears the settings that earlier firmware stored on the
device.

### Steps

1. Have the right `.uf2` file ready.
2. **Hold the button down** and plug the device into USB. Keep holding, unplug,
   and plug it in a second time **within about half a second**. The easiest way
   is to flick the magnetic connector sideways and let it snap back. The device
   now appears as a USB drive.
3. Copy the `.uf2` file onto that drive.
4. The device restarts by itself and the drive disappears, which is normal. It
   comes back in the state it was in before: if it was switched off, switch it
   on as usual.

If no drive appears, try the double connection again and hold the button a
little longer.

The configuration page can also put the device into the bootloader directly.
Doing that by accident away from a computer costs nothing: with no cable
attached it waits about three seconds and then starts up normally again.

### Going back to other firmware

The bootloader and the Bluetooth stack stay as they were delivered, so
Meshtastic installs the same way it did before: copy its file onto the drive.

---

## Using it

| Situation | Button | LED | Sound |
|---|---|---|---|
| **Turn on** | Press briefly, release, then press again and hold about half a second | Green fades up, red flashes alongside | Short tone, then a rising three-tone melody |
| **Searching for satellites** | — | Amber, red blinking out twice a second | — |
| **Satellite count** | — | Short dark gaps in the amber, one per satellite found | — |
| **Position acquired** | — | Amber stops | Two rising tones |
| **Position lost** | — | Amber returns | Two falling tones, after ten seconds |
| **Settings page asking for access** | Press briefly to allow | Alternating green and red | Two-tone question; a click as you press, then a rising confirmation |
| **Transmitting** | — | Short green flash for each transmission | — |
| **Lying still** | — | Very short green blink once a second | — |
| **Check status** | Press briefly | — | Single beep |
| **Turn off** | Hold three seconds, then release | Green fades down, red flashes alongside | Falling three-tone melody |
| **Auto-off after no movement** | — | Green fades down, red flashes | Falling melody |
| **Battery too empty to start** | — | Stays dark | Low double tone |
| **Battery empty, shutting down** | — | Green fades down | Sad, drooping melody |
| **Cable plugged in while off** | — | Stays dark | Two short rising tones |
| **Cable pulled out while off** | — | Stays dark | Two short falling tones |

A few things worth knowing:

**Turning on takes two presses on purpose**, so the beacon cannot switch itself
on in your rucksack. If you are too slow between them it simply stays off.

**Turning off happens when you let go.** Hold the button for three seconds and
the falling melody plays; the device switches off the moment you release.
Holding on longer only postpones it. Once the melody has played there is no way
back, so if you did not mean it, switch it on again.

**Charging does not switch it on or off.** Plugging in or pulling out the cable
leaves the beacon as it was: off stays off, with a short tone to say the cable
was noticed, and on stays on. It can be switched on and off while charging, in
the usual way.

**Lying still, it goes quiet rather than silent.** After about fifteen seconds
without movement it drops to one position every ten seconds instead of the full
rate. That is where most of the battery goes on a day it spends in a rucksack.
The LED keeps blinking roughly once a second, but noticeably shorter than the
flash for a real transmission, so you can still see it is alive. Any movement
puts it straight back to the full rate.

**After half an hour without movement it switches itself off**, with the same
falling melody. The time is adjustable and can be turned off entirely. This is
what stops a forgotten beacon from running the battery flat, and it only works
while the motion sensor is answering: a device that cannot tell whether it is
moving never shuts itself down.

**Amber means waiting, green flashes mean working.** That is the short version
of the whole table.

**It only transmits with a live position.** If the fix is lost, transmission
stops and, after ten seconds, the falling melody tells you. Short dropouts stay
silent. Sending the last known position would put you somewhere you are not,
which is worse than being absent.

**The settings page has to be allowed once.** The first time a browser opens
the settings, the beacon asks with a two-tone sound and an alternating green
and red LED, and the page shows what it is waiting for. Until you press the
button it hands out no settings and accepts no changes. A browser you have
allowed connects without asking from then on. If nobody presses, the
connection is dropped after half a minute.

The buzzer volume is adjustable, including off entirely.

---

## Settings

The settings live in a web page:
**[beacon.breezedude.de](https://beacon.breezedude.de/)**. Open it in
**Chrome or Edge**, on a computer or an Android phone. It needs Web Bluetooth,
which Firefox, Safari and every browser on an iPhone or iPad do not have.
Nothing is installed.

Press **Connect beacon** and pick your device. It is named `BreezedudeBeacon-`
followed by four characters. The first time, the beacon wants a short press on
its button before it lets the page in. The page is in English or German,
following the browser; the button at the top switches it.

Worth setting at least once:

- **Name** — what other pilots see; until you set one it is `Beacon-`
  followed by the same four characters as in the Bluetooth name
- **Aircraft** — paraglider, hang glider, glider and so on
- **Transmit** — FANET, FLARM and ADS-L can each be switched on or off, and
  **Online visibility** says whether tracking sites may show you
- **Listen before talk** — on by default. The beacon checks that the channel is
  free before each transmission, which lets it use more airtime. Switched off,
  it keeps to the 1 % airtime the band allows otherwise, and FANET then goes out
  less often than set
- **Identifiers** — normally nothing to change. Each device gets its own
  address from its chip, and all three protocols use it, so you are one
  aircraft everywhere. Register it at
  [ddb.glidernet.org](https://ddb.glidernet.org/) (device type OGN) to be shown
  by name on tracking sites
- **GPS standby** — keeps the receiver's clock and satellite data alive while
  the device is off, so the next start has a position within seconds; it costs
  a little battery
- **Sounds** — the volume; the slider plays a sample when you let go of it
- **Auto-off without movement** — 30 minutes by default
- **Enable reception** — receiving other aircraft, off by default; see below

Changes take effect as you make them and are stored shortly afterwards, so
there is nothing to confirm.

The page also shows live status: satellites, position, battery, how long the
device has been lying still, and transmission counters for FANET, FLARM and
ADS-L.

---

## Connecting a navigation app

The beacon sends its position as NMEA over Bluetooth LE, on the serial service
most flight apps know as Nordic UART. In the app, choose
`BreezedudeBeacon-…` as the Bluetooth LE GPS or FLARM device. No pairing and no
button press are needed, and the app and the settings page can be connected at
the same time.

That also means anyone within Bluetooth range can read your position from it.
The beacon broadcasts the same position by radio on purpose, so nothing is
given away that was not public already.

---

## Receiving other aircraft

The beacon can also listen, and pass what it hears to your navigation app as
standard traffic sentences. Switch **Enable reception** on and choose a
protocol.

Two things are worth understanding before you rely on it:

**Only one protocol at a time.** The device has a single radio, and FANET, FLARM
and ADS-L use it differently. Listening for one means not hearing the others.

**It costs battery.** Receiving keeps the radio powered continuously, which is
the largest single draw on this device. That is why it is off by default.

With FANET selected you also receive weather stations. The configuration page
lists what it decodes under **Received stations**.

---

## What it does not do

**It does not warn you.** No collision detection happens on the device. It
forwards what it hears and leaves the judgement to your navigation app, which
knows your route and your settings.

**It cannot hear what it is not listening to.** Aircraft on the other two
protocols are invisible to it, as is anything out of range or not transmitting
at all. Treat what you see as incomplete, always.

---

## Before you fly with it

**Europe only.** The frequencies are those of the European 868 MHz band. The
firmware has no settings for other regions and must not be used there.

**Transmit power is your responsibility.** The band allows 25 mW e.r.p., about
14 dBm, which is the default. The setting goes higher, and the small antenna in
the case loses some of it, but what leaves the device is up to you.

**Not certified, and not a safety device.** It is not approved equipment of
any kind, it does not replace looking out, and nobody guarantees that another
aircraft receives you. FLARM and ADS-L in particular are still experimental
here: FANET is the protocol that has been checked against a ground station
most thoroughly.

**Independent.** This project is not affiliated with or endorsed by Seeed
Studio, FLARM Technology or the Open Glider Network. Names and trademarks
belong to their owners.

**Third-party software.** The firmware files contain libraries under their own
licences, listed in [LICENSE.txt](LICENSE.txt). `firmware-full.uf2` also
contains the SoftDevice S140 7.3.0 by Nordic Semiconductor, unchanged, under
[Nordic's licence](licenses/nordic-softdevice.txt).

---

## For developers

Build instructions, pin assignments, the Bluetooth protocol and notes on the
radio formats are in [DEVELOPMENT.md](DEVELOPMENT.md).

---

## Licence

Free for non-commercial use under
[CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/). Commercial use
needs a separate agreement; see [LICENSE.txt](LICENSE.txt).

This is a hobby project, published in the hope that it is useful.
