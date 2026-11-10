# ---------------------------------------------------------------------------
#  Post build step: firmware.hex -> firmware.uf2 and firmware-full.uf2
# ---------------------------------------------------------------------------
#  The board carries a UF2 bootloader, so firmware can be installed by copying
#  a file onto a mass storage volume instead of running a DFU tool.
#
#    firmware.uf2        the application alone. For a device that carries
#                        SoftDevice S140 7.3.0, which is how the T1000-E is
#                        delivered.
#
#    firmware-full.uf2   SoftDevice S140 7.3.0 and the application together.
#                        For a device on which other firmware has replaced the
#                        SoftDevice: an application only runs on the version it
#                        was built for.
#
#  The bootloader counts the SoftDevice as part of the application space, so a
#  UF2 may carry it: everything from the end of the MBR up to the bootloader is
#  written, and blocks addressed to the MBR are skipped. Neither file touches
#  the bootloader or the MBR, which is what makes a failed copy recoverable.
#
#  The SoftDevice is taken out of a bootloader image that ships with the
#  Arduino core, where it sits between the MBR and the application. The image
#  is one for the nRF52833, the board the core builds against 7.3.0; the
#  SoftDevice is the same file for both chips, and nothing else is taken.
#
#  Family id 0xADA52840 identifies the nRF52840. Flash addresses come from the
#  hex files; the application starts at 0x27000.
# ---------------------------------------------------------------------------

Import("env")

import glob
import os

UF2_FAMILY = "0xADA52840"

MBR_SIZE  = 0x1000
APP_START = 0x27000

# The SoftDevice describes itself in a block 0x2000 into its image.
SD_INFO        = MBR_SIZE + 0x2000
SD_MAGIC       = 0x51B1E5DB
SD_FWID_7_3_0  = 0x0123

platform = env.PioPlatform()
framework_dir = platform.get_package_dir("framework-arduinoadafruitnrf52") or ""
uf2conv = os.path.join(framework_dir, "tools", "uf2conv", "uf2conv.py")


def read_hex(path):
    """Intel HEX file -> {address: byte}."""
    memory = {}
    base = 0
    with open(path) as source:
        for line in source:
            line = line.strip()
            if not line.startswith(":"):
                continue
            record = bytes.fromhex(line[1:])
            count, offset, kind = record[0], (record[1] << 8) | record[2], record[3]
            data = record[4:4 + count]
            if kind == 0x00:
                for i, byte in enumerate(data):
                    memory[base + offset + i] = byte
            elif kind == 0x02:
                base = ((data[0] << 8) | data[1]) << 4
            elif kind == 0x04:
                base = ((data[0] << 8) | data[1]) << 16
            elif kind == 0x01:
                break
    return memory


def write_hex(path, memory):
    """{address: byte} -> Intel HEX file, sixteen bytes to a record."""
    def record(offset, kind, data):
        body = bytes([len(data), (offset >> 8) & 0xFF, offset & 0xFF, kind]) + bytes(data)
        checksum = (-sum(body)) & 0xFF
        return ":" + (body + bytes([checksum])).hex().upper() + "\n"

    with open(path, "w") as target:
        upper = None
        addresses = sorted(memory)
        i = 0
        while i < len(addresses):
            start = addresses[i]
            if (start >> 16) != upper:
                upper = start >> 16
                target.write(record(0, 0x04, [(upper >> 8) & 0xFF, upper & 0xFF]))
            # A run of consecutive bytes that stays inside one record and one
            # 64 KB segment.
            run = [memory[start]]
            while (i + 1 < len(addresses) and addresses[i + 1] == addresses[i] + 1
                   and len(run) < 16 and (addresses[i + 1] >> 16) == upper):
                i += 1
                run.append(memory[addresses[i]])
            target.write(record(start & 0xFFFF, 0x00, run))
            i += 1
        target.write(record(0, 0x01, []))


def word(memory, address):
    return sum(memory.get(address + i, 0xFF) << (8 * i) for i in range(4))


def softdevice():
    """The SoftDevice region of the core's bootloader image, or None."""
    pattern = os.path.join(framework_dir, "bootloader", "feather_nrf52833_express",
                           "feather_nrf52833_express_bootloader-*_s140_7.3.0.hex")
    images = sorted(glob.glob(pattern))
    if not images:
        print("*** UF2: no SoftDevice image in the core (%s)" % pattern)
        return None

    image = read_hex(images[-1])
    region = {a: b for a, b in image.items() if MBR_SIZE <= a < APP_START}

    # Checked rather than assumed: a core that ships another SoftDevice would
    # otherwise produce a file that installs the wrong one without a word.
    if (word(region, SD_INFO + 4) != SD_MAGIC
            or word(region, SD_INFO + 8) != APP_START
            or (word(region, SD_INFO + 12) & 0xFFFF) != SD_FWID_7_3_0):
        print("*** UF2: %s does not hold SoftDevice S140 7.3.0" % images[-1])
        return None
    return region


def convert(hex_path, uf2_path):
    rc = env.Execute(
        '"$PYTHONEXE" "%s" -f %s -c -o "%s" "%s"'
        % (uf2conv, UF2_FAMILY, uf2_path, hex_path)
    )
    ok = rc == 0 and os.path.isfile(uf2_path)
    if ok:
        print("UF2 ready: %s (%d bytes)" % (uf2_path, os.path.getsize(uf2_path)))
    else:
        print("*** UF2: conversion failed (rc=%s)" % rc)
    return ok


def make_uf2(source, target, env):
    hex_path  = env.subst("$BUILD_DIR/${PROGNAME}.hex")
    uf2_path  = env.subst("$BUILD_DIR/${PROGNAME}.uf2")
    full_hex  = env.subst("$BUILD_DIR/${PROGNAME}-full.hex")
    full_uf2  = env.subst("$BUILD_DIR/${PROGNAME}-full.uf2")

    if not os.path.isfile(uf2conv):
        print("*** UF2: uf2conv.py not found (%s), skipping" % uf2conv)
        return

    if not convert(hex_path, uf2_path):
        return

    stack = softdevice()
    if stack is None:
        return
    application = read_hex(hex_path)
    if min(application) < APP_START:
        print("*** UF2: the application starts below 0x%X, no full image" % APP_START)
        return

    merged = dict(stack)
    merged.update(application)
    write_hex(full_hex, merged)
    convert(full_hex, full_uf2)

    print("To install: hold the button, plug USB in twice, copy a .uf2 onto the drive")


env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.hex",
    env.VerboseAction(make_uf2, "Building $BUILD_DIR/${PROGNAME}.uf2"),
)
