/* Memory layout for the nRF52840 under SoftDevice S140 7.3.0.
 *
 * The Arduino core ships a script for this chip under S140 6.1.1 only. The
 * T1000-E is delivered with 7.3.0, which is 4 KB larger, so the application
 * starts one page later. Everything below the memory map is the core's own
 * script, unchanged.
 */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  /* From the end of the SoftDevice to the internal file system. */
  FLASH (rx)     : ORIGIN = 0x27000, LENGTH = 0xED000 - 0x27000

  /* The SoftDevice takes its memory from the bottom of RAM, and how much it
   * needs depends on the Bluetooth configuration: attribute table, vendor
   * UUIDs, ATT MTU, number of links, event length and queue sizes. It checks
   * that against the start given here and refuses to start if it is short.
   *
   * The core reserves 24 KB. This is 32 KB: two links with a large MTU are
   * more than the usual single one, the need cannot be read off anywhere
   * except on the running device, and the application has RAM to spare.
   */
  RAM (rwx) :  ORIGIN = 0x20008000, LENGTH = 0x20040000 - 0x20008000
}

SECTIONS
{
  . = ALIGN(4);
  .svc_data :
  {
    PROVIDE(__start_svc_data = .);
    KEEP(*(.svc_data))
    PROVIDE(__stop_svc_data = .);
  } > RAM

  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

INCLUDE "nrf52_common.ld"
