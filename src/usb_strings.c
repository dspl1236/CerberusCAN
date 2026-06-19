/* Custom USB descriptor strings — brands the device as "CerberusCAN" (Teensy 4.1) or
 * "Orthrus" (Teensy 4.0) instead of the generic "Dual Serial" / "USB Serial".
 *
 * Mechanism: the Teensy core declares these as WEAK aliases (see cores/.../usb_desc.c),
 * so a strong definition here overrides them at link time — no framework edits, works in
 * CI, no global side effects. This is PJRC's sanctioned override via usb_names.h.
 *
 * NOTE on what this changes: the USB *descriptor* iProduct/iManufacturer strings. Tools that
 * read USB descriptors (our Console via pyserial, USBView, the PlatformIO/Arduino port menus,
 * the "Bus reported device description" in Device Manager Details) will show the brand. The
 * top-line Windows Device Manager friendly name ("USB Serial Device (COMx)") is set by the
 * in-box usbser.sys INF and needs a custom signed INF to change — out of scope here.
 *
 * Do NOT override usb_string_serial_number: the core fills it from the chip's unique ID, which
 * keeps each unit distinct (and our Console matches the two CDC ports by that serial number).
 */
#include "usb_names.h"

/* bLength = 2 + (char count) * 2 ; bDescriptorType = 3 (STRING) ; wString = UTF-16 chars */

#if defined(ARDUINO_TEENSY40)
/* "Orthrus" (7) */
struct usb_string_descriptor_struct usb_string_product_name = {
    2 + 7 * 2, 3, {'O','r','t','h','r','u','s'}
};
#else
/* "CerberusCAN" (11) — Teensy 4.1 flagship + the fallback */
struct usb_string_descriptor_struct usb_string_product_name = {
    2 + 11 * 2, 3, {'C','e','r','b','e','r','u','s','C','A','N'}
};
#endif

/* "dspl1236" (8) */
struct usb_string_descriptor_struct usb_string_manufacturer_name = {
    2 + 8 * 2, 3, {'d','s','p','l','1','2','3','6'}
};
