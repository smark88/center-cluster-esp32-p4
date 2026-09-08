// Gauge-to-gauge bridge over CAN.
//
// Both gauges want overlapping readings and both were polling OBD for them --
// two nodes asking the same ECU the same questions, with three PIDs duplicated
// between them. This lets one gauge do all the polling and broadcast the
// results for the other to pick up, so only one node ever transmits and the
// second is a pure listener.
//
// WHY THESE IDS
// 0x7F0-0x7F2. Three reasons, in order of how much they matter:
//   1. CAN arbitrates by id, lowest wins, so these are near the bottom of the
//      11-bit priority space. Our frames yield to everything the vehicle is
//      saying rather than delaying it.
//   2. They sit above the ISO 15765-4 diagnostic block -- 0x7DF functional
//      request, 0x7E0-0x7EF request/response -- so they cannot collide with
//      the OBD traffic this project already generates.
//   3. Nothing near 0x7F0 appears in the frames observed on either test car.
//      An FR-S broadcasts 0x018 through 0x6E2 and stops there.
//
// Picking ids that only hybrids and EVs use would also work, but that is not a
// portable namespace -- it varies by manufacturer and some of it overlaps with
// ICE ids. A high, low-priority id outside the diagnostic block is defensible
// on any bus, and CAN_SCAN_MODE can confirm it is clear on a new car before
// you trust it.
//
// FRAME LAYOUT
// Every slot is a little-endian int16. Scale is x10 except rpm, which is sent
// as-is. INT16_MIN means "no reading", so a value the publisher does not have
// arrives as NAN on the subscriber and renders as "--" rather than as zero.
//
//   0x7F0  rpm          speed        coolant      oil temp
//   0x7F1  fuel level   fuel psi     AFR          intake air temp
//   0x7F2  boost        oil psi      trans temp   fuel comp

#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_BRIDGE_OFF        0
#define CAN_BRIDGE_PUBLISH    1   // polls OBD and broadcasts the results
#define CAN_BRIDGE_SUBSCRIBE  2   // transmits nothing, reads the broadcast

// Set per gauge. The publisher must poll every PID both gauges need; the
// subscriber should also set OBD_POLL_ENABLE to 0 so it stays silent.
#define CAN_BRIDGE_MODE       CAN_BRIDGE_SUBSCRIBE

#define CAN_BRIDGE_BASE_ID    0x7F0
#define CAN_BRIDGE_FRAMES     3

// 20 Hz. The display refreshes at GAUGE_TIMER_MS (33ms) and the fastest thing
// on it is the RPM arc, so anything quicker is redrawing the same number.
// Three frames at this rate is 60 frames/sec against a bus that carries well
// over a thousand.
#define CAN_BRIDGE_PERIOD_MS  50

// Call every loop on the publisher; it rate-limits itself.
void can_bridge_publish(void);

// Returns true if the frame belonged to the bridge and was consumed.
bool can_bridge_handle_frame(uint32_t id, const uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif // CAN_BRIDGE_H
