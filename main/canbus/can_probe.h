// OBD link probe -- for when a tile stays "--" and you need to know why.
//
// The dash polls a handful of PIDs and silently shows "--" for anything that
// never answers, which collapses four very different failures into one blank
// tile. This separates them. It runs in TWAI_MODE_NORMAL on the dash's pins,
// asks the one question every OBD-II vehicle must answer -- mode 01 PID 0x00,
// "which PIDs do you support" -- and reports what came back plus the
// controller's own error state.
//
// READING THE OUTPUT
//   frames=0, errors climbing, state bus-off
//       Physical or bitrate. CANH/CANL swapped or not on OBD pins 6 and 14,
//       or the bus is not 500k. The controller is talking into a wall.
//   frames=0, errors flat, state running
//       Wired fine, nothing is talking. Key is off, or the gateway passes no
//       broadcast traffic to the port.
//   frames counting up, no reply from 0x7E8..0x7EF
//       Bus is good and you can hear it, but nothing answers a request --
//       this is what a gateway blocking diagnostics looks like.
//   reply received
//       The link works end to end. The supported-PID bitmap then says whether
//       the PIDs the dash wants actually exist on this vehicle.
//
// HOW TO USE
//   1. Set CAN_PROBE_MODE to 1 below, build, flash.
//   2. idf.py monitor, with the board still on USB from the laptop.
//   3. Key on. Read the summary that prints every CAN_PROBE_REPORT_MS.
//   4. Set it back to 0 when done.
//
// This transmits. Nothing else runs in this mode, and the gauge-two UART link
// is left off so its packets do not bury the output on the shared console pin.

#ifndef CAN_PROBE_H
#define CAN_PROBE_H

#ifdef __cplusplus
extern "C" {
#endif

// 1 = probe the OBD link and log, nothing else runs. 0 = normal dash.
#define CAN_PROBE_MODE 0

// Must match the bus. 500k is ISO 15765-4 on every OBD-II car worth the name.
#define CAN_PROBE_BITRATE 500000

// How often to ask, and how often to summarise.
#define CAN_PROBE_REQUEST_MS 1000
#define CAN_PROBE_REPORT_MS  3000

// Log every frame as it arrives, not just the summary. Loud on a live bus.
#define CAN_PROBE_VERBOSE 0

// Slowly toggle the driver between recessive and dominant, forever, so the
// CANH/CANL screw terminals can be watched with a DMM. This is the only way
// to prove the terminal block actually reaches the chip: the DC test loops
// inside the chip and cannot see that joint, and a 120 ohm reading only shows
// the terminator sits across the terminals, not that its net gets to the pins.
// Unplug from the car first -- a live bus fights the driver and muddies it.
#define CAN_PROBE_SLOW_DRIVE 0
#define CAN_PROBE_SLOW_MS    3000

void can_probe_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif // CAN_PROBE_H
