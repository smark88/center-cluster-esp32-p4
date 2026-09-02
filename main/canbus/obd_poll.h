// OBD-II Mode 01 poller.
//
// Fuel level, intake air temp, boost, fuel pressure and AFR are not broadcast
// on GM's bus -- which is why they appear in no DBC -- but they are all
// standard SAE J1979 PIDs on the same high speed bus. This is how a scan tool
// reads them.
//
// UNLIKE THE REST OF THE CAN CODE, THIS TRANSMITS.
// Everything else in this project listens passively. Polling sends request
// frames to 0x7DF and reads the reply from 0x7E8. That is exactly what any
// scan tool does and is safe on a running vehicle, but it is an active
// participant on the bus rather than a silent observer. Set OBD_POLL_ENABLE
// to 0 to go back to listen-only.
//
// Only single-frame requests are supported. Every PID below returns one or
// two data bytes, which fits. Multi-frame PIDs such as 0x68 (IAT per bank)
// and 0x70 (boost control) would need ISO-TP reassembly and are not handled.

#ifndef OBD_POLL_H
#define OBD_POLL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 1 = poll the PIDs below, 0 = never transmit.
#define OBD_POLL_ENABLE 1

// How often the poller wakes. Each PID carries its own period in the table in
// obd_poll.c, matched to how fast that reading physically moves; at most one
// request goes out per tick so two never collide on the bus.
#define OBD_POLL_TICK_MS 20

// Functional request address and the ECU reply range.
#define OBD_REQ_ID    0x7DF
#define OBD_RESP_LO   0x7E8
#define OBD_RESP_HI   0x7EF

// Starts the polling task. Call after canbus_init().
void obd_poll_start(void);

// Feed replies in from the existing receive path. Returns true if the frame
// was an OBD reply and has been consumed.
bool obd_poll_handle_frame(uint32_t id, const uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif // OBD_POLL_H
