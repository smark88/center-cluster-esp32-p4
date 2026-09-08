// CAN transceiver bench check -- proves the SN65HVD230 is wired and alive
// without a vehicle bus attached.
//
// WHAT IT PROVES
// A CAN transceiver is a dumb analog part: it never identifies itself, and
// with CANH/CANL unconnected there is nothing to passively receive. So this
// tests the loop instead. The controller runs in TWAI_MODE_NO_ACK (self test)
// and transmits a frame with the self reception bit set. The bits leave
// GPIO5 -> the transceiver's CTX pin, are driven onto CANH/CANL, are read
// back by the same chip's receiver, and return on CRX -> GPIO4.
//
// The controller monitors every bit it transmits. If CRX never comes back --
// transceiver unpowered, CTX/CRX swapped, a wire on the wrong header pin --
// the bit monitor faults and the frame is never self received. So:
//
//   PASS = 3.3V, GND, CTX and CRX are all correct and the chip is alive.
//   FAIL = one of those four is wrong. The error counters say which way.
//
// It does NOT prove anything about CANH/CANL out to the car -- there is no
// way to test that from this end without a second node.
//
// HOW TO USE
//   1. Set CAN_SELFTEST_MODE to 1 below, build, flash.
//   2. idf.py monitor. The result prints every CAN_SELFTEST_PERIOD_MS.
//   3. Set it back to 0 when you are done.
//
// Nothing else runs in this mode, and the gauge-two UART link is left off so
// its binary packets do not bury the output on the shared console pin.

#ifndef CAN_SELFTEST_H
#define CAN_SELFTEST_H

#ifdef __cplusplus
extern "C" {
#endif

// 1 = loopback check only, nothing else runs. 0 = normal dash operation.
#define CAN_SELFTEST_MODE 0

// Any rate works for a loopback; 500k matches GM Global A.
#define CAN_SELFTEST_BITRATE 500000

// How often to repeat the check.
#define CAN_SELFTEST_PERIOD_MS 2000

// Runs the loopback check forever and never returns.
void can_selftest_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif // CAN_SELFTEST_H
