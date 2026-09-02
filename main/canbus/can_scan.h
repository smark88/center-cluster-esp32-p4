// CAN bus sniffer -- for discovering which frame and byte carries a value that
// isn't in any DBC (oil pressure, trans temp, fuel level, ...).
//
// It listens to every frame on the bus, standard and extended, and tracks the
// min/max each byte has reached since boot. Bytes that never move are noise;
// the one you're hunting is the one whose range opens up when you change the
// thing you're measuring.
//
// HOW TO USE
//   1. Set CAN_SCAN_MODE to 1 below, build, flash.
//   2. Open a serial monitor (idf.py monitor).
//   3. Let the car idle. Every CAN_SCAN_DUMP_MS it prints a table.
//   4. Change the value you're after -- rev the engine for oil pressure, let
//      it warm up for oil/trans temp, run the tank down for fuel level.
//   5. The byte whose min..max range widened in step 4 is your signal. Read
//      off the frame id and byte offset and put them in the protocol json.
//
// It runs the driver in LISTEN ONLY mode, so the ESP32 never transmits and
// never even sends ACK bits -- safe to hang on a live vehicle bus.
//
// TO REMOVE: set CAN_SCAN_MODE to 0, or delete can_scan.c/.h, the
// "canbus/can_scan.c" line in main/CMakeLists.txt, and the #if CAN_SCAN_MODE
// block in app_main.

#ifndef CAN_SCAN_H
#define CAN_SCAN_H

#ifdef __cplusplus
extern "C" {
#endif

// 1 = sniff the bus and log, nothing else runs. 0 = normal dash operation.
#define CAN_SCAN_MODE 0

// GM Global A high speed powertrain is 500k. GMLAN low speed is 33333.
#define CAN_SCAN_BITRATE 500000

// How often to print the table.
#define CAN_SCAN_DUMP_MS 5000

// Starts the driver in listen-only mode and never returns.
void can_scan_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif // CAN_SCAN_H
