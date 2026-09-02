// Bench demo mode -- drives the dash from a simulated engine so you can watch
// the arcs sweep with no CAN bus, no sensors and no car.
//
// ---------------------------------------------------------------------------
// TO REMOVE COMPLETELY (3 steps):
//   1. Delete the main/demo/ folder.
//   2. In main/CMakeLists.txt drop the "demo/dash_demo.c" SRCS line and the
//      "./demo" INCLUDE_DIRS line.
//   3. In main/main.c delete the #include "dash_demo.h" line and the two
//      #if DASH_DEMO_MODE blocks (one in gauge_timer, one in app_main).
//
// TO JUST TURN IT OFF: set DASH_DEMO_MODE to 0 below.
// ---------------------------------------------------------------------------

#ifndef DASH_DEMO_H
#define DASH_DEMO_H

#ifdef __cplusplus
extern "C" {
#endif

// 1 = simulated engine, 0 = real sensors / CAN.
#define DASH_DEMO_MODE 1

// Walk each tile past its warning threshold in turn so the red flashes can be
// verified on the bench. 0 = plausible values only, nothing ever alarms.
#define DEMO_EXERCISE_WARNINGS 1

// One simulated sample. Mirrors the values the real sensors produce.
typedef struct {
    float rpm;
    float speed_mph;
    float fuel_pct;
    float oil_psi;
    float water_f;
    float oil_temp_f;
    float trans_f;
    float iat_f;
    float fuel_psi;
    float afr;
    float boost_psi;
    char  gear;          // selector: one of P R N D M
    int   drive_gear;    // 1..8, or -1 for reverse
} dash_demo_t;

// Call once at startup to set the time origin.
void dash_demo_start(void);

// Fill `out` with the values for right now.
void dash_demo_sample(dash_demo_t *out);

#ifdef __cplusplus
}
#endif

#endif // DASH_DEMO_H
