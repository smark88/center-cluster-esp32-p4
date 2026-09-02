#include "dash_demo.h"

#if DASH_DEMO_MODE

#include <math.h>
#include "esp_timer.h"
#include "ui.h"          // WARN_* thresholds

#define DEMO_SWEEP_SEC 8.0f     // seconds for one idle -> redline -> idle pull
#define DEMO_FUEL_SEC  45.0f    // seconds to drain the tank from full to empty
#define DEMO_IDLE_RPM  800.0f
// The simulated engine's own ceiling. Deliberately not tied to the dial scale
// -- this gauge reads mph, and the demo still has to produce plausible rpm.
#define DEMO_MAX_RPM   7000.0f
#define DEMO_WARN_SEC  8.0f     // seconds per warning phase

static int64_t s_t0_us = 0;

void dash_demo_start(void)
{
    s_t0_us = esp_timer_get_time();
}

void dash_demo_sample(dash_demo_t *out)
{
    if (!out) return;

    float t = (float)(esp_timer_get_time() - s_t0_us) / 1000000.0f;   // seconds

    // RPM: idle -> redline -> idle, one pull every DEMO_SWEEP_SEC.
    float phase = fmodf(t, DEMO_SWEEP_SEC) / DEMO_SWEEP_SEC;          // 0..1
    float ramp  = (phase < 0.5f) ? (phase * 2.0f)                     // pulling
                                 : (1.0f - (phase - 0.5f) * 2.0f);    // backing off
    out->rpm = DEMO_IDLE_RPM + ramp * (DEMO_MAX_RPM - DEMO_IDLE_RPM);

    // Speed tracks RPM as if we were held in one long gear.
    out->speed_mph = out->rpm / 50.0f;

    // Fuel drains then refills, so the reserve marker at E gets exercised.
    out->fuel_pct = 100.0f - (fmodf(t, DEMO_FUEL_SEC) / DEMO_FUEL_SEC) * 100.0f;

    // Oil pressure rises with RPM.
    out->oil_psi = 15.0f + (out->rpm / DEMO_MAX_RPM) * 60.0f;

    // Temps warm from cold, then hold with a slow ripple.
    float warm   = 1.0f - expf(-t / 20.0f);
    float wobble = sinf(t * 0.6f) * 3.0f;
    out->water_f    = 100.0f + warm * 95.0f  + wobble;
    out->oil_temp_f = 100.0f + warm * 115.0f + wobble;
    out->trans_f    = 100.0f + warm * 80.0f  + wobble;

    // Gauge two channels.
    out->iat_f     = 90.0f + warm * 50.0f + wobble;          // 90 -> ~140F
    out->fuel_psi  = 55.0f - (out->rpm / DEMO_MAX_RPM) * 8.0f;
    out->afr       = 14.7f - (out->rpm / DEMO_MAX_RPM) * 3.5f;  // richens on boost
    out->boost_psi = -8.0f + (out->rpm / DEMO_MAX_RPM) * 26.0f;

    // Walk the selector so the PRNDM row can be seen switching.
    {
        static const char sel[] = "PRNDM";
        out->gear = sel[((int)(t / 3.0f)) % 5];
    }

#if DEMO_EXERCISE_WARNINGS
    // Drive one channel past its limit at a time, then a quiet phase, so each
    // tile can be seen flashing on its own. Offsets are taken from the real
    // thresholds, so retuning a threshold keeps the demo honest.
    switch (((int)(t / DEMO_WARN_SEC)) % 4) {
        case 1:
            out->iat_f = WARN_IAT_MAX + 20.0f;
            break;
        case 2:
            out->fuel_psi = WARN_FUEL_PSI_MIN - 12.0f;
            break;
        case 3:
            // AFR only alarms under load, so force the rpm gate open too.
            out->afr = WARN_AFR_MAX + 1.5f;
            out->rpm = (float)(WARN_AFR_MIN_RPM + 800);
            break;
        default: break;   // phase 0: everything in range
    }
#endif
}

#endif // DASH_DEMO_MODE
