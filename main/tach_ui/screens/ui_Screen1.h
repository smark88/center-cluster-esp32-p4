// Gauge two -- speedometer cluster.
// Drawn entirely with LVGL vectors (no background bitmap) so it scales to any
// round panel size -- 720x720 (4in) and 800x800 (3.4in) both work.

#ifndef UI_SCREEN1_H
#define UI_SCREEN1_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

// Full-scale speed on the outer dial. Numbers are drawn every
// DIAL_MPH_PER_MAJOR, so 180/20 gives 0..180 in twenties.
#define DIAL_MAX_MPH        160
#define DIAL_MPH_PER_MAJOR  20

// ---------------------------------------------------------------------------
// WARNING THRESHOLDS
// A tile flashes red while its reading is outside these limits.
// ---------------------------------------------------------------------------
#define WARN_IAT_MAX        170.0f   // flash above this
#define WARN_FUEL_PSI_MIN   40.0f    // flash below this
#define WARN_AFR_MAX        14.0f    // flash above this, but only under load

// Lean only matters under load -- cruise and overrun run lean by design, so
// the AFR warning is gated on the engine actually pulling.
#define WARN_AFR_MIN_RPM    2000

// Flash half-period.
#define WARN_FLASH_MS       450

// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
void ui_Screen1_screen_destroy(void);

extern lv_obj_t *ui_Screen1;

// Driven from main.c
extern lv_obj_t *ui_speed_arc;           // lv_arc, range 0..DIAL_MAX_MPH
extern lv_obj_t *ui_label_mph_value;
extern lv_obj_t *ui_label_gear_value;
extern lv_obj_t *ui_val_iat;
extern lv_obj_t *ui_val_fuel_psi;
extern lv_obj_t *ui_val_afr;
extern lv_obj_t *ui_val_boost;

// Convenience setters. Pass NAN for "no data" and the tile shows "--".
// ui_dash_set_speed_mph drives both the outer arc and the big centre readout.
void ui_dash_set_speed_mph(float mph);

// RPM is not displayed here -- it lives on gauge one -- but it gates the AFR
// warning, so feed it anyway.
void ui_dash_set_rpm(int rpm);

void ui_dash_set_iat_f(float degf);
void ui_dash_set_fuel_psi(float psi);
void ui_dash_set_afr(float afr);
void ui_dash_set_boost_psi(float psi);

// PRNDM selector row along the bottom.
// One of 'P' 'R' 'N' 'D' 'M'. Anything else clears the selection.
void ui_dash_set_gear(char gear);

// Which gear the transmission is actually in, shown up top.
// 1..8 draws the number, -1 draws "R", anything else draws "--".
#define GEAR_REVERSE (-1)
void ui_dash_set_drive_gear(int gear);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
