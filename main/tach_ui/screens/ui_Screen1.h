// Center cluster dash screen.
// Drawn entirely with LVGL vectors (no background bitmap) so it scales to any
// round panel size -- 720x720 (4in) and 800x800 (3.4in) both work.

#ifndef UI_SCREEN1_H
#define UI_SCREEN1_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

// Full-scale RPM shown on the outer dial. Numbers 0..(TACH_MAX_RPM/1000) are
// drawn automatically, so bumping this to 9000 gives a 0-9 dial.
#define TACH_MAX_RPM      7000
// RPM at which the dial ticks and the top number turn red.
#define TACH_REDLINE_RPM  6400

// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
void ui_Screen1_screen_destroy(void);

extern lv_obj_t *ui_Screen1;

// Driven from main.c
extern lv_obj_t *ui_rpm_arc;             // lv_arc, range 0..TACH_MAX_RPM
extern lv_obj_t *ui_fuel_arc;            // lv_arc, range 0..100
extern lv_obj_t *ui_label_rpm_value;
extern lv_obj_t *ui_label_odometer_value;
extern lv_obj_t *ui_val_oil_psi;
extern lv_obj_t *ui_val_water;
extern lv_obj_t *ui_val_oil_temp;
extern lv_obj_t *ui_val_trans;

// Convenience setters. Pass NAN for "no data" and the tile shows "--".
// ui_dash_set_rpm drives both the outer arc and the big centre readout.
// Speed is NOT shown on this gauge -- it lives on the second cluster.
void ui_dash_set_rpm(int rpm);
void ui_dash_set_fuel_pct(float pct);
void ui_dash_set_oil_psi(float psi);
void ui_dash_set_water_f(float degf);
void ui_dash_set_oil_temp_f(float degf);
void ui_dash_set_trans_f(float degf);
void ui_dash_set_mileage(double miles);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
