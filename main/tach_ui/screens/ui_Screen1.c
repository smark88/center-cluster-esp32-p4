// Center cluster dash screen.
//
// Everything here is drawn with LVGL vectors instead of a background bitmap,
// so the layout follows the panel resolution: all geometry is authored against
// a 720x720 reference and scaled through S() at runtime.

#include "../ui.h"

#include <math.h>
#include <stdio.h>

// ---------------------------------------------------------------- geometry --

// Layout reference resolution. Everything below is in these units.
#define UI_BASE_RES 720
#define S(x)  ((lv_coord_t)(((int32_t)(x) * (int32_t)LV_HOR_RES) / UI_BASE_RES))

// Dial sweep: 0 sits lower-left, numbers climb over the top, max sits
// lower-right. LVGL angles are clockwise with 0 at 3 o'clock.
#define DIAL_START_ANGLE  150.0f
#define DIAL_SWEEP        240.0f

#define TACH_MAJORS       (TACH_MAX_RPM / 1000)
#define TICKS_PER_MAJOR   5
#define TICK_COUNT        (TACH_MAJORS * TICKS_PER_MAJOR + 1)

// Radii (720-space)
#define R_TICK_OUTER      300
#define R_TICK_MAJOR_IN   274
#define R_TICK_MINOR_IN   287
#define R_NUMBERS         257
#define R_RPM_ARC         316
#define R_FUEL_ARC        270
#define R_FUEL_LABELS     292

#define DIAL_BOX          660   // container that holds ticks + numbers

// ------------------------------------------------------------------ colors --

#define C_BEZEL      0x131313
#define C_FACE       0x070707
#define C_TICK_MAJOR 0xFFFFFF
#define C_TICK_MINOR 0x5A5A5A
#define C_RED        0xE01010
#define C_TRACK      0x232323
#define C_MUTED      0x8A9099
#define C_TILE_LINE  0x2A2E33
#define C_TILE_BG    0x0B0D0F

// ----------------------------------------------------------------- objects --

lv_obj_t *ui_Screen1 = NULL;
lv_obj_t *ui_rpm_arc = NULL;
lv_obj_t *ui_fuel_arc = NULL;
lv_obj_t *ui_label_mph_value = NULL;
lv_obj_t *ui_label_odometer_value = NULL;
lv_obj_t *ui_val_oil_psi = NULL;
lv_obj_t *ui_val_water = NULL;
lv_obj_t *ui_val_oil_temp = NULL;
lv_obj_t *ui_val_trans = NULL;

// lv_line keeps a pointer to the caller's points, so they must outlive it.
static lv_point_t s_tick_pts[TICK_COUNT][2];

// ----------------------------------------------------------------- helpers --

static lv_obj_t *make_disc(lv_obj_t *parent, lv_coord_t size, uint32_t color)
{
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_set_size(o, size, size);
    lv_obj_center(o);
    lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(o, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(o, 0, 0);
    lv_obj_set_style_pad_all(o, 0, 0);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE);
    return o;
}

static lv_obj_t *make_label(lv_obj_t *parent, const char *txt,
                            const lv_font_t *font, uint32_t color,
                            lv_coord_t dx, lv_coord_t dy)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    lv_obj_align(l, LV_ALIGN_CENTER, dx, dy);
    return l;
}

// A 190x66 bordered tile with a muted caption and a large value underneath.
static lv_obj_t *make_tile(lv_obj_t *parent, const char *caption,
                           lv_coord_t dx, lv_coord_t dy)
{
    lv_obj_t *tile = lv_obj_create(parent);
    lv_obj_set_size(tile, S(190), S(66));
    lv_obj_align(tile, LV_ALIGN_CENTER, dx, dy);
    lv_obj_set_style_radius(tile, S(10), 0);
    lv_obj_set_style_bg_color(tile, lv_color_hex(C_TILE_BG), 0);
    lv_obj_set_style_bg_opa(tile, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(tile, lv_color_hex(C_TILE_LINE), 0);
    lv_obj_set_style_border_width(tile, S(2), 0);
    lv_obj_set_style_pad_all(tile, 0, 0);
    lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(tile, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *cap = lv_label_create(tile);
    lv_label_set_text(cap, caption);
    lv_obj_set_style_text_font(cap, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(cap, lv_color_hex(C_MUTED), 0);
    lv_obj_set_style_text_letter_space(cap, S(1), 0);
    lv_obj_align(cap, LV_ALIGN_TOP_MID, 0, S(5));

    lv_obj_t *val = lv_label_create(tile);
    lv_label_set_text(val, "--");
    lv_obj_set_style_text_font(val, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(val, lv_color_white(), 0);
    lv_obj_align(val, LV_ALIGN_BOTTOM_MID, 0, S(-4));

    return val;   // caller keeps the value label
}

// Point on the dial circle, in dial-container coordinates.
static void polar(lv_coord_t centre, float radius, float deg,
                  lv_coord_t *out_x, lv_coord_t *out_y)
{
    float rad = deg * (float)M_PI / 180.0f;
    *out_x = (lv_coord_t)(centre + radius * cosf(rad));
    *out_y = (lv_coord_t)(centre + radius * sinf(rad));
}

static void set_value(lv_obj_t *label, float v, const char *fmt)
{
    if (!label) return;

    char buf[16];
    if (isnan(v)) {
        lv_label_set_text(label, "--");
        return;
    }
    snprintf(buf, sizeof(buf), fmt, v);

    // Skip the relayout when nothing actually changed.
    if (strcmp(lv_label_get_text(label), buf) != 0) {
        lv_label_set_text(label, buf);
    }
}

// -------------------------------------------------------------- dial parts --

static void build_dial(lv_obj_t *parent)
{
    lv_obj_t *dial = lv_obj_create(parent);
    lv_obj_set_size(dial, S(DIAL_BOX), S(DIAL_BOX));
    lv_obj_center(dial);
    lv_obj_set_style_bg_opa(dial, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(dial, 0, 0);
    lv_obj_set_style_pad_all(dial, 0, 0);
    lv_obj_clear_flag(dial, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(dial, LV_OBJ_FLAG_CLICKABLE);

    const lv_coord_t c = S(DIAL_BOX) / 2;

    // Ticks
    for (int i = 0; i < TICK_COUNT; i++) {
        float deg = DIAL_START_ANGLE + (DIAL_SWEEP * i) / (TICK_COUNT - 1);
        int   rpm = (TACH_MAX_RPM * i) / (TICK_COUNT - 1);
        bool  major = (i % TICKS_PER_MAJOR) == 0;
        bool  hot   = rpm >= TACH_REDLINE_RPM;

        float r_in = major ? S(R_TICK_MAJOR_IN) : S(R_TICK_MINOR_IN);

        polar(c, r_in,             deg, &s_tick_pts[i][0].x, &s_tick_pts[i][0].y);
        polar(c, (float)S(R_TICK_OUTER), deg, &s_tick_pts[i][1].x, &s_tick_pts[i][1].y);

        lv_obj_t *ln = lv_line_create(dial);
        lv_line_set_points(ln, s_tick_pts[i], 2);
        lv_obj_set_pos(ln, 0, 0);
        lv_obj_set_style_line_width(ln, major ? S(5) : S(3), 0);
        lv_obj_set_style_line_color(
            ln, lv_color_hex(hot ? C_RED : (major ? C_TICK_MAJOR : C_TICK_MINOR)), 0);
        lv_obj_set_style_line_rounded(ln, false, 0);
    }

    // Numbers (0..TACH_MAJORS), drawn as labels so the redline one can be red.
    for (int n = 0; n <= TACH_MAJORS; n++) {
        float deg = DIAL_START_ANGLE + (DIAL_SWEEP * n) / TACH_MAJORS;
        lv_coord_t x, y;
        polar(0, (float)S(R_NUMBERS), deg, &x, &y);

        char txt[4];
        snprintf(txt, sizeof(txt), "%d", n);

        bool hot = (n * 1000) >= TACH_REDLINE_RPM;
        make_label(dial, txt, &lv_font_montserrat_28,
                   hot ? C_RED : C_TICK_MAJOR, x, y);
    }
}

// ------------------------------------------------------------------ screen --

void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(ui_Screen1, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(ui_Screen1, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(ui_Screen1, 0, 0);

    // Bezel ring + face
    make_disc(ui_Screen1, S(720), C_BEZEL);
    make_disc(ui_Screen1, S(636), C_FACE);

    // ---- RPM sweep, sitting just outside the tick track ----
    ui_rpm_arc = lv_arc_create(ui_Screen1);
    lv_obj_set_size(ui_rpm_arc, S(2 * R_RPM_ARC + 12), S(2 * R_RPM_ARC + 12));
    lv_obj_center(ui_rpm_arc);
    lv_arc_set_rotation(ui_rpm_arc, 0);
    lv_arc_set_bg_angles(ui_rpm_arc, (uint16_t)DIAL_START_ANGLE,
                         (uint16_t)fmodf(DIAL_START_ANGLE + DIAL_SWEEP, 360.0f));
    lv_arc_set_range(ui_rpm_arc, 0, TACH_MAX_RPM);
    lv_arc_set_value(ui_rpm_arc, 0);

    lv_obj_set_style_arc_width(ui_rpm_arc, S(12), LV_PART_MAIN);
    lv_obj_set_style_arc_color(ui_rpm_arc, lv_color_hex(C_TRACK), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_rpm_arc, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(ui_rpm_arc, 0, LV_PART_MAIN);

    lv_obj_set_style_arc_width(ui_rpm_arc, S(12), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(ui_rpm_arc, lv_color_hex(0x28FF00), LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(ui_rpm_arc, false, LV_PART_INDICATOR);

    lv_obj_remove_style(ui_rpm_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(ui_rpm_arc, LV_OBJ_FLAG_CLICKABLE);

    // ---- Tick marks and numbers ----
    build_dial(ui_Screen1);

    // ---- Fuel band across the bottom: E on the right, F on the left ----
    // Static reserve marker first so the live bar draws over it.
    lv_obj_t *reserve = lv_arc_create(ui_Screen1);
    lv_obj_set_size(reserve, S(2 * R_FUEL_ARC + 20), S(2 * R_FUEL_ARC + 20));
    lv_obj_center(reserve);
    lv_arc_set_rotation(reserve, 0);
    lv_arc_set_bg_angles(reserve, 55, 69);
    lv_obj_set_style_arc_width(reserve, S(20), LV_PART_MAIN);
    lv_obj_set_style_arc_color(reserve, lv_color_hex(C_RED), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(reserve, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(reserve, 0, LV_PART_MAIN);
    lv_obj_remove_style(reserve, NULL, LV_PART_INDICATOR);
    lv_obj_remove_style(reserve, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(reserve, LV_OBJ_FLAG_CLICKABLE);

    ui_fuel_arc = lv_arc_create(ui_Screen1);
    lv_obj_set_size(ui_fuel_arc, S(2 * R_FUEL_ARC + 20), S(2 * R_FUEL_ARC + 20));
    lv_obj_center(ui_fuel_arc);
    lv_arc_set_rotation(ui_fuel_arc, 0);
    lv_arc_set_bg_angles(ui_fuel_arc, 55, 125);
    lv_arc_set_range(ui_fuel_arc, 0, 100);
    lv_arc_set_value(ui_fuel_arc, 0);

    lv_obj_set_style_arc_width(ui_fuel_arc, S(20), LV_PART_MAIN);
    lv_obj_set_style_arc_color(ui_fuel_arc, lv_color_hex(C_TRACK), LV_PART_MAIN);
    lv_obj_set_style_arc_opa(ui_fuel_arc, LV_OPA_60, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_fuel_arc, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(ui_fuel_arc, 0, LV_PART_MAIN);

    lv_obj_set_style_arc_width(ui_fuel_arc, S(20), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(ui_fuel_arc, lv_color_hex(0xE8E8E8), LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(ui_fuel_arc, false, LV_PART_INDICATOR);

    lv_obj_remove_style(ui_fuel_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(ui_fuel_arc, LV_OBJ_FLAG_CLICKABLE);

    // F / E, sitting just outside the band
    {
        lv_coord_t fx, fy, ex, ey;
        polar(0, (float)S(R_FUEL_LABELS), 125.0f, &fx, &fy);
        polar(0, (float)S(R_FUEL_LABELS),  55.0f, &ex, &ey);
        make_label(ui_Screen1, "F", &lv_font_montserrat_22, 0xFFFFFF, fx, fy);
        make_label(ui_Screen1, "E", &lv_font_montserrat_22, 0xFFFFFF, ex, ey);
    }

    // ---- Speed ----
    make_label(ui_Screen1, "mph", &lv_font_montserrat_20, C_MUTED, 0, S(-142));

    ui_label_mph_value = lv_label_create(ui_Screen1);
    lv_label_set_text(ui_label_mph_value, "--");
    lv_obj_set_style_text_font(ui_label_mph_value, &lv_font_montserrat_44, 0);
    lv_obj_set_style_text_color(ui_label_mph_value, lv_color_white(), 0);
    lv_obj_align(ui_label_mph_value, LV_ALIGN_CENTER, 0, S(-78));

    // ---- Four sensor tiles ----
    ui_val_oil_psi  = make_tile(ui_Screen1, "OIL PSI",  S(-108), S(28));
    ui_val_water    = make_tile(ui_Screen1, "WATER",    S(108),  S(28));
    ui_val_oil_temp = make_tile(ui_Screen1, "OIL TEMP", S(-108), S(104));
    ui_val_trans    = make_tile(ui_Screen1, "TRANS",    S(108),  S(104));

    // ---- Mileage ----
    make_label(ui_Screen1, "MILEAGE", &lv_font_montserrat_14, C_MUTED, 0, S(169));

    lv_obj_t *odo_row = lv_obj_create(ui_Screen1);
    lv_obj_set_size(odo_row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(odo_row, LV_ALIGN_CENTER, 0, S(201));
    lv_obj_set_style_bg_opa(odo_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(odo_row, 0, 0);
    lv_obj_set_style_pad_all(odo_row, 0, 0);
    lv_obj_set_flex_flow(odo_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(odo_row, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(odo_row, S(6), 0);
    lv_obj_clear_flag(odo_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(odo_row, LV_OBJ_FLAG_CLICKABLE);

    ui_label_odometer_value = lv_label_create(odo_row);
    lv_label_set_text(ui_label_odometer_value, "0.0");
    lv_obj_set_style_text_font(ui_label_odometer_value, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(ui_label_odometer_value, lv_color_white(), 0);

    lv_obj_t *mi = lv_label_create(odo_row);
    lv_label_set_text(mi, "mi");
    lv_obj_set_style_text_font(mi, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(mi, lv_color_hex(C_MUTED), 0);
}

void ui_Screen1_screen_destroy(void)
{
    if (ui_Screen1) lv_obj_del(ui_Screen1);

    ui_Screen1 = NULL;
    ui_rpm_arc = NULL;
    ui_fuel_arc = NULL;
    ui_label_mph_value = NULL;
    ui_label_odometer_value = NULL;
    ui_val_oil_psi = NULL;
    ui_val_water = NULL;
    ui_val_oil_temp = NULL;
    ui_val_trans = NULL;
}

// ----------------------------------------------------------------- setters --

void ui_dash_set_rpm(int rpm)
{
    if (!ui_rpm_arc) return;

    if (rpm < 0) rpm = 0;
    if (rpm > TACH_MAX_RPM) rpm = TACH_MAX_RPM;

    // Green below 5000, amber to the redline, red past it.
    static uint32_t last = 0;
    uint32_t want = (rpm >= TACH_REDLINE_RPM) ? C_RED
                  : (rpm >= 5000)             ? 0xFFA000
                                              : 0x28FF00;
    if (want != last) {
        lv_obj_set_style_arc_color(ui_rpm_arc, lv_color_hex(want), LV_PART_INDICATOR);
        last = want;
    }

    lv_arc_set_value(ui_rpm_arc, rpm);
}

void ui_dash_set_speed_mph(float mph)
{
    if (!ui_label_mph_value) return;

    char buf[8];
    if (isnan(mph) || mph < 0.0f) {
        lv_label_set_text(ui_label_mph_value, "--");
        return;
    }
    snprintf(buf, sizeof(buf), "%d", (int)(mph + 0.5f));
    if (strcmp(lv_label_get_text(ui_label_mph_value), buf) != 0) {
        lv_label_set_text(ui_label_mph_value, buf);
    }
}

void ui_dash_set_fuel_pct(float pct)
{
    if (!ui_fuel_arc) return;

    if (isnan(pct)) pct = 0.0f;
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    lv_arc_set_value(ui_fuel_arc, (int16_t)pct);
}

void ui_dash_set_oil_psi(float psi)   { set_value(ui_val_oil_psi,  psi,  "%.0f"); }
void ui_dash_set_water_f(float degf)  { set_value(ui_val_water,    degf, "%.0f"); }
void ui_dash_set_oil_temp_f(float f)  { set_value(ui_val_oil_temp, f,    "%.0f"); }
void ui_dash_set_trans_f(float degf)  { set_value(ui_val_trans,    degf, "%.0f"); }

void ui_dash_set_mileage(double miles)
{
    if (!ui_label_odometer_value) return;

    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", miles);
    if (strcmp(lv_label_get_text(ui_label_odometer_value), buf) != 0) {
        lv_label_set_text(ui_label_odometer_value, buf);
    }
}
