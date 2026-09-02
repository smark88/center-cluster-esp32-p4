// Gauge two -- speedometer cluster.
//
// Everything here is drawn with LVGL vectors instead of a background bitmap,
// so the layout follows the panel resolution: all geometry is authored against
// a 720x720 reference and scaled through S() at runtime.

#include "../ui.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>   // abs

// ---------------------------------------------------------------- geometry --

// Layout reference resolution. Everything below is in these units.
#define UI_BASE_RES 720
#define S(x)  ((lv_coord_t)(((int32_t)(x) * (int32_t)LV_HOR_RES) / UI_BASE_RES))

// Dial sweep: 0 sits at the lower left, numbers climb over the top, full
// scale sits at the lower right. LVGL angles are clockwise with 0 at 3
// o'clock, and the sweep is centred on 270 (straight up).
//
// 200 degrees rather than 240: at 240 the end numbers landed at y=+124, which
// put them behind the sensor tiles. Pulling the ends up to 170/10 degrees
// moves them to y=+43 and clears the tile edge by 11px.
#define DIAL_START_ANGLE  170.0f
#define DIAL_SWEEP        200.0f

#define DIAL_MAJORS       (DIAL_MAX_MPH / DIAL_MPH_PER_MAJOR)
#define TICKS_PER_MAJOR   4                       // a minor tick every 5 mph
#define TICK_COUNT        (DIAL_MAJORS * TICKS_PER_MAJOR + 1)

// Radii (720-space). The numbers sit further in than on gauge one, and the
// major ticks are shorter, because "180" is three digits wide and would
// otherwise run into the tick marks.
#define R_TICK_OUTER      300
#define R_TICK_MAJOR_IN   280
#define R_TICK_MINOR_IN   290
#define R_NUMBERS         248
#define R_SPEED_ARC       316
#define SPEED_ARC_W       18

#define DIAL_BOX          660   // container that holds ticks + numbers

// Sensor tiles. Width is the constrained axis -- the dial numbers sit near
// x = +/-220, so the tiles must stop short of that.
#define TILE_W            200
#define TILE_H            78
#define TILE_DX           107
// 90px row pitch, matching gauge one, which leaves a 12px gap between the
// rows. The mockup's 28/104 was authored for 66px tall tiles; at the current
// 78px the two rows overlapped by 2px.
#define TILE_ROW1_DY      28
#define TILE_ROW2_DY      118

// A flashing tile thickens both ways: the border grows inward and a ring is
// drawn outward, giving one continuous band. Padding is set to the complement
// of the border width so the content inset never changes and nothing reflows.
#define TILE_BORDER_W       2
#define TILE_WARN_BORDER_W  4
#define TILE_WARN_RING_W    4
#define TILE_PAD_REST       (TILE_WARN_BORDER_W - TILE_BORDER_W)

// PRNDM selector along the bottom.
#define GEAR_BOX_W        72
#define GEAR_BOX_H        52
#define GEAR_PITCH        88
#define GEAR_DY           216

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
#define C_GEAR_ON_BG 0x1C2026

// ----------------------------------------------------------------- objects --

lv_obj_t *ui_Screen1 = NULL;
lv_obj_t *ui_speed_arc = NULL;
lv_obj_t *ui_label_mph_value = NULL;
lv_obj_t *ui_label_gear_value = NULL;
lv_obj_t *ui_val_iat = NULL;
lv_obj_t *ui_val_fuel_psi = NULL;
lv_obj_t *ui_val_afr = NULL;
lv_obj_t *ui_val_boost = NULL;

// lv_line keeps a pointer to the caller's points, so they must outlive it.
static lv_point_t s_tick_pts[TICK_COUNT][2];

// ------------------------------------------------------------- tile alarms --

enum { TILE_IAT = 0, TILE_FUEL_PSI, TILE_AFR, TILE_BOOST, TILE_COUNT };

typedef struct {
    lv_obj_t *tile;     // for the border and ring
    lv_obj_t *value;    // for the number
    bool      alarm;    // currently out of range
    bool      lit;      // red is currently applied
} tile_t;

static tile_t s_tiles[TILE_COUNT];
static int    s_last_rpm = 0;

// ----------------------------------------------------------- gear selector --

static const char GEAR_LETTERS[] = "PRNDM";
#define GEAR_COUNT 5

static lv_obj_t *s_gear_box[GEAR_COUNT];
static lv_obj_t *s_gear_lbl[GEAR_COUNT];
static int       s_gear_sel = -1;

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

// A bordered tile with a muted caption and a large value underneath.
static lv_obj_t *make_tile(lv_obj_t *parent, const char *caption,
                           lv_coord_t dx, lv_coord_t dy, int slot)
{
    lv_obj_t *tile = lv_obj_create(parent);
    lv_obj_set_size(tile, S(TILE_W), S(TILE_H));
    lv_obj_align(tile, LV_ALIGN_CENTER, dx, dy);
    lv_obj_set_style_radius(tile, S(10), 0);
    lv_obj_set_style_bg_color(tile, lv_color_hex(C_TILE_BG), 0);
    lv_obj_set_style_bg_opa(tile, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(tile, lv_color_hex(C_TILE_LINE), 0);
    lv_obj_set_style_border_width(tile, S(TILE_BORDER_W), 0);

    // Warning ring, invisible until the tile alarms.
    lv_obj_set_style_outline_width(tile, S(TILE_WARN_RING_W), 0);
    lv_obj_set_style_outline_pad(tile, 0, 0);
    lv_obj_set_style_outline_color(tile, lv_color_hex(C_RED), 0);
    lv_obj_set_style_outline_opa(tile, LV_OPA_TRANSP, 0);

    lv_obj_set_style_pad_all(tile, S(TILE_PAD_REST), 0);
    lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(tile, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *cap = lv_label_create(tile);
    lv_label_set_text(cap, caption);
    lv_obj_set_style_text_font(cap, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(cap, lv_color_hex(C_MUTED), 0);
    lv_obj_set_style_text_letter_space(cap, S(1), 0);
    lv_obj_align(cap, LV_ALIGN_TOP_MID, 0, S(6));

    lv_obj_t *val = lv_label_create(tile);
    lv_label_set_text(val, "--");
    lv_obj_set_style_text_font(val, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(val, lv_color_white(), 0);
    lv_obj_align(val, LV_ALIGN_BOTTOM_MID, 0, S(-6));

    s_tiles[slot].tile  = tile;
    s_tiles[slot].value = val;
    s_tiles[slot].alarm = false;
    s_tiles[slot].lit   = false;

    return val;   // caller keeps the value label
}

// Blinks whichever tiles are out of range. Only touches a tile when its
// appearance actually has to change, so an all-clear dash costs nothing.
static void tile_flash_cb(lv_timer_t *t)
{
    LV_UNUSED(t);
    static bool phase = false;
    phase = !phase;

    for (int i = 0; i < TILE_COUNT; i++) {
        tile_t *tl = &s_tiles[i];
        if (!tl->value) continue;

        bool want_red = tl->alarm && phase;
        if (want_red == tl->lit) continue;

        lv_obj_set_style_text_color(tl->value,
            want_red ? lv_color_hex(C_RED) : lv_color_white(), 0);
        lv_obj_set_style_border_color(tl->tile,
            want_red ? lv_color_hex(C_RED) : lv_color_hex(C_TILE_LINE), 0);
        lv_obj_set_style_outline_opa(tl->tile,
            want_red ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
        // Grow inward too. Padding takes up the slack so the caption and
        // value stay put.
        lv_obj_set_style_border_width(tl->tile,
            S(want_red ? TILE_WARN_BORDER_W : TILE_BORDER_W), 0);
        lv_obj_set_style_pad_all(tl->tile,
            S(want_red ? 0 : TILE_PAD_REST), 0);
        tl->lit = want_red;
    }
}

// Latch the alarm state; the blink timer does the drawing.
static void set_alarm(int slot, bool on)
{
    if (slot < 0 || slot >= TILE_COUNT) return;
    if (s_tiles[slot].alarm == on) return;

    s_tiles[slot].alarm = on;

    if (!on && s_tiles[slot].lit) {          // clear immediately
        lv_obj_set_style_text_color(s_tiles[slot].value, lv_color_white(), 0);
        lv_obj_set_style_border_color(s_tiles[slot].tile,
                                      lv_color_hex(C_TILE_LINE), 0);
        lv_obj_set_style_outline_opa(s_tiles[slot].tile, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(s_tiles[slot].tile, S(TILE_BORDER_W), 0);
        lv_obj_set_style_pad_all(s_tiles[slot].tile, S(TILE_PAD_REST), 0);
        s_tiles[slot].lit = false;
    }
}

// Point on the dial circle, in dial-container coordinates.
static void polar(lv_coord_t centre, float radius, float deg,
                  lv_coord_t *out_x, lv_coord_t *out_y)
{
    float rad = deg * (float)M_PI / 180.0f;
    *out_x = (lv_coord_t)(centre + radius * cosf(rad));
    *out_y = (lv_coord_t)(centre + radius * sinf(rad));
}

// The OBD PIDs land at 0.5-10Hz while this screen redraws at 30, so a raw
// value would visibly step. Each reading is eased toward its latest sample at
// the display rate, with a time constant matched to that channel's poll
// period -- roughly half the sample interval, which glides between samples
// without adding perceptible lag.
//
// This is presentation only. Smoothing cannot recover a transient that was
// never sampled, which is why the poll periods are tuned first.
#define EASE_BOOST     0.25f   // polled 100ms  -> tau ~130ms
#define EASE_AFR       0.15f   // polled 200ms  -> tau ~220ms
#define EASE_FUEL_PSI  0.15f   // polled 200ms
#define EASE_IAT       0.05f   // polled 600ms  -> tau ~670ms
#define EASE_FUEL_LVL  0.01f   // polled 2s, and noisy from tank slosh

static float ease(float *state, float target, float alpha)
{
    if (isnan(target)) { *state = NAN; return NAN; }
    if (isnan(*state)) { *state = target; return target; }   // first sample
    *state += alpha * (target - *state);
    return *state;
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
        bool  major = (i % TICKS_PER_MAJOR) == 0;

        float r_in = major ? S(R_TICK_MAJOR_IN) : S(R_TICK_MINOR_IN);

        polar(c, r_in,                   deg, &s_tick_pts[i][0].x, &s_tick_pts[i][0].y);
        polar(c, (float)S(R_TICK_OUTER), deg, &s_tick_pts[i][1].x, &s_tick_pts[i][1].y);

        lv_obj_t *ln = lv_line_create(dial);
        lv_line_set_points(ln, s_tick_pts[i], 2);
        lv_obj_set_pos(ln, 0, 0);
        lv_obj_set_style_line_width(ln, major ? S(5) : S(3), 0);
        lv_obj_set_style_line_color(
            ln, lv_color_hex(major ? C_TICK_MAJOR : C_TICK_MINOR), 0);
        lv_obj_set_style_line_rounded(ln, false, 0);
    }

    // Numbers, 0 .. DIAL_MAX_MPH in steps of DIAL_MPH_PER_MAJOR.
    for (int n = 0; n <= DIAL_MAJORS; n++) {
        float deg = DIAL_START_ANGLE + (DIAL_SWEEP * n) / DIAL_MAJORS;
        lv_coord_t x, y;
        polar(0, (float)S(R_NUMBERS), deg, &x, &y);

        char txt[6];
        snprintf(txt, sizeof(txt), "%d", n * DIAL_MPH_PER_MAJOR);

        make_label(dial, txt, &lv_font_montserrat_28, C_TICK_MAJOR, x, y);
    }
}

// ----------------------------------------------------------- gear selector --

static void build_gear_selector(lv_obj_t *parent)
{
    const lv_coord_t first = -(GEAR_PITCH * (GEAR_COUNT - 1)) / 2;

    for (int i = 0; i < GEAR_COUNT; i++) {
        lv_obj_t *box = lv_obj_create(parent);
        lv_obj_set_size(box, S(GEAR_BOX_W), S(GEAR_BOX_H));
        lv_obj_align(box, LV_ALIGN_CENTER, S(first + i * GEAR_PITCH), S(GEAR_DY));
        lv_obj_set_style_radius(box, S(8), 0);
        lv_obj_set_style_bg_color(box, lv_color_hex(C_TILE_BG), 0);
        lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(box, lv_color_hex(C_TILE_LINE), 0);
        lv_obj_set_style_border_width(box, S(TILE_BORDER_W), 0);
        lv_obj_set_style_pad_all(box, 0, 0);
        lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(box, LV_OBJ_FLAG_CLICKABLE);

        char txt[2] = { GEAR_LETTERS[i], 0 };
        lv_obj_t *l = lv_label_create(box);
        lv_label_set_text(l, txt);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_28, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(C_MUTED), 0);
        lv_obj_center(l);

        s_gear_box[i] = box;
        s_gear_lbl[i] = l;
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

    // ---- Speed sweep, sitting just outside the tick track ----
    ui_speed_arc = lv_arc_create(ui_Screen1);
    lv_obj_set_size(ui_speed_arc, S(2 * R_SPEED_ARC + SPEED_ARC_W),
                                  S(2 * R_SPEED_ARC + SPEED_ARC_W));
    lv_obj_center(ui_speed_arc);
    lv_arc_set_rotation(ui_speed_arc, 0);
    lv_arc_set_bg_angles(ui_speed_arc, (uint16_t)DIAL_START_ANGLE,
                         (uint16_t)fmodf(DIAL_START_ANGLE + DIAL_SWEEP, 360.0f));
    lv_arc_set_range(ui_speed_arc, 0, DIAL_MAX_MPH);
    lv_arc_set_value(ui_speed_arc, 0);

    lv_obj_set_style_arc_width(ui_speed_arc, S(SPEED_ARC_W), LV_PART_MAIN);
    lv_obj_set_style_arc_color(ui_speed_arc, lv_color_hex(C_TRACK), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_speed_arc, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(ui_speed_arc, 0, LV_PART_MAIN);

    lv_obj_set_style_arc_width(ui_speed_arc, S(SPEED_ARC_W), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(ui_speed_arc, lv_color_hex(0x28FF00), LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(ui_speed_arc, false, LV_PART_INDICATOR);

    lv_obj_remove_style(ui_speed_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(ui_speed_arc, LV_OBJ_FLAG_CLICKABLE);

    // ---- Tick marks and numbers ----
    build_dial(ui_Screen1);

    // ---- Which gear the box is in, up top ----
    make_label(ui_Screen1, "GEAR", &lv_font_montserrat_14, C_MUTED, 0, S(-205));

    ui_label_gear_value = lv_label_create(ui_Screen1);
    lv_label_set_text(ui_label_gear_value, "--");
    lv_obj_set_style_text_font(ui_label_gear_value, &lv_font_montserrat_44, 0);
    lv_obj_set_style_text_color(ui_label_gear_value, lv_color_white(), 0);
    lv_obj_align(ui_label_gear_value, LV_ALIGN_CENTER, 0, S(-168));

    // ---- Speed readout ----
    make_label(ui_Screen1, "mph", &lv_font_montserrat_20, C_MUTED, 0, S(-119));

    ui_label_mph_value = lv_label_create(ui_Screen1);
    lv_label_set_text(ui_label_mph_value, "--");
    lv_obj_set_style_text_font(ui_label_mph_value, &ui_font_rpm_96, 0);
    lv_obj_set_style_text_color(ui_label_mph_value, lv_color_white(), 0);
    lv_obj_align(ui_label_mph_value, LV_ALIGN_CENTER, 0, S(-62));

    // ---- Four sensor tiles ----
    ui_val_iat      = make_tile(ui_Screen1, "IAT",      S(-TILE_DX), S(TILE_ROW1_DY), TILE_IAT);
    ui_val_fuel_psi = make_tile(ui_Screen1, "FUEL PSI",  S(TILE_DX), S(TILE_ROW1_DY), TILE_FUEL_PSI);
    ui_val_afr      = make_tile(ui_Screen1, "AFR",      S(-TILE_DX), S(TILE_ROW2_DY), TILE_AFR);
    ui_val_boost    = make_tile(ui_Screen1, "BOOST",     S(TILE_DX), S(TILE_ROW2_DY), TILE_BOOST);

    lv_timer_create(tile_flash_cb, WARN_FLASH_MS, NULL);

    // ---- PRNDM ----
    build_gear_selector(ui_Screen1);
}

void ui_Screen1_screen_destroy(void)
{
    if (ui_Screen1) lv_obj_del(ui_Screen1);

    ui_Screen1 = NULL;
    ui_speed_arc = NULL;
    ui_label_mph_value = NULL;
    ui_label_gear_value = NULL;
    ui_val_iat = NULL;
    ui_val_fuel_psi = NULL;
    ui_val_afr = NULL;
    ui_val_boost = NULL;

    for (int i = 0; i < TILE_COUNT; i++) {
        s_tiles[i].tile  = NULL;
        s_tiles[i].value = NULL;
        s_tiles[i].alarm = false;
        s_tiles[i].lit   = false;
    }
    for (int i = 0; i < GEAR_COUNT; i++) {
        s_gear_box[i] = NULL;
        s_gear_lbl[i] = NULL;
    }
    s_gear_sel = -1;
}

// ----------------------------------------------------------------- setters --

void ui_dash_set_speed_mph(float mph)
{
    int v = isnan(mph) ? 0 : (int)(mph + 0.5f);
    if (v < 0) v = 0;
    if (v > DIAL_MAX_MPH) v = DIAL_MAX_MPH;

    // Only move the arc when the value actually changed. One mph is ~1.3
    // degrees of sweep here, so every whole mph is worth drawing.
    if (ui_speed_arc) {
        static int last_drawn = -1;
        if (v != last_drawn) {
            lv_arc_set_value(ui_speed_arc, v);
            last_drawn = v;
        }
    }

    if (ui_label_mph_value) {
        char buf[8];
        if (isnan(mph)) snprintf(buf, sizeof(buf), "--");
        else            snprintf(buf, sizeof(buf), "%d", v);
        if (strcmp(lv_label_get_text(ui_label_mph_value), buf) != 0)
            lv_label_set_text(ui_label_mph_value, buf);
    }
}

void ui_dash_set_rpm(int rpm)
{
    // Not displayed on this gauge; it only gates the AFR warning.
    s_last_rpm = rpm;
}

void ui_dash_set_iat_f(float degf)
{
    static float st = NAN;
    float v = ease(&st, degf, EASE_IAT);
    set_value(ui_val_iat, v, "%.0f");
    set_alarm(TILE_IAT, !isnan(v) && v > WARN_IAT_MAX);
}

void ui_dash_set_fuel_psi(float psi)
{
    static float st = NAN;
    float v = ease(&st, psi, EASE_FUEL_PSI);
    set_value(ui_val_fuel_psi, v, "%.0f");
    set_alarm(TILE_FUEL_PSI, !isnan(v) && v < WARN_FUEL_PSI_MIN);
}

void ui_dash_set_afr(float afr)
{
    static float st = NAN;
    afr = ease(&st, afr, EASE_AFR);
    set_value(ui_val_afr, afr, "%.1f");
    // Lean only matters under load. Cruise and overrun are lean by design, so
    // gate the warning on the engine actually pulling.
    set_alarm(TILE_AFR,
              !isnan(afr) && afr > WARN_AFR_MAX &&
              s_last_rpm >= WARN_AFR_MIN_RPM);
}

void ui_dash_set_boost_psi(float psi)
{
    static float st = NAN;
    set_value(ui_val_boost, ease(&st, psi, EASE_BOOST), "%.1f");
}

void ui_dash_set_drive_gear(int gear)
{
    if (!ui_label_gear_value) return;

    char buf[4];
    if (gear == GEAR_REVERSE)          snprintf(buf, sizeof(buf), "R");
    else if (gear >= 1 && gear <= 8)   snprintf(buf, sizeof(buf), "%d", gear);
    else                               snprintf(buf, sizeof(buf), "--");

    if (strcmp(lv_label_get_text(ui_label_gear_value), buf) != 0)
        lv_label_set_text(ui_label_gear_value, buf);
}

void ui_dash_set_gear(char gear)
{
    int sel = -1;
    for (int i = 0; i < GEAR_COUNT; i++) {
        if (GEAR_LETTERS[i] == gear) { sel = i; break; }
    }
    if (sel == s_gear_sel) return;
    s_gear_sel = sel;

    for (int i = 0; i < GEAR_COUNT; i++) {
        if (!s_gear_box[i]) continue;
        bool on = (i == sel);

        lv_obj_set_style_text_color(s_gear_lbl[i],
            on ? lv_color_white() : lv_color_hex(C_MUTED), 0);
        lv_obj_set_style_border_color(s_gear_box[i],
            on ? lv_color_white() : lv_color_hex(C_TILE_LINE), 0);
        lv_obj_set_style_bg_color(s_gear_box[i],
            on ? lv_color_hex(C_GEAR_ON_BG) : lv_color_hex(C_TILE_BG), 0);
    }
}
