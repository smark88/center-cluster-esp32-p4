#include "boot_screen.h"
#include "ui.h"

lv_obj_t *boot_screen= NULL;
static lv_obj_t *boot_logo;

/* ---------- Callbacks ---------- */

static void logo_opa_cb(void *obj, int32_t v)
{
    lv_obj_set_style_opa(obj, v, 0);
}

/* ---------- Create Screen ---------- */
void boot_screen_create(void)
{
    boot_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(boot_screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(boot_screen, LV_OPA_COVER, 0);

    boot_logo = lv_img_create(boot_screen);
    lv_img_set_src(boot_logo, &corvette_c5r_63_720);
    lv_img_set_antialias(boot_logo, true);

    /* Scale the logo up so it completely fills the round display (cover fit),
       independent of the panel resolution (720x720 4in, 800x800 3.4in, ...). */
    lv_coord_t disp_w = lv_disp_get_hor_res(NULL);
    lv_coord_t disp_h = lv_disp_get_ver_res(NULL);
    uint32_t zoom_w = ((uint32_t)disp_w * LV_IMG_ZOOM_NONE) / corvette_c5r_63_720.header.w;
    uint32_t zoom_h = ((uint32_t)disp_h * LV_IMG_ZOOM_NONE) / corvette_c5r_63_720.header.h;
    uint32_t zoom = (zoom_w > zoom_h) ? zoom_w : zoom_h;   /* max ratio => cover */
    lv_img_set_pivot(boot_logo, corvette_c5r_63_720.header.w / 2, corvette_c5r_63_720.header.h / 2);
    lv_img_set_zoom(boot_logo, (uint16_t)zoom);

    lv_obj_center(boot_logo);
    lv_obj_set_style_opa(boot_logo, LV_OPA_0, 0);

    // Glow
    lv_obj_set_style_shadow_color(
        boot_logo, lv_color_hex(0xFF1E1E), 0
    );
}

/* ---------- Start Animation ---------- */
void boot_start(void)
{
    lv_scr_load(boot_screen);

    lv_anim_t a;

    /* Smooth fade IN: transparent -> opaque */
    lv_anim_init(&a);
    lv_anim_set_var(&a, boot_logo);
    lv_anim_set_exec_cb(&a, logo_opa_cb);
    lv_anim_set_values(&a, LV_OPA_TRANSP, LV_OPA_COVER);
    lv_anim_set_time(&a, 600);
    lv_anim_set_delay(&a, 700);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_start(&a);
}

/* ---------- Dismiss: smooth fade OUT, then hand off to the main UI ---------- */
void boot_dismiss(lv_obj_t *next_screen)
{
    const uint32_t hold = 2200;   /* logo fully visible before it starts leaving */
    const uint32_t fade = 600;    /* logo fade-out duration                       */

    lv_anim_t a;

    /* Smooth fade OUT: opaque -> fully invisible */
    lv_anim_init(&a);
    lv_anim_set_var(&a, boot_logo);
    lv_anim_set_exec_cb(&a, logo_opa_cb);
    lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_TRANSP);
    lv_anim_set_time(&a, fade);
    lv_anim_set_delay(&a, hold);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_start(&a);

    /* Once the logo has faded away, cross-fade into the gauge screen */
    lv_scr_load_anim(next_screen, LV_SCR_LOAD_ANIM_FADE_ON, 500, hold + fade, false);
}

/* ---------- Exit ---------- */
void boot_finish(lv_obj_t *next_screen)
{
    // Animate fade from boot screen → main UI
    lv_scr_load_anim(next_screen, LV_SCR_LOAD_ANIM_FADE_ON, 2000, 3000, true);

    // Delete boot objects
    if (boot_screen) {
        lv_obj_del(boot_screen);
        boot_screen = NULL;
    }
}