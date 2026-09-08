#include "can_bridge.h"

#include <math.h>
#include <limits.h>
#include "canbus.h"

#if CAN_BRIDGE_MODE != CAN_BRIDGE_OFF
#include "driver/twai.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "CAN_BRIDGE";
#endif

// Sentinel for "the publisher has no reading for this". Kept out of the
// representable range of a real value so it can never be produced by clamping.
#define BRIDGE_NA  INT16_MIN

// One entry per slot, in wire order: three frames of four.
typedef struct {
    float *field;     // where it lives in can_data
    float  scale;     // multiply by this to get the int16
} bridge_slot_t;

// Built at runtime because can_data is volatile and cannot be used in a static
// initialiser. Order here IS the wire format -- see the layout note in the
// header. Changing it changes the protocol, so both gauges must be reflashed.
static bridge_slot_t s_slots[CAN_BRIDGE_FRAMES * 4];
static bool          s_bound = false;

static void bind_slots(void)
{
    if (s_bound)
        return;

    volatile can_dash_data_t *d = &can_data;

    // 0x7F0
    s_slots[0]  = (bridge_slot_t){ (float *)&d->rpm,            1.0f  };
    s_slots[1]  = (bridge_slot_t){ (float *)&d->speed,          10.0f };
    s_slots[2]  = (bridge_slot_t){ (float *)&d->coolant_temp,   10.0f };
    s_slots[3]  = (bridge_slot_t){ (float *)&d->oil_temp,       10.0f };
    // 0x7F1
    s_slots[4]  = (bridge_slot_t){ (float *)&d->fuel_level,     10.0f };
    s_slots[5]  = (bridge_slot_t){ (float *)&d->fuel_pressure,  10.0f };
    s_slots[6]  = (bridge_slot_t){ (float *)&d->air_fuel_ratio, 10.0f };
    s_slots[7]  = (bridge_slot_t){ (float *)&d->air_temp,       10.0f };
    // 0x7F2
    s_slots[8]  = (bridge_slot_t){ (float *)&d->boost,          10.0f };
    s_slots[9]  = (bridge_slot_t){ (float *)&d->oil_pressure,   10.0f };
    s_slots[10] = (bridge_slot_t){ (float *)&d->trans_temp,     10.0f };
    s_slots[11] = (bridge_slot_t){ (float *)&d->fuel_comp,      10.0f };

    s_bound = true;
}

#if CAN_BRIDGE_MODE == CAN_BRIDGE_PUBLISH

static void put_slot(uint8_t *p, const bridge_slot_t *s)
{
    int32_t q;

    float v = *s->field;
    if (isnan(v)) {
        q = BRIDGE_NA;
    } else {
        q = (int32_t)lrintf(v * s->scale);
        // Clamp inside the range, leaving BRIDGE_NA unreachable so a pegged
        // reading never reads back as "no data".
        if (q > INT16_MAX)     q = INT16_MAX;
        if (q < INT16_MIN + 1) q = INT16_MIN + 1;
    }

    p[0] = (uint8_t)(q & 0xFF);
    p[1] = (uint8_t)((q >> 8) & 0xFF);
}

// Set if another node is already transmitting on one of our ids. Publishing
// stops for good at that point -- two nodes sending different payloads under
// one id do not interleave, they collide mid-frame and produce error frames,
// and this bus has ABS on it.
static volatile bool s_collision = false;

void can_bridge_publish(void)
{
    static int64_t next_ms = 0;

    if (s_collision)
        return;

    int64_t now = esp_timer_get_time() / 1000;
    if (now < next_ms)
        return;
    next_ms = now + CAN_BRIDGE_PERIOD_MS;

    bind_slots();

    for (int f = 0; f < CAN_BRIDGE_FRAMES; f++) {
        twai_message_t msg = {0};
        msg.identifier       = CAN_BRIDGE_BASE_ID + f;
        msg.data_length_code = 8;

        for (int i = 0; i < 4; i++)
            put_slot(&msg.data[i * 2], &s_slots[f * 4 + i]);

        // Never block the gauge on a full queue. A dropped frame is one
        // 50ms refresh missed, which the subscriber cannot even display.
        twai_transmit(&msg, 0);
    }
}

#else

void can_bridge_publish(void) { }

#endif // PUBLISH


#if CAN_BRIDGE_MODE == CAN_BRIDGE_PUBLISH

// The controller does not self-receive in TWAI_MODE_NORMAL, so a frame
// arriving on one of our ids can only have come from another node -- something
// on this bus already owns that id. Stop transmitting rather than fight it.
//
// This exists because the ids cannot be proven free from a datasheet. They are
// chosen to be low priority and outside the diagnostic block, which makes a
// clash unlikely, not impossible. On an unfamiliar car this is the difference
// between finding out from a log line and finding out from the brake module.
bool can_bridge_handle_frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    (void)data; (void)dlc;

    if (id < CAN_BRIDGE_BASE_ID || id >= CAN_BRIDGE_BASE_ID + CAN_BRIDGE_FRAMES)
        return false;

    if (!s_collision) {
        s_collision = true;
        ESP_LOGE(TAG, "id 0x%03X is already in use on this bus -- bridge "
                      "publishing disabled", (unsigned)id);
        ESP_LOGE(TAG, "  move CAN_BRIDGE_BASE_ID somewhere clear, then check "
                      "with CAN_SCAN_MODE that the new range is unused");
    }

    return true;
}

#elif CAN_BRIDGE_MODE == CAN_BRIDGE_SUBSCRIBE

bool can_bridge_handle_frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if (id < CAN_BRIDGE_BASE_ID || id >= CAN_BRIDGE_BASE_ID + CAN_BRIDGE_FRAMES)
        return false;
    if (dlc < 8)
        return true;                 // ours, but malformed

    bind_slots();

    int f = (int)(id - CAN_BRIDGE_BASE_ID);

    for (int i = 0; i < 4; i++) {
        const uint8_t *p = &data[i * 2];
        int16_t q = (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));

        bridge_slot_t *s = &s_slots[f * 4 + i];

        // NAN rather than zero, so a reading the publisher does not have keeps
        // rendering as "--" instead of appearing as a real value of nothing.
        *s->field = (q == BRIDGE_NA) ? NAN : (float)q / s->scale;
    }

    return true;
}

#else

bool can_bridge_handle_frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    (void)id; (void)data; (void)dlc;
    return false;
}

#endif // handler
