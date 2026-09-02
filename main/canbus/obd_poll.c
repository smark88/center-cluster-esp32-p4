#include "obd_poll.h"

#if OBD_POLL_ENABLE

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "canbus.h"

static const char *TAG = "OBD";

#define KPA_TO_PSI 0.145038f

typedef enum {
    DEST_FIELD,      // straight into a can_data field
    DEST_MAP,        // stash for the boost calculation
    DEST_BARO,
} obd_dest_t;

typedef struct {
    uint8_t     pid;
    uint8_t     nbytes;     // data bytes that make up the value, 1 or 2
    uint16_t    period_ms;  // how often to ask; matched to how fast it moves
    float       scale;
    float       offset;
    obd_dest_t  dest;
    float      *target;     // used when dest is DEST_FIELD
    const char *name;
} obd_pid_t;

// Scales fold the unit conversion in, same contract as the protocol jsons:
// everything downstream is imperial.
// Periods are matched to how fast each thing physically moves. Boost changes
// in tens of milliseconds, so asking at a flat 300ms would alias real
// transients away -- no amount of display smoothing recovers that. Barometric
// pressure barely changes at all, so it can idle in the background.
// This gauge shows oil pressure, water, oil temp and trans temp, plus the
// fuel arc. Only three of those exist as standard mode 01 PIDs; see the note
// under the table for the two that do not.
static const obd_pid_t s_pids[] = {
    // Engine coolant temp, A - 40 degC. To degF: A * 1.8 - 40.
    { 0x05, 1,  600, 1.8f, -40.0f, DEST_FIELD, NULL, "coolant" },

    // Engine oil temp, same encoding. Not fitted to every car -- if the ECU
    // does not support it the tile simply stays at "--".
    { 0x5C, 1,  600, 1.8f, -40.0f, DEST_FIELD, NULL, "oil temp" },

    // Fuel tank level, A * 100 / 255, already a percentage. This is the only
    // source the fuel arc has in CAN mode: adc_task does not run there, so
    // without this the arc sits empty.
    { 0x2F, 1, 2000, 100.0f/255.0f, 0.0f, DEST_FIELD, NULL, "fuel level" },
};

// NOT AVAILABLE as standard mode 01, and so not polled here:
//   oil pressure  -> GM enhanced mode 22 PID 0x1470
//   trans temp    -> GM enhanced mode 22 PID 0x1940
// Mode 22 is manufacturer proprietary and varies by model year. Both are also
// on GMLAN in gm_lowspeed.json, which needs the single-wire transceiver.

#define PID_COUNT (sizeof(s_pids)/sizeof(s_pids[0]))

// Set up at runtime because can_data is volatile and cannot be used in a
// static initialiser.
static float *s_targets[PID_COUNT];

static int64_t s_due_ms[PID_COUNT];

static void bind_targets(void)
{
    for (int i = 0; i < PID_COUNT; i++) {
        switch (s_pids[i].pid) {
            case 0x05: s_targets[i] = (float *)&can_data.coolant_temp;   break;
            case 0x5C: s_targets[i] = (float *)&can_data.oil_temp;       break;
            case 0x2F: s_targets[i] = (float *)&can_data.fuel_level;     break;
            default:   s_targets[i] = NULL;                              break;
        }
    }
}

static void send_request(uint8_t pid)
{
    twai_message_t msg = {0};
    msg.identifier = OBD_REQ_ID;
    msg.data_length_code = 8;
    msg.data[0] = 0x02;      // 2 more bytes follow
    msg.data[1] = 0x01;      // mode 01, current data
    msg.data[2] = pid;
    msg.data[3] = 0xAA;      // padding, ignored by the ECU
    msg.data[4] = 0xAA;
    msg.data[5] = 0xAA;
    msg.data[6] = 0xAA;
    msg.data[7] = 0xAA;

    // Never block the task on a full queue; a dropped request just means this
    // PID is refreshed on the next lap.
    twai_transmit(&msg, 0);
}

bool obd_poll_handle_frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if (id < OBD_RESP_LO || id > OBD_RESP_HI)
        return false;
    if (dlc < 3)
        return true;                  // ours, but malformed

    // [len][0x41][pid][A][B]...   0x41 = positive reply to mode 01
    if (data[1] != 0x41)
        return true;

    uint8_t pid = data[2];

    for (int i = 0; i < PID_COUNT; i++) {
        if (s_pids[i].pid != pid)
            continue;

        uint32_t raw;
        if (s_pids[i].nbytes == 2) {
            if (dlc < 5) return true;
            raw = ((uint32_t)data[3] << 8) | data[4];
        } else {
            if (dlc < 4) return true;
            raw = data[3];
        }

        float value = raw * s_pids[i].scale + s_pids[i].offset;

        switch (s_pids[i].dest) {
            case DEST_MAP:
            case DEST_BARO:
                break;      // boost is on gauge two, not this one
            case DEST_FIELD:
                if (s_targets[i]) *s_targets[i] = value;
                break;
        }
        return true;
    }
    return true;      // an OBD reply, just not a PID we asked for
}

static void obd_poll_task(void *arg)
{
    bind_targets();
    ESP_LOGI(TAG, "polling %d PIDs (transmitting on 0x%03X)",
             (int)PID_COUNT, OBD_REQ_ID);

    while (1) {
        int64_t now = esp_timer_get_time() / 1000;

        // One request per tick at most, so two PIDs never collide on the bus.
        for (int i = 0; i < PID_COUNT; i++) {
            if (now >= s_due_ms[i]) {
                send_request(s_pids[i].pid);
                s_due_ms[i] = now + s_pids[i].period_ms;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(OBD_POLL_TICK_MS));
    }
}

void obd_poll_start(void)
{
    xTaskCreatePinnedToCore(obd_poll_task, "obd_poll", 3072, NULL, 8, NULL, 0);
}

#else  // OBD_POLL_ENABLE

#include <stdbool.h>
void obd_poll_start(void) {}
bool obd_poll_handle_frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    (void)id; (void)data; (void)dlc;
    return false;
}

#endif // OBD_POLL_ENABLE
