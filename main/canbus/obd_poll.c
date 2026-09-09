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
    uint16_t    pid;        // one byte for mode 01, two for mode 22
    uint8_t     nbytes;     // data bytes that make up the value, 1 or 2
    uint16_t    period_ms;  // how often to ask; matched to how fast it moves
    float       scale;
    float       offset;
    obd_dest_t  dest;
    float      *target;     // used when dest is DEST_FIELD
    const char *name;
    // Everything below is optional and left zero for the common case: standard
    // mode 01, functional request to 0x7DF, answered by the ECM at 0x7E8. Only
    // the GM enhanced entries fill them in.
    uint8_t     mode;       // 0 or 0x01 = mode 01, 0x22 = GM enhanced
    uint16_t    req_id;     // 0 = OBD_REQ_ID
    uint16_t    resp_id;    // 0 = OBD_ECU_ID
} obd_pid_t;

// Scales fold the unit conversion in, same contract as the protocol jsons:
// everything downstream is imperial.
// Periods are matched to how fast each thing physically moves. Boost changes
// in tens of milliseconds, so asking at a flat 300ms would alias real
// transients away -- no amount of display smoothing recovers that. Barometric
// pressure barely changes at all, so it can idle in the background.
// This gauge shows oil pressure, water, oil temp and trans temp, plus the
// fuel arc, the centre RPM readout and the odometer. Only five of those exist
// as standard mode 01 PIDs; see the note under the table for the two that do
// not.
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

    // Engine RPM, ((A*256)+B)/4. Drives the centre readout and the outer arc,
    // which is the fastest-moving thing on the gauge, so it gets the shortest
    // period in the table.
    { 0x0C, 2,  100, 0.25f, 0.0f, DEST_FIELD, NULL, "rpm" },

    // Vehicle speed, A km/h. Folded to mph here like every protocol json does,
    // because everything downstream is imperial. The odometer integrates this
    // against elapsed time, so it needs to arrive steadily rather than fast.
    { 0x0D, 1,  250, 0.621371f, 0.0f, DEST_FIELD, NULL, "speed" },

    // ---- polled for gauge two, which listens rather than asking ----------
    // This gauge does not display any of these. It polls them because it is
    // the bridge publisher: one node asking the ECU once is cheaper than two
    // nodes asking separately, and three of the PIDs were duplicated between
    // them anyway. See canbus/can_bridge.h.

    // Manifold absolute pressure, A kPa. Kept raw for the boost maths.
    { 0x0B, 1,  100, 1.0f,          0.0f,   DEST_MAP,   NULL, "MAP" },

    // Commanded equivalence ratio, ((A*256)+B) * 2 / 65535 lambda.
    // Stoichiometric petrol is 14.7:1, so lambda * 14.7 gives AFR.
    { 0x44, 2,  200, (2.0f/65535.0f)*14.7f, 0.0f, DEST_FIELD, NULL, "AFR" },

    // Fuel rail gauge pressure, ((A*256)+B) * 10 kPa -> psi.
    { 0x23, 2,  200, 10.0f*KPA_TO_PSI, 0.0f, DEST_FIELD, NULL, "fuel psi" },

    // Intake air temp, A - 40 degC. To degF: A * 1.8 - 40.
    { 0x0F, 1,  600, 1.8f,          -40.0f, DEST_FIELD, NULL, "IAT" },

    // Barometric pressure, A kPa. Also raw.
    { 0x33, 1, 5000, 1.0f,          0.0f,   DEST_BARO,  NULL, "baro" },

    // Ethanol content, A * 100 / 255 percent. This is standard J1979, not a
    // GM enhanced PID, so no mode 22 is needed -- a flex-fuel car answers it
    // and anything else returns a negative response and the tile stays "--".
    // Blend only changes when fuel is added, so it can idle in the background.
    { 0x52, 1, 5000, 100.0f/255.0f, 0.0f,   DEST_FIELD, NULL, "ethanol" },

    // Throttle position, A * 100 / 255 percent. Not on any tile -- carried
    // because knock and gear both only mean something under throttle, and
    // having it costs one slot that was spare anyway.
    { 0x11, 1,  200, 100.0f/255.0f, 0.0f,   DEST_FIELD, NULL, "throttle" },

    // ---- GM enhanced, mode 22 ---------------------------------------------
    // Neither of these exists as a standard mode 01 PID, which is why both
    // tiles sat at "--". HP Tuners reads them off this same bus, so the data
    // is there; it is just behind manufacturer-proprietary PIDs that have to
    // be asked for by physical address rather than functionally.
    //
    // PID and formula from eigger/espcomponents ble_elm327 presets.py, which
    // carries 49 GM mode 22 definitions with per-PID maths -- the first source
    // found that publishes formulas rather than just PID numbers. Its
    // gm_trans_temp agrees with every other source, which is some evidence the
    // rest is not invented.
    //
    // It lists two oil pressure PIDs and they disagree about which ECU family
    // they suit:
    //     gm_oil_pressure      115C   (A * 0.65) - 17.5
    //     gm_oil_pressure_alt  1470    A * 3.985
    // 115C is the unsuffixed one, so it goes in first. If it answers 0x7F, try
    // 1470 with a scale of 3.985 and no offset.
    //
    // 115C is corroborated independently by dchad/OBD-Monitor, whose notes
    // record it as a hand-built custom PID because it is not in the extended
    // set. That source writes the equation as (A * .065) - 17.5, a tenth of
    // the scale used here, but it cannot be right for a single byte: 255 *
    // 0.065 - 17.5 is negative, so the gauge could never read above zero. Its
    // own note mentions a steady 42 psi observed, which needs A = 92 at 0.65
    // and A = 915 at 0.065 -- out of range for one byte. The decimal point is
    // in the wrong place there, not here.
    //
    // A hot LT4 idles near 25 psi and shows 60-70 at 3000 rpm. HP Tuners
    // already reads this on the car, so put the two side by side -- that is a
    // direct check of both PID and formula in one go.
    { 0x115C, 1,  300, 0.65f, -17.5f, DEST_FIELD, NULL, "oil psi",
      0x22, OBD_ECM_REQ, OBD_ECU_ID },

    // Transmission fluid temp, A - 40 degC, from the TCM rather than the
    // engine. Widely reported for the 8L90E and the one number that actually
    // kills these boxes.
    { 0x1940, 1,  600, 1.8f, -40.0f, DEST_FIELD, NULL, "trans temp",
      0x22, OBD_TCM_REQ, OBD_TCM_ID },

    // Engaged gear straight from the transmission. This makes the RPM/speed
    // ratio estimate on gauge two a fallback rather than the primary source --
    // the TCM cannot be wrong about tyre diameter.
    { 0x199A, 1,  200, 1.0f, 0.0f, DEST_FIELD, NULL, "gear",
      0x22, OBD_TCM_REQ, OBD_TCM_ID },

    // PRNDL. The enum this returns is NOT known to match the broadcast one
    // gm.json decodes, which is 0 Park, 1 Neutral, 2 Drive, 3 Reverse. If the
    // letter is wrong, that mapping in can_mapping_task is what to change.
    { 0x1951, 1,  200, 1.0f, 0.0f, DEST_FIELD, NULL, "prndl",
      0x22, OBD_TCM_REQ, OBD_TCM_ID },

    // Knock retard, degrees of timing pulled. On a supercharged motor this is
    // the number worth a tile: it moves before anything else does when the
    // charge temp, fuel or timing is wrong, and it reads a clean zero when
    // nothing is happening.
    { 0x11A6, 1,  200, 0.0878906f, 0.0f, DEST_FIELD, NULL, "knock",
      0x22, OBD_ECM_REQ, OBD_ECU_ID },
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

// Boost is manifold pressure above ambient, so it needs both readings. Seeded
// at sea level: barometric is polled every 5s and until the first reply lands
// this keeps boost plausible rather than reading a full atmosphere of it.
static float s_map_kpa  = 101.3f;
static float s_baro_kpa = 101.3f;

static void bind_targets(void)
{
    for (int i = 0; i < PID_COUNT; i++) {
        switch (s_pids[i].pid) {
            case 0x05: s_targets[i] = (float *)&can_data.coolant_temp;   break;
            case 0x5C: s_targets[i] = (float *)&can_data.oil_temp;       break;
            case 0x2F: s_targets[i] = (float *)&can_data.fuel_level;     break;
            case 0x0C: s_targets[i] = (float *)&can_data.rpm;            break;
            case 0x0D: s_targets[i] = (float *)&can_data.speed;          break;
            case 0x44: s_targets[i] = (float *)&can_data.air_fuel_ratio; break;
            case 0x23: s_targets[i] = (float *)&can_data.fuel_pressure;  break;
            case 0x0F: s_targets[i] = (float *)&can_data.air_temp;       break;
            case 0x52: s_targets[i] = (float *)&can_data.fuel_comp;      break;
            case 0x11: s_targets[i] = (float *)&can_data.throttle_pct;   break;
            case 0x115C: s_targets[i] = (float *)&can_data.oil_pressure; break;
            case 0x1940: s_targets[i] = (float *)&can_data.trans_temp;   break;
            case 0x199A: s_targets[i] = (float *)&can_data.gear_num;     break;
            case 0x1951: s_targets[i] = (float *)&can_data.gear_sel;     break;
            case 0x11A6: s_targets[i] = (float *)&can_data.knock_retard; break;
            default:   s_targets[i] = NULL;                              break;
        }
    }
}

static void send_request(const obd_pid_t *p)
{
    uint8_t mode = p->mode ? p->mode : 0x01;

    twai_message_t msg = {0};
    msg.identifier = p->req_id ? p->req_id : OBD_REQ_ID;
    msg.data_length_code = 8;

    if (mode == 0x22) {
        // Enhanced PIDs are two bytes, so the frame carries one more byte
        // than a mode 01 request and the length reflects that.
        msg.data[0] = 0x03;
        msg.data[1] = 0x22;
        msg.data[2] = (uint8_t)(p->pid >> 8);
        msg.data[3] = (uint8_t)(p->pid & 0xFF);
        msg.data[4] = 0xAA;      // padding, ignored
        msg.data[5] = 0xAA;
        msg.data[6] = 0xAA;
        msg.data[7] = 0xAA;
    } else {
        msg.data[0] = 0x02;      // 2 more bytes follow
        msg.data[1] = 0x01;      // mode 01, current data
        msg.data[2] = (uint8_t)p->pid;
        msg.data[3] = 0xAA;      // padding, ignored by the ECU
        msg.data[4] = 0xAA;
        msg.data[5] = 0xAA;
        msg.data[6] = 0xAA;
        msg.data[7] = 0xAA;
    }

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

    // [len][service][pid...][A][B]...
    // 0x41 answers mode 01 and echoes a one byte pid; 0x62 answers mode 22 and
    // echoes two. Anything else is a negative response (0x7F when the module
    // does not support the pid) or not for us.
    uint16_t pid;
    int      first;                   // index of data byte A

    if (data[1] == 0x41) {
        pid   = data[2];
        first = 3;
    } else if (data[1] == 0x62) {
        if (dlc < 4)
            return true;
        pid   = ((uint16_t)data[2] << 8) | data[3];
        first = 4;
    } else {
        return true;
    }

    for (int i = 0; i < PID_COUNT; i++) {
        if (s_pids[i].pid != pid)
            continue;

        // A one byte mode 01 pid and a two byte mode 22 pid could in principle
        // collide numerically, so the service has to agree as well.
        uint8_t mode = s_pids[i].mode ? s_pids[i].mode : 0x01;
        if ((data[1] == 0x41) != (mode == 0x01))
            continue;

        // Requests go out functionally on 0x7DF unless the entry says
        // otherwise, so every OBD-capable module answers and more than one
        // will claim the same pid. On an FR-S a second module reports coolant
        // as 0, which the A*1.8-40 scaling turns into a clean -40 degF, so the
        // tile flips between the real reading and -40 depending on which reply
        // landed last. Only the module that owns the value is authoritative --
        // the ECM for engine data, the TCM for transmission data.
        uint16_t expect = s_pids[i].resp_id ? s_pids[i].resp_id : OBD_ECU_ID;
        if (id != expect)
            return true;              // right pid, wrong module

        uint32_t raw;
        if (s_pids[i].nbytes == 2) {
            if (dlc < first + 2) return true;
            raw = ((uint32_t)data[first] << 8) | data[first + 1];
        } else {
            if (dlc < first + 1) return true;
            raw = data[first];
        }

        float value = raw * s_pids[i].scale + s_pids[i].offset;

        switch (s_pids[i].dest) {
            case DEST_MAP:
                s_map_kpa = value;
                can_data.boost = (s_map_kpa - s_baro_kpa) * KPA_TO_PSI;
                break;
            case DEST_BARO:
                s_baro_kpa = value;
                break;
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
                send_request(&s_pids[i]);
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
