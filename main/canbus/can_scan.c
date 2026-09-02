#include "can_scan.h"

#if CAN_SCAN_MODE

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_timer.h"
#include "esp_log.h"

// Same pins the normal driver uses.
#define CAN_TX GPIO_NUM_5
#define CAN_RX GPIO_NUM_4

#define MAX_IDS 160

static const char *TAG = "CAN_SCAN";

typedef struct {
    uint32_t id;
    bool     extd;
    uint8_t  dlc;
    uint8_t  now[8];
    uint8_t  lo[8];
    uint8_t  hi[8];
    uint32_t count;
} scan_entry_t;

static scan_entry_t s_ids[MAX_IDS];
static int s_id_count = 0;

static scan_entry_t *find_or_add(uint32_t id, bool extd)
{
    for (int i = 0; i < s_id_count; i++) {
        if (s_ids[i].id == id && s_ids[i].extd == extd)
            return &s_ids[i];
    }
    if (s_id_count >= MAX_IDS)
        return NULL;

    scan_entry_t *e = &s_ids[s_id_count++];
    memset(e, 0, sizeof(*e));
    e->id   = id;
    e->extd = extd;
    memset(e->lo, 0xFF, sizeof(e->lo));   // so the first sample sets both ends
    memset(e->hi, 0x00, sizeof(e->hi));
    return e;
}

static void dump_table(void)
{
    ESP_LOGI(TAG, "---- %d ids seen ----", s_id_count);
    ESP_LOGI(TAG, "%-12s %-5s %-6s %-24s %s",
             "ID", "type", "count", "current bytes", "bytes that MOVED (lo..hi)");

    for (int i = 0; i < s_id_count; i++) {
        scan_entry_t *e = &s_ids[i];

        char cur[26];
        int n = 0;
        for (int b = 0; b < e->dlc && b < 8; b++)
            n += snprintf(cur + n, sizeof(cur) - n, "%02X ", e->now[b]);
        cur[n > 0 ? n - 1 : 0] = 0;

        // Only bytes whose range opened up are interesting.
        char moved[96];
        int m = 0;
        moved[0] = 0;
        for (int b = 0; b < e->dlc && b < 8; b++) {
            if (e->hi[b] != e->lo[b] && m < (int)sizeof(moved) - 20) {
                m += snprintf(moved + m, sizeof(moved) - m,
                              "b%d:%02X..%02X ", b, e->lo[b], e->hi[b]);
            }
        }
        if (m == 0) snprintf(moved, sizeof(moved), "(static)");

        char idbuf[12];
        if (e->extd) snprintf(idbuf, sizeof(idbuf), "0x%08lX", (unsigned long)e->id);
        else         snprintf(idbuf, sizeof(idbuf), "0x%03lX",  (unsigned long)e->id);

        ESP_LOGI(TAG, "%-12s %-5s %-6lu %-24s %s",
                 idbuf, e->extd ? "ext" : "std",
                 (unsigned long)e->count, cur, moved);
    }
    ESP_LOGI(TAG, " ");
}

void can_scan_task(void *arg)
{
    // LISTEN_ONLY: never transmits, never ACKs. Safe on a live vehicle bus.
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_LISTEN_ONLY);
    g_config.rx_queue_len = 64;

    twai_timing_config_t t_config;
    switch (CAN_SCAN_BITRATE) {
        case 1000000: t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();   break;
        case 500000:  t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS(); break;
        case 250000:  t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS(); break;
        case 125000:  t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS(); break;
        case 100000:  t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_100KBITS(); break;
        default:      t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS(); break;
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    ESP_LOGI(TAG, "listening at %d bps (listen-only), dumping every %d ms",
             CAN_SCAN_BITRATE, CAN_SCAN_DUMP_MS);

    int64_t last_dump = esp_timer_get_time();
    twai_message_t msg;

    while (1) {
        if (twai_receive(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
            if (!msg.rtr) {
                scan_entry_t *e = find_or_add(msg.identifier, msg.extd);
                if (e) {
                    e->dlc = msg.data_length_code > 8 ? 8 : msg.data_length_code;
                    e->count++;
                    for (int b = 0; b < e->dlc; b++) {
                        uint8_t v = msg.data[b];
                        e->now[b] = v;
                        if (v < e->lo[b]) e->lo[b] = v;
                        if (v > e->hi[b]) e->hi[b] = v;
                    }
                }
            }
        }

        if ((esp_timer_get_time() - last_dump) / 1000 >= CAN_SCAN_DUMP_MS) {
            last_dump = esp_timer_get_time();
            dump_table();
        }
    }
}

#endif // CAN_SCAN_MODE
