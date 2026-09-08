#include "can_probe.h"

#if CAN_PROBE_MODE

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

static const char *TAG = "CAN_PROBE";

// Same pins the dash uses.
#define CAN_TX GPIO_NUM_5
#define CAN_RX GPIO_NUM_4

#define OBD_REQ_ID   0x7DF
#define OBD_RESP_LO  0x7E8
#define OBD_RESP_HI  0x7EF

// Unique ids worth remembering; a busy bus has far more, the count still adds
// up correctly, we just stop listing new ones.
#define MAX_IDS 32

static uint32_t s_ids[MAX_IDS];
static int      s_id_count;

static uint32_t s_frames;
static uint32_t s_replies;
static bool     s_got_bitmap;
static uint8_t  s_bitmap[4];

static void note_id(uint32_t id)
{
    for (int i = 0; i < s_id_count; i++)
        if (s_ids[i] == id)
            return;
    if (s_id_count < MAX_IDS)
        s_ids[s_id_count++] = id;
}

static const char *state_name(twai_state_t st)
{
    switch (st) {
        case TWAI_STATE_STOPPED:    return "stopped";
        case TWAI_STATE_RUNNING:    return "running";
        case TWAI_STATE_BUS_OFF:    return "bus-off";
        case TWAI_STATE_RECOVERING: return "recovering";
    }
    return "unknown";
}

// Mode 01 PID 0x00 answers with a 32 bit mask: bit 31 is PID 0x01, bit 0 is
// PID 0x20. Spell out the three the dash actually asks for.
static void report_bitmap(void)
{
    uint32_t mask = ((uint32_t)s_bitmap[0] << 24) | ((uint32_t)s_bitmap[1] << 16) |
                    ((uint32_t)s_bitmap[2] << 8)  |  (uint32_t)s_bitmap[3];

    ESP_LOGI(TAG, "  supported PID mask 0x01-0x20: %08lX", (unsigned long)mask);

    // PID n is bit (32 - n).
    bool coolant = mask & (1u << (32 - 0x05));
    bool speed   = mask & (1u << (32 - 0x0D));
    bool rpm     = mask & (1u << (32 - 0x0C));
    ESP_LOGI(TAG, "  0x05 coolant: %s   0x0C rpm: %s   0x0D speed: %s",
             coolant ? "yes" : "NO", rpm ? "yes" : "NO", speed ? "yes" : "NO");
    ESP_LOGI(TAG, "  (0x2F fuel and 0x5C oil temp live in the 0x21-0x40 block, "
                  "not this one)");
}

static void report(void)
{
    twai_status_info_t st;
    twai_get_status_info(&st);

    ESP_LOGI(TAG, "---- frames=%lu  replies=%lu  unique ids=%d ----",
             (unsigned long)s_frames, (unsigned long)s_replies, s_id_count);
    ESP_LOGI(TAG, "  state=%s tx_err=%d rx_err=%d tx_failed=%d bus_err=%d",
             state_name(st.state),
             (int)st.tx_error_counter, (int)st.rx_error_counter,
             (int)st.tx_failed_count,  (int)st.bus_error_count);

    if (s_id_count) {
        char line[MAX_IDS * 6 + 1];
        int n = 0;
        for (int i = 0; i < s_id_count && n < (int)sizeof(line) - 6; i++)
            n += snprintf(line + n, sizeof(line) - n, "%03lX ",
                          (unsigned long)s_ids[i]);
        ESP_LOGI(TAG, "  ids: %s", line);
    }

    if (s_got_bitmap) {
        report_bitmap();
    } else if (s_frames == 0 && st.bus_error_count == 0 &&
               st.state == TWAI_STATE_RUNNING) {
        ESP_LOGW(TAG, "  wired ok, bus silent -- key off, or the gateway "
                      "passes no traffic to this port");
    } else if (s_frames == 0) {
        ESP_LOGE(TAG, "  nothing heard and the controller is erroring -- check "
                      "CANH on OBD pin 6, CANL on pin 14, and that the bus "
                      "really is %d bps", CAN_PROBE_BITRATE);
    } else {
        ESP_LOGW(TAG, "  bus is readable but nothing answered a request -- "
                      "gateway blocking diagnostics is the usual cause");
    }
}

// Before the driver claims the pin, read CRX as a plain input. On a live bus
// it toggles every couple of microseconds; a stuck level says the bits are
// not reaching the transceiver's receiver at all. This is the multimeter
// check done in software, and it separates "no signal on the wire" from
// "signal is there but we cannot frame it".
static void check_rx_pin_activity(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CAN_RX,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    int      last    = gpio_get_level(CAN_RX);
    uint32_t edges   = 0, samples = 0, high = 0;
    int64_t  stop    = esp_timer_get_time() + 100000;   // 100 ms

    while (esp_timer_get_time() < stop) {
        int lvl = gpio_get_level(CAN_RX);
        samples++;
        if (lvl) high++;
        if (lvl != last) { edges++; last = lvl; }
    }

    ESP_LOGI(TAG, "CRX pin (GPIO%d): %lu edges in 100ms over %lu samples, "
                  "%lu%% high",
             (int)CAN_RX, (unsigned long)edges, (unsigned long)samples,
             (unsigned long)(samples ? high * 100 / samples : 0));

    if (edges == 0 && high == samples)
        ESP_LOGE(TAG, "  STUCK RECESSIVE -- no bus traffic is reaching the "
                      "transceiver. CANH/CANL are not connected to a live "
                      "pair. Nothing past this point can work.");
    else if (edges == 0 && high == 0)
        ESP_LOGE(TAG, "  STUCK DOMINANT -- CANH/CANL shorted together, shorted "
                      "to ground, or the transceiver is faulted.");
    else
        ESP_LOGI(TAG, "  ACTIVITY PRESENT -- bits are arriving at the chip, so "
                      "the wiring is good and the problem is framing/bitrate.");
}

// Static DC test of the transceiver, done before any CAN framing exists to
// confuse it. CTX is the driver's input and CRX the receiver's output, so
// holding CTX low must drive the pair dominant and come straight back as a
// low on CRX. If CRX does not follow CTX, the chip is not doing its job --
// unpowered, in standby with Rs pulled high, or not actually wired to these
// two pins. This needs no bus and no other node.
static void check_transceiver_dc(void)
{
    gpio_config_t out = {
        .pin_bit_mask = 1ULL << CAN_TX,
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config_t in = {
        .pin_bit_mask = 1ULL << CAN_RX,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);
    gpio_config(&in);

    gpio_set_level(CAN_TX, 1);            // recessive
    esp_rom_delay_us(500);
    int rx_recessive = gpio_get_level(CAN_RX);

    gpio_set_level(CAN_TX, 0);            // dominant
    esp_rom_delay_us(500);
    int rx_dominant = gpio_get_level(CAN_RX);

    gpio_set_level(CAN_TX, 1);            // leave the bus alone again
    esp_rom_delay_us(500);

    ESP_LOGI(TAG, "transceiver DC test: CTX=1 -> CRX=%d   CTX=0 -> CRX=%d",
             rx_recessive, rx_dominant);

    if (rx_recessive == 1 && rx_dominant == 0) {
        ESP_LOGI(TAG, "  TRANSCEIVER OK -- driver and receiver both respond, "
                      "and CTX/CRX are on the pins we think they are");
    } else if (rx_recessive == 1 && rx_dominant == 1) {
        ESP_LOGE(TAG, "  DRIVER NOT DRIVING -- CRX ignored a dominant CTX. "
                      "Chip unpowered, Rs pulled high (standby), or CTX/CRX "
                      "not landing on the chip.");
    } else if (rx_recessive == 0 && rx_dominant == 0) {
        ESP_LOGE(TAG, "  CRX STUCK LOW -- CANH/CANL shorted, or the receiver "
                      "output is held down.");
    } else {
        ESP_LOGE(TAG, "  CRX INVERTED -- follows CTX backwards, CANH/CANL are "
                      "reversed somewhere in the path.");
    }
}

// Is CRX genuinely driven by the transceiver, or is that pin simply floating?
// The DC test above cannot tell: CAN_RX and CAN_TX are adjacent header pins,
// and a floating CMOS input capacitively follows its switching neighbour and
// holds that level for milliseconds, which looks exactly like a receiver
// doing its job. An internal pull resistor separates them -- a real push-pull
// receiver output drives milliamps and wins against the ~45k pull, a floating
// pin loses to it every time.
static void check_crx_driven(void)
{
    gpio_config_t out = {
        .pin_bit_mask = 1ULL << CAN_TX,
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);

    // Recessive in -> the receiver should be holding CRX HIGH. Fight it with a
    // pulldown.
    gpio_set_level(CAN_TX, 1);
    gpio_config_t pd = {
        .pin_bit_mask = 1ULL << CAN_RX,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&pd);
    esp_rom_delay_us(2000);
    int with_pulldown = gpio_get_level(CAN_RX);

    // Dominant in -> the receiver should be pulling CRX LOW. Fight it with a
    // pullup.
    gpio_set_level(CAN_TX, 0);
    gpio_config_t pu = {
        .pin_bit_mask = 1ULL << CAN_RX,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&pu);
    esp_rom_delay_us(2000);
    int with_pullup = gpio_get_level(CAN_RX);

    gpio_set_level(CAN_TX, 1);

    ESP_LOGI(TAG, "CRX drive test: pulldown+recessive -> %d (want 1)   "
                  "pullup+dominant -> %d (want 0)", with_pulldown, with_pullup);

    if (with_pulldown == 1 && with_pullup == 0) {
        ESP_LOGI(TAG, "  CRX IS DRIVEN -- the receiver output really is on "
                      "GPIO%d and beats the internal pull both ways",
                 (int)CAN_RX);
    } else if (with_pulldown == 0 && with_pullup == 1) {
        ESP_LOGE(TAG, "  CRX IS FLOATING -- GPIO%d follows the internal pull, "
                      "so nothing is driving it. The transceiver's receiver "
                      "output is NOT on this pin. Most likely CTX and CRX are "
                      "swapped at the transceiver: check the board's silkscreen "
                      "order, it is commonly 3V3, GND, CTX, CRX -- TX before RX.",
                 (int)CAN_RX);
    } else {
        ESP_LOGW(TAG, "  INCONCLUSIVE -- weak or intermittent drive on GPIO%d",
                 (int)CAN_RX);
    }
}

#if CAN_PROBE_SLOW_DRIVE
// Holds the bus pins at a steady level for seconds at a time so a handheld
// meter can follow them. Never returns.
static void slow_drive_loop(void)
{
    gpio_config_t out = {
        .pin_bit_mask = 1ULL << CAN_TX,
        .mode         = GPIO_MODE_OUTPUT,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);

    ESP_LOGW(TAG, "SLOW DRIVE -- unplug from the car, then meter CANH and CANL");
    ESP_LOGW(TAG, "  measure at the SCREW TERMINALS, black probe on GND");
    ESP_LOGW(TAG, "  recessive: CANH and CANL both sit near 2.3 V");
    ESP_LOGW(TAG, "  dominant:  CANH rises toward 3 V, CANL falls toward 1 V");
    ESP_LOGW(TAG, "  if neither moves, the terminals do NOT reach the chip");

    while (1) {
        gpio_set_level(CAN_TX, 1);
        ESP_LOGI(TAG, "RECESSIVE  -- expect CANH ~2.3 V, CANL ~2.3 V");
        vTaskDelay(pdMS_TO_TICKS(CAN_PROBE_SLOW_MS));

        gpio_set_level(CAN_TX, 0);
        ESP_LOGI(TAG, "DOMINANT   -- expect CANH ~3.0 V, CANL ~1.0 V");
        vTaskDelay(pdMS_TO_TICKS(CAN_PROBE_SLOW_MS));
    }
}
#endif

void can_probe_task(void *arg)
{
#if CAN_PROBE_SLOW_DRIVE
    slow_drive_loop();      // never returns
#endif
    check_crx_driven();
    check_transceiver_dc();
    check_rx_pin_activity();

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 64;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    ESP_LOGI(TAG, "probing OBD link: TX=GPIO%d RX=GPIO%d at %d bps",
             (int)CAN_TX, (int)CAN_RX, CAN_PROBE_BITRATE);

    int64_t next_req = 0, next_report = 0;

    while (1) {
        int64_t now = esp_timer_get_time() / 1000;

        if (now >= next_req) {
            next_req = now + CAN_PROBE_REQUEST_MS;

            twai_message_t req = {0};
            req.identifier       = OBD_REQ_ID;
            req.data_length_code = 8;
            req.data[0] = 0x02;
            req.data[1] = 0x01;   // mode 01
            req.data[2] = 0x00;   // PID 00, supported PIDs
            for (int i = 3; i < 8; i++)
                req.data[i] = 0xAA;

            esp_err_t e = twai_transmit(&req, 0);
            if (e != ESP_OK)
                ESP_LOGW(TAG, "request did not queue: %s", esp_err_to_name(e));
        }

        twai_message_t msg;
        while (twai_receive(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
            if (msg.rtr)
                continue;

            s_frames++;
            if (!msg.extd)
                note_id(msg.identifier);

#if CAN_PROBE_VERBOSE
            ESP_LOGI(TAG, "rx %s %03lX [%d] %02X %02X %02X %02X %02X %02X %02X %02X",
                     msg.extd ? "ext" : "std", (unsigned long)msg.identifier,
                     msg.data_length_code,
                     msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                     msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
#endif

            if (msg.identifier >= OBD_RESP_LO && msg.identifier <= OBD_RESP_HI) {
                s_replies++;
                // [len][0x41][pid][A][B][C][D]
                if (msg.data_length_code >= 6 &&
                    msg.data[1] == 0x41 && msg.data[2] == 0x00) {
                    memcpy(s_bitmap, &msg.data[3], 4);
                    s_got_bitmap = true;
                }
            }
        }

        if (now >= next_report) {
            next_report = now + CAN_PROBE_REPORT_MS;
            report();
        }
    }
}

#endif // CAN_PROBE_MODE
