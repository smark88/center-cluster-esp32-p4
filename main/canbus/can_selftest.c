#include "can_selftest.h"

#if CAN_SELFTEST_MODE

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"

static const char *TAG = "CAN_SELFTEST";

// Same pins the dash uses. Kept in step with canbus.c by hand; there is only
// one transceiver, so a mismatch here would test the wrong thing.
#define CAN_TX GPIO_NUM_5
#define CAN_RX GPIO_NUM_4

// Arbitrary -- nothing else is on the bus to care.
#define SELFTEST_ID 0x7FF

static void report_failure(void)
{
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) {
        ESP_LOGE(TAG, "could not read controller status");
        return;
    }

    const char *state = "unknown";
    switch (st.state) {
        case TWAI_STATE_STOPPED:     state = "stopped";     break;
        case TWAI_STATE_RUNNING:     state = "running";     break;
        case TWAI_STATE_BUS_OFF:     state = "bus-off";     break;
        case TWAI_STATE_RECOVERING:  state = "recovering";  break;
    }

    ESP_LOGE(TAG, "state=%s tx_err=%d rx_err=%d tx_failed=%d bus_err=%d",
             state,
             (int)st.tx_error_counter, (int)st.rx_error_counter,
             (int)st.tx_failed_count,  (int)st.bus_error_count);

    // A climbing tx error counter means the bits went out but never came
    // back the way they were sent -- that is the bit monitor faulting, and it
    // is what a missing CRX wire or an unpowered transceiver looks like.
    if (st.tx_error_counter > 0)
        ESP_LOGE(TAG, "bits left GPIO%d but did not return on GPIO%d -- check "
                      "3.3V, GND, and that CTX/CRX are not swapped",
                 (int)CAN_TX, (int)CAN_RX);
}

void can_selftest_task(void *arg)
{
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NO_ACK);

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    ESP_LOGI(TAG, "loopback check on TX=GPIO%d RX=GPIO%d at %d bps",
             (int)CAN_TX, (int)CAN_RX, CAN_SELFTEST_BITRATE);

    uint32_t pass = 0, fail = 0;

    while (1) {
        twai_message_t tx = {0};
        tx.identifier       = SELFTEST_ID;
        tx.data_length_code = 8;
        tx.self             = 1;   // ask the controller to receive its own frame
        for (int i = 0; i < 8; i++)
            tx.data[i] = (uint8_t)(0xA5 ^ i);

        // Drain anything stale so a hit below is definitely this frame.
        twai_message_t rx;
        while (twai_receive(&rx, 0) == ESP_OK)
            ;

        esp_err_t sent = twai_transmit(&tx, pdMS_TO_TICKS(200));
        if (sent != ESP_OK) {
            fail++;
            ESP_LOGE(TAG, "FAIL -- transmit did not queue (%s)  pass=%lu fail=%lu",
                     esp_err_to_name(sent), pass, fail);
            report_failure();
            vTaskDelay(pdMS_TO_TICKS(CAN_SELFTEST_PERIOD_MS));
            continue;
        }

        esp_err_t got = twai_receive(&rx, pdMS_TO_TICKS(200));
        if (got != ESP_OK) {
            fail++;
            ESP_LOGE(TAG, "FAIL -- frame never came back on GPIO%d  pass=%lu fail=%lu",
                     (int)CAN_RX, pass, fail);
            report_failure();
            vTaskDelay(pdMS_TO_TICKS(CAN_SELFTEST_PERIOD_MS));
            continue;
        }

        bool match = (rx.identifier == SELFTEST_ID) &&
                     (rx.data_length_code == 8);
        for (int i = 0; match && i < 8; i++)
            match = (rx.data[i] == (uint8_t)(0xA5 ^ i));

        if (match) {
            pass++;
            ESP_LOGI(TAG, "PASS -- id 0x%03X returned intact, transceiver is "
                          "powered and CTX/CRX are correct  pass=%lu fail=%lu",
                     (unsigned)rx.identifier, pass, fail);
        } else {
            fail++;
            ESP_LOGE(TAG, "FAIL -- came back corrupted: id 0x%03X dlc %d  "
                          "pass=%lu fail=%lu",
                     (unsigned)rx.identifier, (int)rx.data_length_code,
                     pass, fail);
            report_failure();
        }

        vTaskDelay(pdMS_TO_TICKS(CAN_SELFTEST_PERIOD_MS));
    }
}

#endif // CAN_SELFTEST_MODE
