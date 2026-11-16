#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#define BLE_REPORT_QUEUE_LENGTH 96
#define BLE_CSV_LINE_MAX_LEN 160

static const char *TAG = "ble_monitor";

typedef struct {
    uint64_t timestamp_us;
    int8_t rssi;
    uint8_t address_type;
    uint8_t address[6];
    uint8_t event_type;
    uint8_t data_length;
} ble_report_t;

static QueueHandle_t report_queue;
static uint32_t dropped_reports;
static portMUX_TYPE dropped_reports_mux = portMUX_INITIALIZER_UNLOCKED;

static int ble_gap_event(struct ble_gap_event *event, void *arg);

static void start_passive_scan(void)
{
    uint8_t own_address_type;
    int rc = ble_hs_id_infer_auto(0, &own_address_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Could not determine BLE address type: %d", rc);
        return;
    }

    struct ble_gap_disc_params params = {0};
    params.passive = 1;
    params.filter_duplicates = 0;

    rc = ble_gap_disc(own_address_type, BLE_HS_FOREVER, &params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Could not start passive BLE scan: %d", rc);
    }
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        ble_report_t report = {
            .timestamp_us = (uint64_t)esp_timer_get_time(),
            .rssi = event->disc.rssi,
            .address_type = event->disc.addr.type,
            .event_type = event->disc.event_type,
            .data_length = event->disc.length_data,
        };
        memcpy(report.address, event->disc.addr.val, sizeof(report.address));

        if (xQueueSend(report_queue, &report, 0) != pdTRUE) {
            portENTER_CRITICAL(&dropped_reports_mux);
            ++dropped_reports;
            portEXIT_CRITICAL(&dropped_reports_mux);
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGW(TAG, "BLE scan ended (reason %d); restarting", event->disc_complete.reason);
        start_passive_scan();
        return 0;
    default:
        return 0;
    }
}

static void ble_host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_sync_callback(void)
{
    start_passive_scan();
}

static const char *advertisement_event_type_name(uint8_t event_type)
{
    switch (event_type) {
    case 0:
        return "adv_ind";
    case 1:
        return "directed";
    case 2:
        return "scan_ind";
    case 3:
        return "nonconn_ind";
    case 4:
        return "scan_response";
    default:
        return "other";
    }
}

static const char *address_type_name(uint8_t address_type)
{
    switch (address_type) {
    case 0:
        return "public";
    case 1:
        return "random";
    default:
        return "other";
    }
}

static void print_report_csv(const ble_report_t *report)
{
    char address[18];
    snprintf(address, sizeof(address), "%02X:%02X:%02X:%02X:%02X:%02X",
             report->address[5], report->address[4], report->address[3],
             report->address[2], report->address[1], report->address[0]);

    portENTER_CRITICAL(&dropped_reports_mux);
    uint32_t dropped = dropped_reports;
    portEXIT_CRITICAL(&dropped_reports_mux);

    char line[BLE_CSV_LINE_MAX_LEN];
    int line_length = snprintf(line, sizeof(line),
                               "%" PRIu64 ",%u,%s,%u,%s,%s,%d,%u,%lu\n",
                               report->timestamp_us,
                               report->event_type,
                               advertisement_event_type_name(report->event_type),
                               report->address_type,
                               address_type_name(report->address_type),
                               address,
                               report->rssi,
                               report->data_length,
                               (unsigned long)dropped);
    if (line_length < 0 || (size_t)line_length >= sizeof(line)) {
        ESP_LOGE(TAG, "BLE CSV row formatting failed");
        return;
    }
    if (fwrite(line, 1, (size_t)line_length, stdout) != (size_t)line_length ||
        fflush(stdout) != 0) {
        ESP_LOGE(TAG, "Failed to write a BLE CSV row");
    }
}

void ble_monitor_run(void)
{
    report_queue = xQueueCreate(BLE_REPORT_QUEUE_LENGTH, sizeof(ble_report_t));
    if (report_queue == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = ble_sync_callback;

    printf("Modo BLE: metadatos de anuncios pasivos; no exporta sus datos ni captura conexiones.\n");
    printf("timestamp_us,advertisement_event_type_id,advertisement_event_type,"
           "address_type_id,address_type,address,rssi_dbm,"
           "advertisement_data_length,dropped_reports_total\n");
    fflush(stdout);

    nimble_port_freertos_init(ble_host_task);

    uint8_t rows_since_idle = 0;
    while (true) {
        ble_report_t report;
        if (xQueueReceive(report_queue, &report, pdMS_TO_TICKS(1000)) == pdTRUE) {
            print_report_csv(&report);
            if (++rows_since_idle == 16) {
                rows_since_idle = 0;
                vTaskDelay(1);
            }
        } else {
            rows_since_idle = 0;
        }
    }
}
