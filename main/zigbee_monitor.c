#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_ieee802154.h"
#include "esp_ieee802154_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define ZIGBEE_FRAME_QUEUE_LENGTH 96
#define ZIGBEE_CSV_LINE_MAX_LEN 320
#define ZIGBEE_CHANNEL_DWELL_MS 100

static const char *TAG = "zigbee_monitor";
static const uint8_t zigbee_channels[] = {
    11, 12, 13, 14, 15, 16, 17, 18,
    19, 20, 21, 22, 23, 24, 25, 26,
};

typedef struct {
    uint64_t timestamp_us;
    int8_t rssi;
    uint8_t lqi;
    uint8_t channel;
    uint8_t frame_length;
    uint8_t frame_type;
    uint8_t frame_version;
    uint8_t security_enabled;
    uint8_t frame_pending;
    uint8_t ack_request;
    uint8_t pan_id_compression;
    uint8_t destination_address_mode;
    uint8_t source_address_mode;
    uint8_t sequence_number;
    uint8_t frame_control_valid;
    uint8_t sequence_valid;
    uint8_t destination_pan_id_valid;
    uint8_t source_pan_id_valid;
    uint8_t destination_address_length;
    uint8_t source_address_length;
    uint16_t destination_pan_id;
    uint16_t source_pan_id;
    uint8_t destination_address[8];
    uint8_t source_address[8];
} zigbee_frame_t;

static QueueHandle_t frame_queue;
static uint32_t dropped_frames;
static uint32_t receive_handler_errors;
static portMUX_TYPE dropped_frames_mux = portMUX_INITIALIZER_UNLOCKED;

static inline bool IRAM_ATTR has_frame_bytes(uint8_t frame_length, size_t offset,
                                            size_t count)
{
    return offset >= 1 && offset <= frame_length &&
           count <= (size_t)frame_length - offset + 1;
}

void IRAM_ATTR esp_ieee802154_receive_done(uint8_t *frame,
                                           esp_ieee802154_frame_info_t *frame_info)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (frame != NULL && frame_info != NULL) {
        zigbee_frame_t event = {
            .timestamp_us = frame_info->timestamp,
            .rssi = frame_info->rssi,
            .lqi = frame_info->lqi,
            .channel = frame_info->channel,
            .frame_length = frame[0],
        };

        if (frame[0] >= 2) {
            uint16_t frame_control = (uint16_t)frame[1] |
                                     ((uint16_t)frame[2] << 8);
            event.frame_type = (uint8_t)(frame_control & 0x07);
            event.security_enabled = (uint8_t)((frame_control >> 3) & 0x01);
            event.frame_pending = (uint8_t)((frame_control >> 4) & 0x01);
            event.ack_request = (uint8_t)((frame_control >> 5) & 0x01);
            event.pan_id_compression = (uint8_t)((frame_control >> 6) & 0x01);
            event.destination_address_mode = (uint8_t)((frame_control >> 10) & 0x03);
            event.frame_version = (uint8_t)((frame_control >> 12) & 0x03);
            event.source_address_mode = (uint8_t)((frame_control >> 14) & 0x03);
            event.frame_control_valid = 1;

            bool sequence_suppressed = ((frame_control >> 8) & 0x01) != 0;
            size_t offset = 3;
            bool header_valid = true;
            if (!sequence_suppressed) {
                if (has_frame_bytes(frame[0], offset, 1)) {
                    event.sequence_number = frame[offset];
                    event.sequence_valid = 1;
                    ++offset;
                } else {
                    header_valid = false;
                }
            }

            if (header_valid && event.frame_version <= 1 &&
                event.frame_type <= 3 &&
                event.destination_address_mode != 1 &&
                event.source_address_mode != 1) {
                if (event.destination_address_mode != 0) {
                    if (has_frame_bytes(frame[0], offset, 2)) {
                        event.destination_pan_id = (uint16_t)frame[offset] |
                                                   ((uint16_t)frame[offset + 1] << 8);
                        event.destination_pan_id_valid = 1;
                        offset += 2;
                    } else {
                        header_valid = false;
                    }

                    uint8_t address_length =
                        event.destination_address_mode == 2 ? 2 : 8;
                    if (header_valid && has_frame_bytes(frame[0], offset, address_length)) {
                        memcpy(event.destination_address, frame + offset, address_length);
                        event.destination_address_length = address_length;
                        offset += address_length;
                    } else {
                        header_valid = false;
                    }
                }

                if (header_valid && event.source_address_mode != 0) {
                    bool source_pan_compressed =
                        event.pan_id_compression && event.destination_address_mode != 0;
                    if (source_pan_compressed && event.destination_pan_id_valid) {
                        event.source_pan_id = event.destination_pan_id;
                        event.source_pan_id_valid = 1;
                    } else if (!source_pan_compressed) {
                        if (has_frame_bytes(frame[0], offset, 2)) {
                            event.source_pan_id = (uint16_t)frame[offset] |
                                                  ((uint16_t)frame[offset + 1] << 8);
                            event.source_pan_id_valid = 1;
                            offset += 2;
                        } else {
                            header_valid = false;
                        }
                    }

                    uint8_t address_length =
                        event.source_address_mode == 2 ? 2 : 8;
                    if (header_valid && has_frame_bytes(frame[0], offset, address_length)) {
                        memcpy(event.source_address, frame + offset, address_length);
                        event.source_address_length = address_length;
                    }
                }
            }
        }

        if (xQueueSendFromISR(frame_queue, &event, &higher_priority_task_woken) != pdTRUE) {
            portENTER_CRITICAL_ISR(&dropped_frames_mux);
            ++dropped_frames;
            portEXIT_CRITICAL_ISR(&dropped_frames_mux);
        }
    }

    if (frame != NULL) {
        if (esp_ieee802154_receive_handle_done(frame) != ESP_OK) {
            portENTER_CRITICAL_ISR(&dropped_frames_mux);
            ++receive_handler_errors;
            portEXIT_CRITICAL_ISR(&dropped_frames_mux);
        }
    }
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static const char *frame_type_name(uint8_t frame_type)
{
    switch (frame_type) {
    case 0:
        return "beacon";
    case 1:
        return "data";
    case 2:
        return "ack";
    case 3:
        return "command";
    case 5:
        return "multipurpose";
    case 6:
        return "fragment";
    case 7:
        return "extended";
    default:
        return "reserved";
    }
}

static void format_address(char *output, size_t output_size,
                           const uint8_t *address, uint8_t address_length)
{
    if (address_length == 2) {
        snprintf(output, output_size, "%02X%02X", address[1], address[0]);
    } else if (address_length == 8) {
        snprintf(output, output_size,
                 "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 address[7], address[6], address[5], address[4],
                 address[3], address[2], address[1], address[0]);
    } else {
        output[0] = '\0';
    }
}

static void print_frame_csv(const zigbee_frame_t *frame)
{
    char frame_type_id[4] = "";
    char frame_type[16] = "";
    char frame_version[4] = "";
    char security_enabled[4] = "";
    char frame_pending[4] = "";
    char ack_request[4] = "";
    char pan_id_compression[4] = "";
    char destination_address_mode[4] = "";
    char source_address_mode[4] = "";
    char sequence_number[4] = "";
    char destination_pan_id[5] = "";
    char source_pan_id[5] = "";
    char destination_address[24] = "";
    char source_address[24] = "";

    if (frame->frame_control_valid) {
        snprintf(frame_type_id, sizeof(frame_type_id), "%u", frame->frame_type);
        snprintf(frame_type, sizeof(frame_type), "%s", frame_type_name(frame->frame_type));
        snprintf(frame_version, sizeof(frame_version), "%u", frame->frame_version);
        snprintf(security_enabled, sizeof(security_enabled), "%u", frame->security_enabled);
        snprintf(frame_pending, sizeof(frame_pending), "%u", frame->frame_pending);
        snprintf(ack_request, sizeof(ack_request), "%u", frame->ack_request);
        snprintf(pan_id_compression, sizeof(pan_id_compression), "%u",
                 frame->pan_id_compression);
        snprintf(destination_address_mode, sizeof(destination_address_mode), "%u",
                 frame->destination_address_mode);
        snprintf(source_address_mode, sizeof(source_address_mode), "%u",
                 frame->source_address_mode);
        if (frame->sequence_valid) {
            snprintf(sequence_number, sizeof(sequence_number), "%u", frame->sequence_number);
        }
    }
    if (frame->destination_pan_id_valid) {
        snprintf(destination_pan_id, sizeof(destination_pan_id), "%04X",
                 frame->destination_pan_id);
    }
    if (frame->source_pan_id_valid) {
        snprintf(source_pan_id, sizeof(source_pan_id), "%04X", frame->source_pan_id);
    }
    format_address(destination_address, sizeof(destination_address),
                   frame->destination_address, frame->destination_address_length);
    format_address(source_address, sizeof(source_address),
                   frame->source_address, frame->source_address_length);

    portENTER_CRITICAL(&dropped_frames_mux);
    uint32_t dropped = dropped_frames;
    uint32_t handler_errors = receive_handler_errors;
    portEXIT_CRITICAL(&dropped_frames_mux);

    char line[ZIGBEE_CSV_LINE_MAX_LEN];
    int line_length = snprintf(
        line, sizeof(line),
        "%" PRIu64 ",%u,%d,%u,%u,"
        "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,"
        "%lu,%lu\n",
        frame->timestamp_us,
        frame->channel,
        frame->rssi,
        frame->lqi,
        frame->frame_length,
        frame_type,
        frame_type_id,
        frame_version,
        security_enabled,
        frame_pending,
        ack_request,
        pan_id_compression,
        destination_address_mode,
        source_address_mode,
        destination_pan_id,
        destination_address,
        source_pan_id,
        source_address,
        sequence_number,
        (unsigned long)dropped,
        (unsigned long)handler_errors);
    if (line_length < 0 || (size_t)line_length >= sizeof(line)) {
        ESP_LOGE(TAG, "Zigbee CSV row formatting failed");
        return;
    }
    if (fwrite(line, 1, (size_t)line_length, stdout) != (size_t)line_length ||
        fflush(stdout) != 0) {
        ESP_LOGE(TAG, "Failed to write a Zigbee CSV row");
    }
}

void zigbee_monitor_run(void)
{
    frame_queue = xQueueCreate(ZIGBEE_FRAME_QUEUE_LENGTH, sizeof(zigbee_frame_t));
    if (frame_queue == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    ESP_ERROR_CHECK(esp_ieee802154_enable());
    ESP_ERROR_CHECK(esp_ieee802154_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_ieee802154_set_channel(zigbee_channels[0]));
    ESP_ERROR_CHECK(esp_ieee802154_receive());

    printf("Modo Zigbee: metadatos y cabecera MAC; no exporta payload. Salto canales 11-26 cada %u ms.\n",
           ZIGBEE_CHANNEL_DWELL_MS);
    printf("timestamp_us,channel,rssi_dbm,lqi,frame_length_bytes,frame_type,frame_type_id,"
           "frame_version,security_enabled,frame_pending,ack_request,pan_id_compression,"
           "destination_address_mode,source_address_mode,destination_pan_id_hex,"
           "destination_address,source_pan_id_hex,source_address,sequence_number,"
           "dropped_frames_total,receive_handler_errors_total\n");
    fflush(stdout);

    size_t channel_index = 0;
    TickType_t next_hop = xTaskGetTickCount() + pdMS_TO_TICKS(ZIGBEE_CHANNEL_DWELL_MS);
    uint8_t rows_since_idle = 0;
    while (true) {
        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(now - next_hop) >= 0) {
            channel_index = (channel_index + 1) %
                            (sizeof(zigbee_channels) / sizeof(zigbee_channels[0]));
            ESP_ERROR_CHECK(esp_ieee802154_sleep());
            ESP_ERROR_CHECK(esp_ieee802154_set_channel(zigbee_channels[channel_index]));
            ESP_ERROR_CHECK(esp_ieee802154_receive());
            next_hop = xTaskGetTickCount() + pdMS_TO_TICKS(ZIGBEE_CHANNEL_DWELL_MS);
        }

        now = xTaskGetTickCount();
        if ((int32_t)(now - next_hop) >= 0) {
            continue;
        }
        TickType_t wait_ticks = next_hop - now;
        zigbee_frame_t frame;
        if (xQueueReceive(frame_queue, &frame, wait_ticks) == pdTRUE) {
            print_frame_csv(&frame);
            if (++rows_since_idle == 16) {
                rows_since_idle = 0;
                vTaskDelay(1);
            }
        } else {
            rows_since_idle = 0;
        }
    }
}
