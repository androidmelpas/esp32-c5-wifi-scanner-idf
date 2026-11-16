#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "ble_monitor.h"
#include "zigbee_monitor.h"

#define FRAME_QUEUE_LENGTH 96
#define FRAME_SUBTYPE_COUNT 16
#define ADDRESS_COUNT 9
#define SSID_MAX_LEN 32
#define SSID_CSV_MAX_LEN (SSID_MAX_LEN * 4 + 3)
#define CSV_LINE_MAX_LEN 640

static const char *TAG = "wifi_monitor";

typedef enum {
    MONITOR_WIFI,
    MONITOR_BLE,
    MONITOR_ZIGBEE,
} monitor_mode_t;

enum {
    ADDRESS_1,
    ADDRESS_2,
    ADDRESS_3,
    ADDRESS_4,
    ADDRESS_BSSID,
    ADDRESS_SA,
    ADDRESS_DA,
    ADDRESS_TA,
    ADDRESS_RA,
};

typedef struct {
    uint32_t timestamp_us;
    uint32_t phy_siga1;
    uint32_t ht_control;
    uint16_t phy_siga2;
    uint16_t frame_length;
    uint16_t duration_id;
    uint16_t sequence_number;
    uint8_t fragment_number;
    uint16_t qos_control;
    uint8_t channel;
    uint8_t secondary_channel;
    uint8_t phy_format;
    uint8_t phy_rate;
    uint8_t rx_state;
    uint8_t frame_type;
    uint8_t subtype;
    uint8_t protocol_version;
    uint8_t flags;
    uint8_t frame_control_valid;
    uint8_t duration_valid;
    uint8_t sequence_valid;
    uint8_t qos_valid;
    uint8_t ht_control_valid;
    uint8_t address_valid_mask;
    uint8_t addresses[ADDRESS_COUNT][6];
    uint8_t ssid_state;
    uint8_t ssid_length;
    uint8_t ssid[SSID_MAX_LEN];
    int8_t rssi;
    int8_t noise_floor;
} frame_event_t;

static const uint8_t supported_channels[] = {
    36, 40, 44, 48, 52, 56, 60, 64,
    100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
    149, 153, 157, 161, 165, 169, 173, 177,
};
static const char *const management_subtype_names[FRAME_SUBTYPE_COUNT] = {
    "assoc_req", "assoc_resp", "reassoc_req", "reassoc_resp",
    "probe_req", "probe_resp", "timing_advertisement", "reserved",
    "beacon", "atim", "disassoc", "auth", "deauth", "action",
    "action_no_ack", "reserved",
};
static const char *const control_subtype_names[FRAME_SUBTYPE_COUNT] = {
    "reserved", "reserved", "reserved", "reserved",
    "reserved", "reserved", "reserved", "control_wrapper",
    "block_ack_req", "block_ack", "ps_poll", "rts",
    "cts", "ack", "cf_end", "cf_end_ack",
};
static const char *const data_subtype_names[FRAME_SUBTYPE_COUNT] = {
    "data", "data_cf_ack", "data_cf_poll", "data_cf_ack_poll",
    "null", "cf_ack", "cf_poll", "cf_ack_poll",
    "qos_data", "qos_data_cf_ack", "qos_data_cf_poll", "qos_data_cf_ack_poll",
    "qos_null", "reserved", "qos_cf_poll", "qos_cf_ack_poll",
};

static QueueHandle_t frame_queue;
static uint32_t dropped_frames;
static portMUX_TYPE dropped_frames_mux = portMUX_INITIALIZER_UNLOCKED;

static bool is_supported_channel(long channel)
{
    if (channel >= 1 && channel <= 14) {
        return true;
    }

    for (size_t i = 0; i < sizeof(supported_channels) / sizeof(supported_channels[0]); ++i) {
        if (supported_channels[i] == channel) {
            return true;
        }
    }
    return false;
}

static bool read_console_line(char *input, size_t input_size, bool *overflow_out,
                              TickType_t timeout)
{
    size_t input_length = 0;
    bool input_overflow = false;
    TickType_t started = xTaskGetTickCount();

    while ((TickType_t)(xTaskGetTickCount() - started) < timeout) {
        int character = getchar();
        if (character == EOF) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (character != '\r' && character != '\n') {
            if (!input_overflow) {
                if (input_length < input_size - 1) {
                    input[input_length++] = (char)character;
                } else {
                    input_overflow = true;
                }
            }
            continue;
        }

        if (input_length == 0 && !input_overflow) {
            continue;
        }

        input[input_length] = '\0';
        *overflow_out = input_overflow;
        return true;
    }

    input[0] = '\0';
    *overflow_out = false;
    return false;
}

static monitor_mode_t read_monitor_mode(void)
{
    char input[16];
    while (true) {
        bool input_overflow;
        printf("\nSelecciona el analizador:\n");
        printf("1. Wi-Fi\n2. BLE\n3. Zigbee (IEEE 802.15.4)\n");
        printf("Tienes 10 segundos; sin respuesta se elegira Wi-Fi.\nOpcion: ");
        fflush(stdout);

        if (!read_console_line(input, sizeof(input), &input_overflow,
                               pdMS_TO_TICKS(10000))) {
            printf("\nTiempo agotado; se elige Wi-Fi.\n");
            return MONITOR_WIFI;
        }

        if (input_overflow) {
            printf("Opcion no valida; introduce 1, 2 o 3.\n");
            continue;
        }
        if (strcmp(input, "1") == 0 || strcasecmp(input, "wifi") == 0 ||
            strcasecmp(input, "w") == 0) {
            return MONITOR_WIFI;
        }
        if (strcmp(input, "2") == 0 || strcasecmp(input, "ble") == 0 ||
            strcasecmp(input, "b") == 0) {
            return MONITOR_BLE;
        }
        if (strcmp(input, "3") == 0 || strcasecmp(input, "zigbee") == 0 ||
            strcasecmp(input, "z") == 0) {
            return MONITOR_ZIGBEE;
        }

        printf("Opcion no valida; introduce 1, 2 o 3.\n");
    }
}

static uint8_t read_channel_from_console(void)
{
    char input[32];
    while (true) {
        bool input_overflow;
        printf("\nCanales 2.4 GHz: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14\n");
        printf("Canales 5 GHz: 36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116,\n");
        printf("120, 124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165, 169, 173, 177\n");
        printf("Nota: la disponibilidad depende de la normativa local; el canal 14 puede estar restringido.\n");
        printf("Tienes 10 segundos; sin respuesta se elegira el canal 100.\n");
        printf("Escribe el canal y pulsa Enter: ");
        fflush(stdout);

        if (!read_console_line(input, sizeof(input), &input_overflow,
                               pdMS_TO_TICKS(10000))) {
            printf("\nTiempo agotado; se elige el canal 100.\n");
            return 100;
        }

        char *end = NULL;
        long channel = strtol(input, &end, 10);
        while (end != NULL && (*end == ' ' || *end == '\t')) {
            ++end;
        }

        if (!input_overflow && end != input && end != NULL && *end == '\0' &&
            is_supported_channel(channel)) {
            return (uint8_t)channel;
        }

        printf("Entrada no valida. Elige uno de los canales de la lista.\n");
    }
}

static bool has_bytes(uint16_t length, size_t offset, size_t count)
{
    return offset <= length && count <= (size_t)length - offset;
}

static void set_address(frame_event_t *event, uint8_t address_index,
                        const uint8_t *frame, size_t offset)
{
    memcpy(event->addresses[address_index], frame + offset, 6);
    event->address_valid_mask |= (uint8_t)(1U << address_index);
}

static void copy_address(frame_event_t *event, uint8_t target, uint8_t source)
{
    if ((event->address_valid_mask & (1U << source)) != 0) {
        memcpy(event->addresses[target], event->addresses[source], 6);
        event->address_valid_mask |= (uint8_t)(1U << target);
    }
}

static const char *frame_type_name(uint8_t frame_type)
{
    switch (frame_type) {
    case 0:
        return "management";
    case 1:
        return "control";
    case 2:
        return "data";
    case 3:
        return "extension";
    default:
        return "misc";
    }
}

static const char *frame_subtype_name(uint8_t frame_type, uint8_t subtype)
{
    if (frame_type == 0) {
        return management_subtype_names[subtype];
    }
    if (frame_type == 1) {
        return control_subtype_names[subtype];
    }
    if (frame_type == 2) {
        return data_subtype_names[subtype];
    }
    return frame_type == 3 ? "extension" : "NA";
}

static void parse_mac_header(frame_event_t *event, const uint8_t *frame)
{
    uint16_t length = event->frame_length;
    if (length < 2) {
        return;
    }

    uint16_t frame_control = (uint16_t)frame[0] | ((uint16_t)frame[1] << 8);
    event->frame_type = (uint8_t)((frame_control >> 2) & 0x03);
    event->subtype = (uint8_t)((frame_control >> 4) & 0x0F);
    event->protocol_version = (uint8_t)(frame_control & 0x03);
    event->flags = (uint8_t)((frame_control >> 8) & 0xFF);
    event->frame_control_valid = 1;

    if (!has_bytes(length, 2, 2)) {
        return;
    }
    event->duration_id = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
    event->duration_valid = 1;

    if (event->frame_type == 0 || event->frame_type == 2) {
        if (!has_bytes(length, 4, 20)) {
            return;
        }

        set_address(event, ADDRESS_1, frame, 4);
        set_address(event, ADDRESS_2, frame, 10);
        set_address(event, ADDRESS_3, frame, 16);
        uint16_t sequence_control = (uint16_t)frame[22] |
                                    ((uint16_t)frame[23] << 8);
        event->sequence_number = (uint16_t)(sequence_control >> 4);
        event->fragment_number = (uint8_t)(sequence_control & 0x0F);
        event->sequence_valid = 1;
        uint8_t to_ds = (uint8_t)((frame_control >> 8) & 0x01);
        uint8_t from_ds = (uint8_t)((frame_control >> 9) & 0x01);

        if (event->frame_type == 0) {
            copy_address(event, ADDRESS_DA, ADDRESS_1);
            copy_address(event, ADDRESS_SA, ADDRESS_2);
            copy_address(event, ADDRESS_BSSID, ADDRESS_3);
            copy_address(event, ADDRESS_TA, ADDRESS_2);
            copy_address(event, ADDRESS_RA, ADDRESS_1);
        } else if (to_ds && from_ds) {
            if (has_bytes(length, 24, 6)) {
                set_address(event, ADDRESS_4, frame, 24);
            }
            copy_address(event, ADDRESS_DA, ADDRESS_3);
            copy_address(event, ADDRESS_SA, ADDRESS_4);
            copy_address(event, ADDRESS_TA, ADDRESS_2);
            copy_address(event, ADDRESS_RA, ADDRESS_1);
        } else if (to_ds) {
            copy_address(event, ADDRESS_DA, ADDRESS_3);
            copy_address(event, ADDRESS_SA, ADDRESS_2);
            copy_address(event, ADDRESS_BSSID, ADDRESS_1);
            copy_address(event, ADDRESS_TA, ADDRESS_2);
            copy_address(event, ADDRESS_RA, ADDRESS_1);
        } else if (from_ds) {
            copy_address(event, ADDRESS_DA, ADDRESS_1);
            copy_address(event, ADDRESS_SA, ADDRESS_3);
            copy_address(event, ADDRESS_BSSID, ADDRESS_2);
            copy_address(event, ADDRESS_TA, ADDRESS_2);
            copy_address(event, ADDRESS_RA, ADDRESS_1);
        } else {
            copy_address(event, ADDRESS_DA, ADDRESS_1);
            copy_address(event, ADDRESS_SA, ADDRESS_2);
            copy_address(event, ADDRESS_BSSID, ADDRESS_3);
            copy_address(event, ADDRESS_TA, ADDRESS_2);
            copy_address(event, ADDRESS_RA, ADDRESS_1);
        }

        size_t qos_offset = (to_ds && from_ds) ? 30 : 24;
        if (event->frame_type == 2 && (event->subtype & 0x08) != 0 &&
            has_bytes(length, qos_offset, 2)) {
            event->qos_control = (uint16_t)frame[qos_offset] |
                                 ((uint16_t)frame[qos_offset + 1] << 8);
            event->qos_valid = 1;
            if ((event->flags & 0x10) != 0 &&
                has_bytes(length, qos_offset + 2, 4)) {
                event->ht_control = (uint32_t)frame[qos_offset + 2] |
                                    ((uint32_t)frame[qos_offset + 3] << 8) |
                                    ((uint32_t)frame[qos_offset + 4] << 16) |
                                    ((uint32_t)frame[qos_offset + 5] << 24);
                event->ht_control_valid = 1;
            }
        }
        return;
    }

    if (event->frame_type == 1) {
        if (has_bytes(length, 4, 6)) {
            set_address(event, ADDRESS_1, frame, 4);
            copy_address(event, ADDRESS_RA, ADDRESS_1);
        }
        if (event->subtype != 12 && event->subtype != 13 &&
            has_bytes(length, 10, 6)) {
            set_address(event, ADDRESS_2, frame, 10);
            copy_address(event, ADDRESS_TA, ADDRESS_2);
        }
    }
}

static void parse_beacon_ssid(frame_event_t *event, const uint8_t *frame)
{
    if (event->frame_type != 0 ||
        (event->subtype != 5 && event->subtype != 8) ||
        event->frame_length < 40) {
        return;
    }

    uint16_t ie_end = (uint16_t)(event->frame_length - 4);
    size_t offset = 36;
    while (has_bytes(ie_end, offset, 2)) {
        uint8_t element_id = frame[offset];
        uint8_t element_length = frame[offset + 1];
        offset += 2;
        if (!has_bytes(ie_end, offset, element_length)) {
            return;
        }
        if (element_id == 0) {
            if (element_length <= SSID_MAX_LEN) {
                memcpy(event->ssid, frame + offset, element_length);
                event->ssid_length = element_length;
                event->ssid_state = 1;
            }
            return;
        }
        offset += element_length;
    }
}

static uint16_t channel_frequency_mhz(uint8_t channel)
{
    if (channel >= 1 && channel <= 13) {
        return (uint16_t)(2407 + 5 * channel);
    }
    if (channel == 14) {
        return 2484;
    }
    return channel == 0 ? 0 : (uint16_t)(5000 + 5 * channel);
}

static const char *channel_band(uint8_t channel)
{
    if (channel >= 1 && channel <= 14) {
        return "2.4GHz";
    }
    return channel == 0 ? "NA" : "5GHz";
}

static void format_mac(char *output, const uint8_t *address, bool valid)
{
    if (!valid) {
        strcpy(output, "NA");
        return;
    }
    snprintf(output, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             address[0], address[1], address[2],
             address[3], address[4], address[5]);
}

static void format_ssid(char *output, size_t output_size, const frame_event_t *event)
{
    if (event->ssid_state == 0) {
        strcpy(output, "NA");
        return;
    }

    size_t used = 0;
    output[used++] = '"';
    for (size_t i = 0; i < event->ssid_length; ++i) {
        uint8_t value = event->ssid[i];
        if (value == '"') {
            output[used++] = '"';
            output[used++] = '"';
        } else if (value >= 0x20 && value <= 0x7E) {
            output[used++] = (char)value;
        } else {
            snprintf(output + used, output_size - used, "\\x%02X", value);
            used += 4;
        }
    }
    output[used++] = '"';
    output[used] = '\0';
}

static void print_event_csv(const frame_event_t *event)
{
    char macs[ADDRESS_COUNT][18];
    char ssid[SSID_CSV_MAX_LEN];
    for (size_t i = 0; i < ADDRESS_COUNT; ++i) {
        format_mac(macs[i], event->addresses[i],
                   (event->address_valid_mask & (1U << i)) != 0);
    }
    format_ssid(ssid, sizeof(ssid), event);

    bool has_frame_control = event->frame_control_valid != 0;
    uint8_t to_ds = (uint8_t)((event->flags >> 0) & 0x01);
    uint8_t from_ds = (uint8_t)((event->flags >> 1) & 0x01);
    uint8_t more_fragments = (uint8_t)((event->flags >> 2) & 0x01);
    uint8_t retry = (uint8_t)((event->flags >> 3) & 0x01);
    uint8_t power_management = (uint8_t)((event->flags >> 4) & 0x01);
    uint8_t more_data = (uint8_t)((event->flags >> 5) & 0x01);
    uint8_t protected_frame = (uint8_t)((event->flags >> 6) & 0x01);
    uint8_t order = (uint8_t)((event->flags >> 7) & 0x01);
    char oui[9] = "NA";
    char aid[6] = "";
    char qos_control[7] = "";
    char ht_control[11] = "";
    char protocol_version[4] = "";
    char type_id[4] = "";
    char subtype_id[4] = "";
    char to_ds_text[4] = "";
    char from_ds_text[4] = "";
    char retry_text[4] = "";
    char more_fragments_text[4] = "";
    char duration_id[6] = "";
    char sequence_number[6] = "";
    char fragment_number[4] = "";
    char frequency[6] = "";
    char power_management_text[4] = "";
    char more_data_text[4] = "";
    char protected_text[4] = "";
    char order_text[4] = "";
    char ul_bit_text[4] = "";
    char ig_bit_text[4] = "";

    if ((event->address_valid_mask & (1U << ADDRESS_2)) != 0) {
        const uint8_t *address = event->addresses[ADDRESS_2];
        snprintf(oui, sizeof(oui), "%02X:%02X:%02X",
                 address[0], address[1], address[2]);
        snprintf(ul_bit_text, sizeof(ul_bit_text), "%u", (address[0] >> 1) & 0x01);
        snprintf(ig_bit_text, sizeof(ig_bit_text), "%u", address[0] & 0x01);
    }
    if (has_frame_control) {
        snprintf(protocol_version, sizeof(protocol_version), "%u",
                 event->protocol_version);
        snprintf(type_id, sizeof(type_id), "%u", event->frame_type);
        snprintf(subtype_id, sizeof(subtype_id), "%u", event->subtype);
        snprintf(to_ds_text, sizeof(to_ds_text), "%u", to_ds);
        snprintf(from_ds_text, sizeof(from_ds_text), "%u", from_ds);
        snprintf(retry_text, sizeof(retry_text), "%u", retry);
        snprintf(more_fragments_text, sizeof(more_fragments_text), "%u", more_fragments);
        snprintf(power_management_text, sizeof(power_management_text), "%u", power_management);
        snprintf(more_data_text, sizeof(more_data_text), "%u", more_data);
        snprintf(protected_text, sizeof(protected_text), "%u", protected_frame);
        snprintf(order_text, sizeof(order_text), "%u", order);
        if (event->duration_valid) {
            snprintf(duration_id, sizeof(duration_id), "%u", event->duration_id);
        }
        if (event->sequence_valid) {
            snprintf(sequence_number, sizeof(sequence_number), "%u", event->sequence_number);
            snprintf(fragment_number, sizeof(fragment_number), "%u", event->fragment_number);
        }
        if (event->frame_type == 1 && event->subtype == 10 && event->duration_valid) {
            snprintf(aid, sizeof(aid), "%u", event->duration_id & 0x3FFF);
        }
        if (event->qos_valid) {
            snprintf(qos_control, sizeof(qos_control), "%04X", event->qos_control);
        }
        if (event->ht_control_valid) {
            snprintf(ht_control, sizeof(ht_control), "%08lX",
                     (unsigned long)event->ht_control);
        }
    }

    uint16_t frequency_mhz = channel_frequency_mhz(event->channel);
    if (frequency_mhz != 0) {
        snprintf(frequency, sizeof(frequency), "%u", frequency_mhz);
    }

    portENTER_CRITICAL(&dropped_frames_mux);
    uint32_t dropped = dropped_frames;
    portEXIT_CRITICAL(&dropped_frames_mux);

    char line[CSV_LINE_MAX_LEN];
    int line_length = snprintf(
        line, sizeof(line),
        "%lu,%u,%s,%u,%d,%d,%d,%s,%u,0x%08lX,0x%04X,%u,%u,%u,"
        "%s,%s,%s,%s,%s,%s,%s,%s,"
        "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,"
        "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%lu\n",
        (unsigned long)event->timestamp_us,
        event->channel,
        frequency,
        event->secondary_channel,
        event->rssi,
        event->noise_floor,
        (int)event->rssi - (int)event->noise_floor,
        channel_band(event->channel),
        event->phy_format,
        (unsigned long)event->phy_siga1,
        event->phy_siga2,
        event->phy_rate,
        event->frame_length,
        event->rx_state,
        has_frame_control ? protocol_version : "",
        frame_type_name(event->frame_type),
        has_frame_control ? frame_subtype_name(event->frame_type, event->subtype) : "NA",
        type_id,
        subtype_id,
        ssid,
        to_ds_text,
        from_ds_text,
        macs[ADDRESS_1],
        macs[ADDRESS_2],
        macs[ADDRESS_3],
        macs[ADDRESS_4],
        macs[ADDRESS_BSSID],
        macs[ADDRESS_SA],
        macs[ADDRESS_DA],
        macs[ADDRESS_TA],
        macs[ADDRESS_RA],
        oui,
        ul_bit_text,
        ig_bit_text,
        duration_id,
        aid,
        retry_text,
        more_fragments_text,
        power_management_text,
        more_data_text,
        protected_text,
        order_text,
        sequence_number,
        fragment_number,
        qos_control,
        ht_control,
        (unsigned long)dropped);
    if (line_length < 0 || (size_t)line_length >= sizeof(line)) {
        ESP_LOGE(TAG, "CSV row formatting failed or exceeded its buffer");
        return;
    }
    if (fwrite(line, 1, (size_t)line_length, stdout) != (size_t)line_length ||
        fflush(stdout) != 0) {
        ESP_LOGE(TAG, "Failed to write a CSV row to the serial console");
    }
}

static void promiscuous_rx_callback(void *buffer, wifi_promiscuous_pkt_type_t packet_type)
{
    if (buffer == NULL || packet_type > WIFI_PKT_MISC) {
        return;
    }

    const wifi_promiscuous_pkt_t *packet = (const wifi_promiscuous_pkt_t *)buffer;
    frame_event_t event = {
        .timestamp_us = packet->rx_ctrl.timestamp,
        .phy_siga1 = packet->rx_ctrl.he_siga1,
        .phy_siga2 = packet->rx_ctrl.he_siga2,
        .frame_length = packet->rx_ctrl.sig_len,
        .channel = packet->rx_ctrl.channel,
        .secondary_channel = packet->rx_ctrl.second,
        .phy_format = (uint8_t)packet->rx_ctrl.cur_bb_format,
        .phy_rate = (uint8_t)packet->rx_ctrl.rate,
        .rx_state = (uint8_t)packet->rx_ctrl.rx_state,
        .rssi = (int8_t)packet->rx_ctrl.rssi,
        .noise_floor = (int8_t)packet->rx_ctrl.noise_floor,
        .frame_type = 4,
    };

    if (packet_type != WIFI_PKT_MISC) {
        parse_mac_header(&event, packet->payload);
        parse_beacon_ssid(&event, packet->payload);
    }

    if (xQueueSend(frame_queue, &event, 0) != pdTRUE) {
        portENTER_CRITICAL(&dropped_frames_mux);
        ++dropped_frames;
        portEXIT_CRITICAL(&dropped_frames_mux);
    }
}

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void run_wifi_monitor(uint8_t channel)
{
    frame_queue = xQueueCreate(FRAME_QUEUE_LENGTH, sizeof(frame_event_t));
    if (frame_queue == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));

    wifi_promiscuous_filter_t packet_filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT |
                       WIFI_PROMIS_FILTER_MASK_CTRL |
                       WIFI_PROMIS_FILTER_MASK_DATA |
                       WIFI_PROMIS_FILTER_MASK_MISC |
                       WIFI_PROMIS_FILTER_MASK_FCSFAIL |
                       WIFI_PROMIS_FILTER_MASK_DATA_MPDU |
                       WIFI_PROMIS_FILTER_MASK_DATA_AMPDU,
    };
    wifi_promiscuous_filter_t control_filter = {
        .filter_mask = WIFI_PROMIS_CTRL_FILTER_MASK_ALL,
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&packet_filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_ctrl_filter(&control_filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(promiscuous_rx_callback));

    printf("Canal elegido: %u (%s)\n", (unsigned)channel, channel_band(channel));
    printf("rx_timestamp_us,channel,channel_frequency_mhz,secondary_channel,rssi_dbm,"
           "noise_floor_dbm,snr_db,band,phy_format,phy_siga1_hex,phy_siga2_hex,phy_rate_field,"
           "frame_length_bytes,rx_state,protocol_version,frame_type,subtype,frame_type_id,"
           "subtype_id,ssid,to_ds,from_ds,addr1,addr2,addr3,addr4,bssid,sa,da,ta,ra,addr2_oui,"
           "addr2_ul_bit,addr2_ig_bit,duration_id,aid,retry,more_fragments,power_management,"
           "more_data,protected,order,sequence_number,fragment_number,qos_control_hex,"
           "ht_control_hex,"
           "dropped_frames_total\n");
    fflush(stdout);

    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    uint8_t rows_since_idle = 0;
    while (true) {
        frame_event_t event;
        if (xQueueReceive(frame_queue, &event, pdMS_TO_TICKS(1000)) == pdTRUE) {
            print_event_csv(&event);
            if (++rows_since_idle == 16) {
                rows_since_idle = 0;
                vTaskDelay(1);
            }
        } else {
            rows_since_idle = 0;
        }
    }
}

void app_main(void)
{
    printf("\nESP32-C5 - analizador pasivo Wi-Fi / BLE / Zigbee\n");
    printf("Consola serie: 921600 baudios.\n");

    monitor_mode_t mode = read_monitor_mode();
    uint8_t channel = 0;
    if (mode == MONITOR_WIFI) {
        channel = read_channel_from_console();
    }

    initialize_nvs();

    switch (mode) {
    case MONITOR_WIFI:
        printf("Modo Wi-Fi: solo cabeceras; SSID anunciado en beacons/respuestas de probe.\n");
        run_wifi_monitor(channel);
        break;
    case MONITOR_BLE:
        ble_monitor_run();
        break;
    case MONITOR_ZIGBEE:
        zigbee_monitor_run();
        break;
    }
}
