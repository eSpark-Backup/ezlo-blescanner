#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define TAG "BLE_UUID_TRANSMIT"
#define ESP_BLE_AD_TYPE_128SERVICE_UUID_CMPL 0x07

static const uint8_t UUIDS[][16] = {
    // Predefined GATT Service UUIDs
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00},     // Generic Access Service
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x01, 0x18, 0x00, 0x00},     // Generic Attribute Service
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x04, 0x18, 0x00, 0x00},     // Tx Power
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x09, 0x18, 0x00, 0x00},     // Health Thermometer Service
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x0A, 0x18, 0x00, 0x00},     // Device Information Service
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x0D, 0x18, 0x00, 0x00},     // Heart Rate Service
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x0E, 0x18, 0x00, 0x00},     // Phone Alert Status Service
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x0F, 0x18, 0x00, 0x00},     // Battery Service
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x11, 0x18, 0x00, 0x00},     // Alert Notification Service
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00},     // Human Interface Device (HID) Service
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x15, 0x18, 0x00, 0x00},     // Automation IO
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x1A, 0x18, 0x00, 0x00},     // Environmental Sensing
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x19, 0x18, 0x00, 0x00},     // Location and Navigation
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x21, 0x18, 0x00, 0x00},     // Indoor Positioning

    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x32, 0x11, 0x00, 0x00},     // Message Access Profile (MAP)
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x2F, 0x11, 0x00, 0x00},     // Phone Book Access Profile (PBAP)
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x0D, 0x11, 0x00, 0x00},     // A2DP (Audio Streaming)
    {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x1E, 0x11, 0x00, 0x00},     // Hands-Free Profile (HFP)
};

static const size_t NUM_UUIDS = sizeof(UUIDS) / sizeof(UUIDS[0]);

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Advertising started successfully");
            } else {
                ESP_LOGE(TAG, "Failed to start advertising, error code: %d", param->adv_start_cmpl.status);
            }
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Advertising stopped successfully");
            } else {
                ESP_LOGE(TAG, "Failed to stop advertising, error code: %d", param->adv_stop_cmpl.status);
            }
            break;

        default:
            break;
    }
}

void advertise_uuid_with_name(const uint8_t *uuid, const char *device_name) {
    uint8_t raw_adv_data[31]; // Maximum size of BLE advertising packet is 31 bytes
    uint8_t index = 0;

    // Add Flags field
    raw_adv_data[index++] = 2;                      // Length
    raw_adv_data[index++] = ESP_BLE_AD_TYPE_FLAG;   // Flags type
    raw_adv_data[index++] = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);

    // Add Complete Local Name field
    size_t name_len = strlen(device_name);
    if (name_len > 29 - 3 - 2 - 16) { // Ensure it fits in the advertising packet
        name_len = 29 - 3 - 2 - 16;
    }
    raw_adv_data[index++] = name_len + 1;           // Length
    raw_adv_data[index++] = ESP_BLE_AD_TYPE_NAME_CMPL; // Complete Local Name type
    memcpy(&raw_adv_data[index], device_name, name_len);
    index += name_len;

    // Add 128-bit UUID field
    raw_adv_data[index++] = 16 + 1;                 // Length
    raw_adv_data[index++] = ESP_BLE_AD_TYPE_128SERVICE_UUID_CMPL; // Complete 128-bit UUIDs type
    memcpy(&raw_adv_data[index], uuid, 16);
    index += 16;

    esp_err_t ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, index);
    if (ret) {
        ESP_LOGE(TAG, "Failed to configure raw advertising data, error code: %d", ret);
        return;
    }

    ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret) {
        ESP_LOGE(TAG, "Failed to start advertising, error code: %d", ret);
    }
}

void advertise_uuid(const uint8_t *uuid) {
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = false,
        .min_interval = 0x20,
        .max_interval = 0x40,
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 16, // One 128-bit UUID
        .p_service_uuid = (uint8_t *)uuid,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };

    esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret) {
        ESP_LOGE(TAG, "Failed to configure advertising data, error code: %d", ret);
        return;
    }

    // Start advertising
    ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret) {
        ESP_LOGE(TAG, "Failed to start advertising, error code: %d", ret);
    }
}

void app_main() {
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Initialize the Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Failed to initialize Bluetooth controller, error code: %d", ret);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Failed to enable Bluetooth controller, error code: %d", ret);
        return;
    }

    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Failed to initialize Bluedroid, error code: %d", ret);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Failed to enable Bluedroid, error code: %d", ret);
        return;
    }

    // Register GAP callback handler
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "Failed to register GAP callback, error code: %d", ret);
        return;
    }

    const char *device_name = "RandUUID";       // Random UUID publisher
    size_t current_uuid_index = 0;

    while (1) {
        ESP_LOGI(TAG, "Advertising UUID %zu", current_uuid_index + 1);
        // advertise_uuid(UUIDS[current_uuid_index]);
        advertise_uuid_with_name(UUIDS[current_uuid_index], device_name);

        // Advertise the UUID for 1 second
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Stop advertising
        ret = esp_ble_gap_stop_advertising();
        if (ret) {
            ESP_LOGE(TAG, "Failed to stop advertising, error code: %d", ret);
        }

        // Cycle to the next UUID
        current_uuid_index = (current_uuid_index + 1) % NUM_UUIDS;
    }
}
