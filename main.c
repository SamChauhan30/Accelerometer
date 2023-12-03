#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_common_api.h"

static const char *TAG = "BLE_SCAN";

static void parse_accelerometer_data(uint8_t *raw_data) {
    // Extract accelerometer data from the raw packet
    // Modify this function based on the actual data format
    int16_t x, y, z;
    memcpy(&x, &raw_data[0], sizeof(int16_t));
    memcpy(&y, &raw_data[2], sizeof(int16_t));
    memcpy(&z, &raw_data[4], sizeof(int16_t));

    printf("Accelerometer Data: x=%d, y=%d, z=%d\n", x, y, z);
}

static void check_movement_status(uint8_t *accelerometer_data) {
    // Placeholder function to determine movement status
    // Modify this function based on your movement detection logic
    int16_t movement_threshold = 100;
    int16_t x, y, z;
    memcpy(&x, &accelerometer_data[0], sizeof(int16_t));
    memcpy(&y, &accelerometer_data[2], sizeof(int16_t));
    memcpy(&z, &accelerometer_data[4], sizeof(int16_t));

    if (abs(x) > movement_threshold || abs(y) > movement_threshold || abs(z) > movement_threshold) {
        printf("Movement Status: Moving\n");
    } else {
        printf("Movement Status: Stationary\n");
    }
}

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            if (param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                esp_ble_gap_start_scanning(0);
            } else {
                ESP_LOGE(TAG, "Scan parameters set failed, error status = %x", param->scan_param_cmpl.status);
            }
            break;
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                // Check if the device has Accelerometer data
                uint8_t *accelerometer_data = esp_ble_resolve_adv_data(param->scan_rst.ble_adv, ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE);
                if (accelerometer_data != NULL) {
                    parse_accelerometer_data(&accelerometer_data[2]);  // Assuming the data starts from the 3rd byte
                    check_movement_status(&accelerometer_data[2]);
                }
            }
            break;
        default:
            break;
    }
}

void app_main() {
    esp_err_t ret;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gap_register_callback(gap_cb);

    esp_ble_gap_set_scan_params(&ble_scan_params);

    while (1) {
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
