#include "BleBroadcast.h"

const char *TAG = "BleBroadcast";

BleBroadcast::BleBroadcast() {
    ESP_LOGI(TAG, "初始化 BLE 广播");
    init_adv_params();
}

BleBroadcast::~BleBroadcast() {
    stop(); // 确保在析构时停止广播
    ESP_LOGI(TAG, "BLE 广播已销毁");
}

void BleBroadcast::init_adv_params() {
    adv_params = {
        .adv_int_min = 0x20,                // 最小广播间隔
        .adv_int_max = 0x40,                // 最大广播间隔
        .adv_type = ADV_TYPE_IND,           // 广播类型
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };
}

esp_err_t BleBroadcast::start() {
    ESP_LOGI(TAG, "启动 BLE 广播");

    // 配置广播数据 (示例数据)
    uint8_t adv_data[] = {
        0x02, 0x01, 0x06,                   // Flags
        0x03, 0x03, 0xAB, 0xCD,             // Complete List of 16-bit UUIDs
        0x0F, 0x09, 'E', 'S', 'P', '3', '2', '-', 'B', 'L', 'E', '-', 'D', 'E', 'V' // Device Name
    };
    esp_err_t ret = esp_ble_gap_config_adv_data_raw(adv_data, sizeof(adv_data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "配置广播数据失败: %s", esp_err_to_name(ret));
        return ret;
    }
    // 启动广播
    ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BLE 广播已启动");
    } else {
        ESP_LOGE(TAG, "启动广播失败: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t BleBroadcast::stop() {
    ESP_LOGI(TAG, "停止 BLE 广播");
    esp_err_t ret = esp_ble_gap_stop_advertising();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BLE 广播已停止");
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "BLE 广播当前未启动");
    } else {
        ESP_LOGE(TAG, "停止广播失败: %s", esp_err_to_name(ret));
    }
    return ret;
}
