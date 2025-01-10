#ifndef BLE_BROADCAST_H
#define BLE_BROADCAST_H

#include "esp_gap_ble_api.h"
#include "esp_log.h"

class BleBroadcast {
public:
    BleBroadcast();
    ~BleBroadcast();

    // 启动广播
    esp_err_t start();

    // 停止广播
    esp_err_t stop();

private:

    // 广播参数
    esp_ble_adv_params_t adv_params;

    // 初始化广播参数
    void init_adv_params();
};

#endif // BLE_BROADCAST_H
