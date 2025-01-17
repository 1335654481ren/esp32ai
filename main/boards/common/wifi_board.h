#ifndef WIFI_BOARD_H
#define WIFI_BOARD_H

#include "board.h"

enum WIFI_CONFIG_TYPE{
    SOFT_AP_WEB,
    SMART_CFG,
    AIR_KISS_CFG
};

class WifiBoard : public Board {
protected:
    bool wifi_config_mode_ = false;
    WIFI_CONFIG_TYPE wifi_cfg_type = SMART_CFG;
    virtual std::string GetBoardJson() override;
    WifiBoard();
public:
    virtual void Initialize() override;
    virtual void StartNetwork() override;
    virtual Http* CreateHttp() override;
    virtual WebSocket* CreateWebSocket() override;
    virtual Mqtt* CreateMqtt() override;
    virtual Udp* CreateUdp() override;
    virtual bool GetNetworkState(std::string& network_name, int& signal_quality, std::string& signal_quality_text) override;
    virtual const char* GetNetworkStateIcon() override;
    virtual void SetPowerSaveMode(bool enabled) override;
    virtual void ResetWifiConfiguration();
};

#endif // WIFI_BOARD_H
