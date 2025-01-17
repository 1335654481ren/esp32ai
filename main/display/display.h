#ifndef DISPLAY_H
#define DISPLAY_H

#include <lvgl.h>
#include <esp_timer.h>
#include "esp_lcd_touch_ft5x06.h"
#include <string>


class Display {
public:
    Display();
    virtual ~Display();

    virtual void SetStatus(const std::string &status);
    virtual void SetTTS(const std::string &tts);
    virtual void SetSTT(const std::string &stt);
    virtual void ShowNotification(const std::string &notification, int duration_ms = 3000);
    virtual void SetEmotion(const std::string &emotion);
    virtual void SetChatMessage(const std::string &role, const std::string &content);
    virtual void SetIcon(const char* icon);

    uint32_t width() const { return width_; }
    uint32_t height() const { return height_; }

protected:
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    
    esp_lcd_touch_handle_t tp;       // 触摸屏句柄
    lv_disp_t *disp_ = nullptr;      // 指向液晶屏     
    lv_indev_t *disp_indev = NULL;   // 指向触摸屏

    lv_obj_t *emotion_label_ = nullptr;
    lv_obj_t *network_label_ = nullptr;
    lv_obj_t *status_label_ = nullptr;
    lv_obj_t *notification_label_ = nullptr;
    lv_obj_t *mute_label_ = nullptr;
    lv_obj_t *battery_label_ = nullptr;
    lv_obj_t *btn_label_ = nullptr;
    lv_obj_t *tts_label_ = nullptr;
    lv_obj_t *stt_label_ = nullptr;
    const char* battery_icon_ = nullptr;
    const char* network_icon_ = nullptr;
    bool muted_ = false;

    esp_timer_handle_t notification_timer_ = nullptr;
    esp_timer_handle_t update_timer_ = nullptr;

    friend class DisplayLockGuard;
    virtual bool Lock(int timeout_ms = 0) = 0;
    virtual void Unlock() = 0;

    virtual void Update();
};


class DisplayLockGuard {
public:
    DisplayLockGuard(Display *display) : display_(display) {
        display_->Lock();
    }
    ~DisplayLockGuard() {
        display_->Unlock();
    }

private:
    Display *display_;
};

#endif
