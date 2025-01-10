#ifndef MP3_UI_H
#define MP3_UI_H

#include <string.h>
#include "math.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_types.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch_ft5x06.h"
#include "esp_lvgl_port.h"

#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "driver/spi_master.h"
#include "driver/i2s_std.h"

#include "esp_spiffs.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#include "audio_player.h"
#include "file_iterator.h"
#include "string.h"
#include <dirent.h>
#include "display.h"
#include "board.h"

// 按钮样式相关定义
typedef struct {
    lv_style_t style_bg;
    lv_style_t style_focus_no_outline;
} button_style_t;

class Mp3Ui {
public:
    Mp3Ui(lv_obj_t* container) {
        container_ = container;
        g_sys_volume = 10;
    }
    ~Mp3Ui() {

    }
    button_style_t g_btn_styles;
    audio_player_config_t player_config = {0};

    file_iterator_instance_t *file_iterator = NULL;

    static uint8_t g_sys_volume;         // 静态音量变量
    static lv_obj_t *music_list;         // 静态音乐列表对象

    lv_obj_t *label_play_pause;
    lv_obj_t *btn_play_pause;
    lv_obj_t *volume_slider;
    lv_obj_t* container_;
    void mp3_player_init(void);
    void music_ui(void);
    static void play_index(int index);
    // 设置声音处理函数
    static esp_err_t _audio_player_mute_fn(AUDIO_PLAYER_MUTE_SETTING setting);

    // 播放音乐函数 播放音乐的时候 会不断进入
    static esp_err_t _audio_player_write_fn(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms);

    // 设置采样率 播放的时候进入一次
    static esp_err_t _audio_player_std_clock(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch);

    // 回调函数 播放器每次动作都会进入
    static void _audio_player_callback(audio_player_cb_ctx_t *ctx);

    button_style_t *ui_button_styles(void)
    {
        return &g_btn_styles;
    }

    // 按钮样式初始化
    void ui_button_style_init(void);
    // 播放暂停按钮 事件处理函数
    static void btn_play_pause_cb(lv_event_t *event);
    // 上一首 下一首 按键事件处理函数
    static void btn_prev_next_cb(lv_event_t *event);
    // 音量调节滑动条 事件处理函数
    static void volume_slider_cb(lv_event_t *event);
    // 音乐列表 点击事件处理函数
    static void music_list_cb(lv_event_t *event);
    // 音乐名称加入列表
    static void build_file_list(lv_obj_t *music_list);

    void ai_gui_in(void);
    void ai_gui_out(void);

    void ai_play(void);
    void ai_pause(void);
    void ai_resume(void);
    void ai_prev_music(void);
    void ai_next_music(void);
    void ai_volume_up(void);
    void ai_volume_down(void);
};

#endif
