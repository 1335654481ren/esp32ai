#ifndef ST7789_DISPLAY_H
#define ST7789_DISPLAY_H

#include "display.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include "esp_lcd_types.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_touch_ft5x06.h"
#include "esp_lvgl_port.h"
#include "mp3_ui.h"
#include <esp_timer.h>


class St7789Display : public Display {
private:
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    lv_disp_drv_t disp_drv_;
    lv_disp_t *disp_;      // 指向液晶屏
    lv_indev_t *disp_indev_ = NULL; // 指向触摸屏
    esp_lcd_touch_handle_t tp_;   // 触摸屏句柄
    gpio_num_t backlight_pin_ = GPIO_NUM_NC;
    bool backlight_output_invert_ = false;
    i2c_master_bus_handle_t i2c_bus_;
    bool mirror_x_ = false;
    bool mirror_y_ = false;
    bool swap_xy_ = false;
    uint32_t offset_x_ = 0;
    uint32_t offset_y_ = 0;
    SemaphoreHandle_t lvgl_mutex_ = nullptr;
    esp_timer_handle_t lvgl_tick_timer_ = nullptr;

    lv_obj_t* status_bar_ = nullptr;
    lv_obj_t* content_ = nullptr;
    lv_obj_t* container_ = nullptr;
    lv_obj_t* side_bar_ = nullptr;
    lv_obj_t *btn_ = nullptr;
    void InitializeBacklight(gpio_num_t backlight_pin);
    void SetBacklight(uint8_t brightness);
    void SetupUI();
    Mp3Ui* mp3ui_;
    static void btn_play_pause_cb(lv_event_t *event);
    // 显示图片
    void lcd_draw_bitmap(int x_start, int y_start, int x_end, int y_end, const void *color_data);
    esp_err_t touch_pad_ft5x06(esp_lcd_touch_handle_t *ret_touch);
    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;
 
    void lcd_set_color(uint16_t color);
    void lcd_draw_pictrue(int x_start, int y_start, int x_end, int y_end, const unsigned char *gImage);

public:
    St7789Display(i2c_master_bus_handle_t i2c_bus, esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                  gpio_num_t backlight_pin, bool backlight_output_invert,
                  uint32_t width, uint32_t height,  uint32_t offset_x, uint32_t offset_y, bool mirror_x, bool mirror_y, bool swap_xy);
    ~St7789Display();
};

#endif // ST7789_DISPLAY_H
