#include "wifi_board.h"
#include "audio_codecs/box_audio_codec.h"
#include "display/st7789_display.h"
#include "application.h"
#include "button.h"
#include "led.h"
#include "config.h"
#include "i2c_device.h"
#include "esp_check.h"
#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <wifi_station.h>

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#include "file_server.h"

#include "esp_camera.h"

#define TAG "LichuangDevBoard"


class Pca9557 : public I2cDevice {
public:
    Pca9557(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x01, 0x03);
        WriteReg(0x03, 0xf8);
    }

    void SetOutputState(uint8_t bit, uint8_t level) {
        uint8_t data = ReadReg(0x01);
        data = (data & ~(1 << bit)) | (level << bit);
        WriteReg(0x01, data);
    }

    // 控制 PCA9557_LCD_CS 引脚输出高低电平 参数0输出低电平 参数1输出高电平 
    void lcd_cs(uint8_t level)
    {
        SetOutputState(LCD_CS_GPIO, level);
    }

    // 控制 PCA9557_PA_EN 引脚输出高低电平 参数0输出低电平 参数1输出高电平 
    void pa_en(uint8_t level)
    {
        SetOutputState(PA_EN_GPIO, level);
    }

    // 控制 PCA9557_DVP_PWDN 引脚输出高低电平 参数0输出低电平 参数1输出高电平 
    void dvp_pwdn(uint8_t level)
    {
        SetOutputState(DVP_PWDN_GPIO, level);
    }

};

class Qmi8658a : public I2cDevice {
public:
    Qmi8658a(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
    }

    // 初始化qmi8658
    void init(void)
    {
        uint8_t id = ReadReg(QMI8658_WHO_AM_I); // 芯片的ID号
        while (id != 0x05)  // 判断读到的ID号是否是0x05
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);  // 延时1秒
            id = ReadReg(QMI8658_WHO_AM_I); // 读取ID号
        }
        ESP_LOGI(TAG, "QMI8658 OK!");  // 打印信息

        WriteReg(QMI8658_RESET, 0xb0);  // 复位  
        vTaskDelay(10 / portTICK_PERIOD_MS);  // 延时10ms
        WriteReg(QMI8658_CTRL1, 0x40); // CTRL1 设置地址自动增加
        WriteReg(QMI8658_CTRL7, 0x03); // CTRL7 允许加速度和陀螺仪
        WriteReg(QMI8658_CTRL2, 0x95); // CTRL2 设置ACC 4g 250Hz
        WriteReg(QMI8658_CTRL3, 0xd5); // CTRL3 设置GRY 512dps 250Hz 
    }

    // 读取加速度和陀螺仪寄存器值
    void Read_AccAndGry(t_sQMI8658 *p)
    {
        uint8_t status, data_ready=0;
        int16_t buf[6];
        status = ReadReg(QMI8658_STATUS0);// 读状态寄存器  
        if (status & 0x03) // 判断加速度和陀螺仪数据是否可读
            data_ready = 1;
        if (data_ready == 1){  // 如果数据可读
            data_ready = 0;
            ReadBuff(QMI8658_AX_L, (uint8_t *)buf, 12); // 读加速度和陀螺仪值
            p->acc_x = buf[0];
            p->acc_y = buf[1];
            p->acc_z = buf[2];
            p->gyr_x = buf[3];
            p->gyr_y = buf[4];
            p->gyr_z = buf[5];
        }
    }
};


class Camera {
private:
    Pca9557 *pca9557_ = NULL;
    EventGroupHandle_t event_group_ = nullptr;
    QueueHandle_t xFrame = NULL; 
public:
    Camera(Pca9557 *pca9557){
        pca9557_ = pca9557;
        xFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    }
    // 摄像头硬件初始化
    void init(void)
    {
        camera_config_t config;
        config.ledc_channel = LEDC_CHANNEL_1;  // LEDC通道选择  用于生成XCLK时钟 但是S3不用
        config.ledc_timer = LEDC_TIMER_1; // LEDC timer选择  用于生成XCLK时钟 但是S3不用
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = -1;   // 这里写-1 表示使用已经初始化的I2C接口
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = I2C_PORT_NUM; // 0
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 12;
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

        // camera init
        esp_err_t err = esp_camera_init(&config); // 配置上面定义的参数
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
            return;
        }

        sensor_t *s = esp_camera_sensor_get(); // 获取摄像头型号

        if (s->id.PID == GC0308_PID) {
            s->set_hmirror(s, 1);  // 这里控制摄像头镜像 写1镜像 写0不镜像
        }
    }

    void open_camera() {
        pca9557_->dvp_pwdn(0);
        init();
    }

    void close_camera() {
        esp_camera_deinit();
        pca9557_->dvp_pwdn(1);
    }
    // 摄像头处理任务
    void task_process_camera(void *arg)
    {
        while (true)
        {
            camera_fb_t *frame = esp_camera_fb_get();
            if (frame)
                xQueueSend(xFrame, &frame, portMAX_DELAY);
        }
    }

    camera_fb_t *get_one_frame()
    {
        camera_fb_t *frame = NULL;

        while (true)
        {
            if (xQueueReceive(xFrame, &frame, portMAX_DELAY))
            {
                //lcd_draw_bitmap(0, 0, frame->width, frame->height, (uint16_t *)frame->buf);
                esp_camera_fb_return(frame);
            }
        }
    }

};

class LichuangDevBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    i2c_master_dev_handle_t pca9557_handle_;
    Button boot_button_;
    St7789Display* display_;
    Pca9557* pca9557_;
    Qmi8658a* qmi8658a_;
    sdmmc_card_t *card_;
    Camera *camera_;

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t) I2C_PORT_NUM,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // Initialize PCA9557
        pca9557_ = new Pca9557(i2c_bus_, 0x19);
        qmi8658a_ = new Qmi8658a(i2c_bus_, 0x6a);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_40;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_41;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
        ESP_LOGD(TAG, "Initialize SPI bus");
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetChatState() == kChatStateUnknown && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
        });
        boot_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });
        boot_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });
    }

    esp_err_t InitializeSt7789Display() {

        /* 初始化LVGL */
        lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
        lvgl_port_init(&lvgl_cfg);
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_NC;
        io_config.dc_gpio_num = GPIO_NUM_39;
        io_config.spi_mode = 2;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片ST7789
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);
        pca9557_->SetOutputState(0, 0);
        //pca9557_->lcd_cs(0);  // 拉低CS引脚

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new St7789Display(i2c_bus_, panel_io, panel, DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);

        return ESP_OK;
    }

    void InitializeSCcard() {
        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = false,   // 如果挂载不成功是否需要格式化SD卡
            .max_files = 5, // 允许打开的最大文件数
            .allocation_unit_size = 16 * 1024  // 分配单元大小
        };
        
        const char mount_point[] = MOUNT_POINT;
        ESP_LOGI(TAG, "Initializing SD card");
        ESP_LOGI(TAG, "Using SDMMC peripheral");

        sdmmc_host_t host = SDMMC_HOST_DEFAULT(); // SDMMC主机接口配置
        sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT(); // SDMMC插槽配置
        slot_config.width = 1;  // 设置为1线SD模式
        slot_config.clk = BSP_SD_CLK; 
        slot_config.cmd = BSP_SD_CMD;
        slot_config.d0 = BSP_SD_D0;
        slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP; // 打开内部上拉电阻

        ESP_LOGI(TAG, "Mounting filesystem");
        esp_err_t ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card_); // 挂载SD卡

        if (ret != ESP_OK) {  // 如果没有挂载成功
            if (ret == ESP_FAIL) { // 如果挂载失败
                ESP_LOGE(TAG, "Failed to mount filesystem. ");
            } else { // 如果是其它错误 打印错误名称
                ESP_LOGE(TAG, "Failed to initialize the card (%s). ", esp_err_to_name(ret));
            }
            return;
        }
        ESP_LOGI(TAG, "Filesystem mounted"); // 提示挂载成功
        sdmmc_card_print_info(stdout, card_); // 终端打印SD卡的一些信息
    }

    void InitalizaCamera() {
        camera_ = new Camera(pca9557_);
        camera_->init();
    }
public:
    LichuangDevBoard() : boot_button_(BOOT_BUTTON_GPIO) {
    }

    virtual void Initialize() override {
        ESP_LOGI(TAG, "Initializing LichuangDevBoard");
        InitializeI2c();
        InitializeSpi();
        InitializeSt7789Display();
        InitializeButtons();
        InitializeSCcard();
        WifiBoard::Initialize();
    }

    virtual Led* GetBuiltinLed() override {
        static Led led(GPIO_NUM_NC);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec* audio_codec = nullptr;
        if (audio_codec == nullptr) {
            audio_codec = new BoxAudioCodec(i2c_bus_, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
                GPIO_NUM_NC, AUDIO_CODEC_ES8311_ADDR, AUDIO_CODEC_ES7210_ADDR, AUDIO_INPUT_REFERENCE);
            audio_codec->SetOutputVolume(AUDIO_DEFAULT_OUTPUT_VOLUME);
        }
        return audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
};

DECLARE_BOARD(LichuangDevBoard);
