#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_touch_ft5x06.h>
#include <esp_log.h>
#include <esp_lvgl_port.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lvgl.h>
#include <wifi_station.h>

#include <array>

#include "application.h"
#include "audio_codecs/box_audio_codec.h"
#include "bot_controller.h"
#include "button.h"
#include "config.h"
#include "display/lcd_display.h"
#include "esp32_camera.h"
#include "i2c_device.h"
#include "iot/thing_manager.h"
#include "l298n_motor_controller.h"
#include "pca9685.h"
#include "rcwl1605.h"
#include "wifi_board.h"

#define TAG "LichuangDevBoard"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);

class Pca9557 : public I2cDevice {
   public:
    Pca9557(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
        : I2cDevice(i2c_bus, addr) {
        WriteReg(0x01, 0x03);
        WriteReg(0x03, 0xf8);
    }

    void SetOutputState(uint8_t bit, uint8_t level) {
        uint8_t data = ReadReg(0x01);
        data = (data & ~(1 << bit)) | (level << bit);
        WriteReg(0x01, data);
    }
};

class CustomAudioCodec : public BoxAudioCodec {
   private:
    Pca9557* pca9557_;

   public:
    CustomAudioCodec(i2c_master_bus_handle_t i2c_bus, Pca9557* pca9557)
        : BoxAudioCodec(i2c_bus, AUDIO_INPUT_SAMPLE_RATE,
                        AUDIO_OUTPUT_SAMPLE_RATE, AUDIO_I2S_GPIO_MCLK,
                        AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS,
                        AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN, GPIO_NUM_NC,
                        AUDIO_CODEC_ES8311_ADDR, AUDIO_CODEC_ES7210_ADDR,
                        AUDIO_INPUT_REFERENCE),
          pca9557_(pca9557) {}

    virtual void EnableOutput(bool enable) override {
        BoxAudioCodec::EnableOutput(enable);
        if (enable) {
            pca9557_->SetOutputState(1, 1);
        } else {
            pca9557_->SetOutputState(1, 0);
        }
    }
};

class LichuangDevBoard : public WifiBoard {
   private:
    i2c_master_bus_handle_t i2c_bus_;
    i2c_master_dev_handle_t pca9557_handle_;
    Button boot_button_;
    LcdDisplay* display_;
    Pca9557* pca9557_;
    Esp32Camera* camera_;
    Rcwl1605* ultrasonic_sensor_;  // RCWL-1605超声波传感器

    /** 调试 Start ***************************************************/
    // TODO 定时任务相关
    esp_timer_handle_t task_timer_;
    int current_task_index_ = 0;

    // TODO 调试用 定时任务回调函数
    static void TaskTimerCallback(void* arg) {
        LichuangDevBoard* board = static_cast<LichuangDevBoard*>(arg);
        int idx = board->current_task_index_;
        // 更新当前任务索引
        board->current_task_index_ = (board->current_task_index_ + 1) % 10;

        int angle = 10 * idx % 181;

        board->ScanI2cDevices();

        // 测试舵机控制
        Pca9685* pca9685_i2c = Pca9685::GetInstance(GPIO_I2C_PCA9685);
        Pca9685* pca9685_10 = Pca9685::GetInstance(GPIO_10_PCA9685);
        // // 使用结构体方式，让两个舵机同时运动（自动计算大小）
        // ServoControl servos[] = {
        //     {0, angle},  // 通道0
        //     {1, angle / 2}   // 通道1
        // };

        // ESP_LOGI(TAG, "同时设置通道0和1舵机角度: %d°", angle);
        // pca9685->SetServoAngles(servos);  // 自动计算大小为2
        pca9685_i2c->SetServoAngle(0, angle);
        pca9685_10->SetServoAngle(0, angle);

        // 测试电机控制
        // L298nMotorController* motor_controller =
        // L298nMotorController::GetInstance(); motor_controller->SetMotorB(idx
        // % 2 == 0 ? MotorDirection::FORWARD : MotorDirection::BACKWARD, 15 *
        // idx); motor_controller->SetMotorB(MotorDirection::FORWARD, 16);

        // 测试超声波传感器
        // board->TestUltrasonicSensor();
    }

    // TODO 调试用 定时任务回调函数
    void InitializeTaskTimer() {
        esp_timer_create_args_t timer_args = {
            .callback = TaskTimerCallback, .arg = this, .name = "task_timer"};
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &task_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(
            task_timer_, 2000000));  // 1秒 = 1,000,000微秒
    }
    /** 调试 End ***************************************************/

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags =
                {
                    .enable_internal_pullup = 1,
                },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // Initialize PCA9557
        pca9557_ = new Pca9557(i2c_bus_, 0x19);

        // Initialize I2C接口上的PCA9685 (舵机控制器) - 使用单例模式
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "初始化主I2C总线上的PCA9685");
        
        // 初始化PCA9685设备 - 先初始化0x40，再初始化0x41
        ESP_LOGI(TAG, "先初始化0x40地址PCA9685");
        Pca9685::Initialize(GPIO_10_PCA9685, i2c_bus_, 0x40);
        vTaskDelay(pdMS_TO_TICKS(200));  // 增加延迟
        
        ESP_LOGI(TAG, "再初始化0x41地址PCA9685");
        Pca9685::Initialize(GPIO_I2C_PCA9685, i2c_bus_, 0x43);

        // Initialize L298N电机控制器 - 使用单例模式
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "初始化L298N电机控制器");
        L298nMotorController::Initialize();
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_40;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_41;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz =
            DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(
            spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting &&
                !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });

#if CONFIG_USE_DEVICE_AEC
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide
                                                           : kAecOff);
            }
        });
#endif
    }

    void InitializeSt7789Display() {
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
        ESP_ERROR_CHECK(
            esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片ST7789
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(
            esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        pca9557_->SetOutputState(0, 0);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new SpiLcdDisplay(panel_io, panel, DISPLAY_WIDTH,
                                     DISPLAY_HEIGHT, DISPLAY_OFFSET_X,
                                     DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
                                     DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                     {
                                         .text_font = &font_puhui_20_4,
                                         .icon_font = &font_awesome_20_4,
#if CONFIG_USE_WECHAT_MESSAGE_STYLE
                                         .emoji_font = font_emoji_32_init(),
#else
                                         .emoji_font = font_emoji_64_init(),
#endif
                                     });
    }

    void InitializeTouch() {
        esp_lcd_touch_handle_t tp;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH,
            .y_max = DISPLAY_HEIGHT,
            .rst_gpio_num = GPIO_NUM_NC,  // Shared with LCD reset
            .int_gpio_num = GPIO_NUM_NC,
            .levels =
                {
                    .reset = 0,
                    .interrupt = 0,
                },
            .flags =
                {
                    .swap_xy = 1,
                    .mirror_x = 1,
                    .mirror_y = 0,
                },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config =
            ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
        tp_io_config.scl_speed_hz = 400000;

        esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle);
        esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp);
        assert(tp);

        /* Add touch input (for selected screen) */
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(),
            .handle = tp,
        };

        lvgl_port_add_touch(&touch_cfg);
    }

    void InitializeCamera() {
        // Open camera power
        pca9557_->SetOutputState(2, 0);

        camera_config_t config = {};
        config.ledc_channel =
            LEDC_CHANNEL_2;  // LEDC通道选择  用于生成XCLK时钟 但是S3不用
        config.ledc_timer =
            LEDC_TIMER_2;  // LEDC timer选择  用于生成XCLK时钟 但是S3不用
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
        config.pin_sccb_sda = -1;  // 这里写-1 表示使用已经初始化的I2C接口
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 1;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

        camera_ = new Esp32Camera(config);
    }

    void InitializeUltrasonicSensor() {
        // 初始化RCWL-1605超声波传感器
        ESP_LOGI(TAG, "初始化RCWL-1605超声波传感器");
        ultrasonic_sensor_ =
            new Rcwl1605(RCWL1605_TRIGGER_PIN, RCWL1605_ECHO_PIN);

        esp_err_t ret = ultrasonic_sensor_->Initialize();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "RCWL-1605超声波传感器初始化成功");
            // 设置超时时间为50ms
            ultrasonic_sensor_->SetTimeout(50000);
        } else {
            ESP_LOGE(TAG, "RCWL-1605超声波传感器初始化失败: %s",
                     esp_err_to_name(ret));
        }
    }

   public:
    LichuangDevBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeSpi();
        InitializeSt7789Display();
        InitializeTouch();
        InitializeButtons();
        InitializeCamera();
        InitializeUltrasonicSensor();  // 初始化超声波传感器
        InitializeTaskTimer();         // TODO 调试用 定时任务
        
        // 扫描I2C设备
        ScanI2cDevices();
        
        // 测试0x41地址PCA9685功能
        TestPca9685_0x41();

#if CONFIG_IOT_PROTOCOL_XIAOZHI
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
#endif
        GetBacklight()->RestoreBrightness();
        GetAudioCodec()->SetOutputVolume(1);  // 设置输出音量

        // 初始化BotController
        ESP_LOGI(TAG, "BotController初始化");
        BotController::GetInstance();

        // 验证L298N电机控制器初始化
        L298nMotorController* motor_controller =
            L298nMotorController::GetInstance();
        if (motor_controller != nullptr) {
            ESP_LOGI(TAG, "L298N电机控制器初始化成功");
        } else {
            ESP_LOGE(TAG, "L298N电机控制器初始化失败");
        }
    }

    virtual AudioCodec* GetAudioCodec() override {
        static CustomAudioCodec audio_codec(i2c_bus_, pca9557_);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return display_; }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN,
                                      DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual Camera* GetCamera() override { return camera_; }

    // TODO 测试超声波传感器功能
    void TestUltrasonicSensor() {
        if (ultrasonic_sensor_ && ultrasonic_sensor_->IsValid()) {
            // 测试精确测量
            float distance = ultrasonic_sensor_->GetDistanceAccurate();
            ESP_LOGI(TAG, "精确测量完成: 距离 = %.2f cm", distance);
        }
    }

    // TODO debug 探测I2C设备地址
    void ScanI2cDevices() {
        ESP_LOGI(TAG, "开始扫描I2C设备...");
        ESP_LOGI(TAG, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
        
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                uint8_t address = i + j;
                
                esp_err_t ret = i2c_master_probe(i2c_bus_, address, pdMS_TO_TICKS(200));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                    ESP_LOGI(TAG, "发现I2C设备，地址: 0x%02x", address);
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\n");
        }
        ESP_LOGI(TAG, "I2C设备扫描完成");
    }
    
    // 测试0x41地址PCA9685功能
    void TestPca9685_0x41() {
        ESP_LOGI(TAG, "开始测试0x41地址PCA9685功能...");
        
        // 获取0x41地址的PCA9685实例
        Pca9685* pca9685_0x41 = Pca9685::GetInstance(GPIO_I2C_PCA9685);
        if (pca9685_0x41 == nullptr) {
            ESP_LOGE(TAG, "0x41地址PCA9685实例获取失败");
            return;
        }
        
        ESP_LOGI(TAG, "0x41地址PCA9685实例获取成功");
        
        // 测试设备状态
        if (!pca9685_0x41->IsDeviceReady()) {
            ESP_LOGE(TAG, "0x41地址PCA9685设备状态异常");
            return;
        }
        
        ESP_LOGI(TAG, "0x41地址PCA9685设备状态正常");
        
        // 测试PWM输出
        ESP_LOGI(TAG, "测试通道10-15的PWM输出...");
        for (int channel = 10; channel <= 15; channel++) {
            ESP_LOGI(TAG, "设置通道%d为50%%占空比", channel);
            pca9685_0x41->SetPWM(channel, 0, 2048);  // 50%占空比
            vTaskDelay(pdMS_TO_TICKS(500));
            
            ESP_LOGI(TAG, "关闭通道%d", channel);
            pca9685_0x41->SetPWM(channel, 0, 0);     // 关闭
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        ESP_LOGI(TAG, "0x41地址PCA9685功能测试完成");
    }
};

DECLARE_BOARD(LichuangDevBoard);
