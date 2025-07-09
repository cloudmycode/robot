/*
 * @Author: wangyong
 * @Date: 2025-07-05 10:00:00
 * @LastEditors: wangyong
 * @LastEditTime: 2025-07-05 10:00:00
 * @Description: RobotBoard开发板实现
 * 
 * Copyright (c) 2025 by wangyong, All Rights Reserved. 
 * 
 * 硬件配置：
 * - 显示屏：SSD1306 128x64 OLED （型号MCP23071）
 * - 音频/视频/mic：USB音频流
 * - 主控芯片：ESP32-S3
 * - 按钮：BOOT按钮、触摸按钮、音量加减按钮
 * - LED：板载LED指示灯
 * 
 * 功能特性：
 * - WiFi连接管理
 * - OLED显示界面
 * - 音频编解码处理
 * - 按钮交互控制
 * - 物联网设备管理
 * - LED状态指示
*/

#include "wifi_board.h"
#include "audio_codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "iot/thing_manager.h"
#include "led/single_led.h"
#include "assets/lang_config.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

// 日志标签
#define TAG "RobotBoard"

// 声明字体资源
LV_FONT_DECLARE(font_puhui_14_1);    // 中文字体
LV_FONT_DECLARE(font_awesome_14_1);  // 图标字体

/**
 * @brief RobotBoard开发板类
 * 
 * 继承自WifiBoard，实现机器人开发板的硬件抽象层
 * 负责管理显示屏、按钮、音频、LED等硬件资源
 */
class RobotBoard : public WifiBoard {
private:
    // I2C总线句柄，用于连接SSD1306显示屏
    i2c_master_bus_handle_t display_i2c_bus_;
    
    // LCD面板IO句柄，用于与显示屏通信
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    
    // LCD面板句柄，控制显示屏操作
    esp_lcd_panel_handle_t panel_ = nullptr;
    
    // 显示对象指针，提供显示接口
    Display* display_ = nullptr;
    
    // 按钮对象
    Button boot_button_;           // BOOT按钮，用于系统重置和聊天状态切换
    Button touch_button_;          // 触摸按钮，用于语音输入控制
    Button volume_up_button_;      // 音量增加按钮
    Button volume_down_button_;    // 音量减少按钮

    /**
     * @brief 初始化显示屏I2C总线
     * 
     * 配置I2C总线参数，包括GPIO引脚、时钟频率等
     * 为SSD1306显示屏提供通信通道
     */
    void InitializeDisplayI2c() {
        // I2C总线配置
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,                    // 使用I2C0端口
            .sda_io_num = DISPLAY_SDA_PIN,                // SDA数据线引脚
            .scl_io_num = DISPLAY_SCL_PIN,                // SCL时钟线引脚
            .clk_source = I2C_CLK_SRC_DEFAULT,            // 默认时钟源
            .glitch_ignore_cnt = 7,                       // 抗干扰计数
            .intr_priority = 0,                           // 中断优先级
            .trans_queue_depth = 0,                       // 传输队列深度
            .flags = {
                .enable_internal_pullup = 1,              // 启用内部上拉电阻
            },
        };
        // 创建I2C主总线
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    /**
     * @brief 初始化SSD1306 OLED显示屏
     * 
     * 配置SSD1306显示屏参数，包括I2C地址、通信协议等
     * 创建显示面板对象并设置显示参数
     */
    void InitializeSsd1306Display() {
        // SSD1306 I2C通信配置
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = DISPLAY_I2C_ADDR,                 // 设备I2C地址
            .on_color_trans_done = nullptr,               // 传输完成回调（未使用）
            .user_ctx = nullptr,                          // 用户上下文（未使用）
            .control_phase_bytes = 1,                     // 控制阶段字节数
            .dc_bit_offset = 6,                           // 数据/命令位偏移
            .lcd_cmd_bits = 8,                            // LCD命令位数
            .lcd_param_bits = 8,                          // LCD参数位数
            .flags = {
                .dc_low_on_data = 0,                      // 数据时DC为低电平
                .disable_control_phase = 0,               // 不禁用控制阶段
            },
            .scl_speed_hz = 400 * 1000,                   // I2C时钟频率400kHz
        };

        // 创建I2C面板IO对象
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(display_i2c_bus_, &io_config, &panel_io_));

        ESP_LOGI(TAG, "安装SSD1306驱动");
        
        // 面板设备配置
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = -1;                 // 不使用复位引脚
        panel_config.bits_per_pixel = 1;                  // 1位像素（黑白显示）

        // SSD1306特定配置
        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT), // 显示屏高度
        };
        panel_config.vendor_config = &ssd1306_config;

        // 创建SSD1306面板对象
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
        ESP_LOGI(TAG, "SSD1306驱动安装完成");

        // 复位显示屏
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        
        // 初始化显示屏
        if (esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "显示屏初始化失败");
            display_ = new NoDisplay();  // 使用空显示对象作为后备
            return;
        }
        
        // 设置显示颜色（不反转）
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, false));

        // 开启显示屏
        ESP_LOGI(TAG, "开启显示屏");
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        // 创建OLED显示对象，传入字体资源
        display_ = new OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
            {&font_puhui_14_1, &font_awesome_14_1});
    }

    /**
     * @brief 初始化按钮功能
     * 
     * 为各个按钮设置点击、长按等事件处理函数
     * 实现音量控制、语音输入、系统重置等功能
     */
    void InitializeButtons() {
        // BOOT按钮事件处理
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            // 如果设备正在启动且WiFi未连接，则重置WiFi配置
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            // 切换聊天状态
            app.ToggleChatState();
        });
        
        // 触摸按钮事件处理（语音输入控制）
        touch_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();  // 开始监听语音
        });
        touch_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();   // 停止监听语音
        });

        // 音量增加按钮事件处理
        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;    // 音量增加10
            if (volume > 100) {
                volume = 100;                             // 最大音量限制
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        // 音量增加按钮长按事件（直接设为最大音量）
        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        // 音量减少按钮事件处理
        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;    // 音量减少10
            if (volume < 0) {
                volume = 0;                               // 最小音量限制
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        // 音量减少按钮长按事件（直接设为静音）
        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

    /**
     * @brief 初始化物联网功能
     * 
     * 创建物联网设备管理器，支持智能家居设备控制
     * 目前包含LED灯控制器
     */
    void InitializeIot() {
        // 创建LED灯控制器，用于智能家居控制
        static LampController lamp(LAMP_GPIO);
    }

    /**
     * @brief 在SSD1306显示屏上显示测试文案
     * @param test_text 要显示的测试文案
     * @param duration_ms 显示持续时间（毫秒），默认3000ms
     * 
     * 这个方法会：
     * 1. 清空当前显示内容
     * 2. 显示测试文案
     * 3. 在指定时间后自动清除
     */
    void ShowTestText(const char* test_text, int duration_ms = 3000) {
        if (display_ == nullptr) {
            ESP_LOGE(TAG, "显示屏未初始化，无法显示测试文案");
            return;
        }
        
        ESP_LOGI(TAG, "显示测试文案: %s", test_text);
        
        // 使用Display类的ShowNotification方法显示测试文案
        display_->ShowNotification(test_text, duration_ms);
        
        // 同时设置状态栏显示测试信息
        display_->SetStatus("测试模式");
    }

    /**
     * @brief 显示多行测试文案
     * @param lines 文案行数组
     * @param line_count 行数
     * @param duration_ms 显示持续时间（毫秒），默认5000ms
     */
    void ShowMultiLineTestText(const char* lines[], int line_count, int duration_ms = 5000) {
        if (display_ == nullptr) {
            ESP_LOGE(TAG, "显示屏未初始化，无法显示测试文案");
            return;
        }
        
        ESP_LOGI(TAG, "显示多行测试文案，共%d行", line_count);
        
        // 组合多行文案
        std::string combined_text;
        for (int i = 0; i < line_count; i++) {
            if (i > 0) combined_text += "\n";
            combined_text += lines[i];
        }
        
        // 显示组合后的文案
        display_->ShowNotification(combined_text, duration_ms);
        display_->SetStatus("多行测试");
    }

public:
    /**
     * @brief RobotBoard构造函数
     * 
     * 初始化开发板硬件，包括：
     * - 显示屏I2C总线和SSD1306驱动
     * - 按钮GPIO配置和事件处理
     * - 物联网设备管理
     * - 显示测试文案
     */
    RobotBoard() :
        boot_button_(BOOT_BUTTON_GPIO),           // 初始化BOOT按钮
        touch_button_(TOUCH_BUTTON_GPIO),         // 初始化触摸按钮
        volume_up_button_(VOLUME_UP_BUTTON_GPIO), // 初始化音量增加按钮
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) { // 初始化音量减少按钮
        
        ESP_LOGI(TAG, "=== 初始化 RobotBoard 开发板 ===");
        ESP_LOGI(TAG, "板子类型: RobotBoard");
        ESP_LOGI(TAG, "目标芯片: ESP32-S3");
        ESP_LOGI(TAG, "显示屏: SSD1306 128x64 OLED");
        ESP_LOGI(TAG, "音频编解码器: NoAudioCodec");
        ESP_LOGI(TAG, "LED: SingleLed on GPIO %d", BUILTIN_LED_GPIO);
        ESP_LOGI(TAG, "=====================================");
        
        // 按顺序初始化各个硬件模块
        InitializeDisplayI2c();      // 初始化显示屏I2C总线
        InitializeSsd1306Display();  // 初始化SSD1306显示屏
        InitializeButtons();         // 初始化按钮功能
        InitializeIot();             // 初始化物联网功能

        // 显示测试文案，验证显示屏功能
        ShowTestText("Hello World!", 3000);
        ShowMultiLineTestText(new const char*[] {"Hello", "World", "Test"}, 3, 5000);
    }

    /**
     * @brief 获取LED对象
     * @return 返回单LED控制对象指针
     * 
     * 提供板载LED的访问接口，用于状态指示
     */
    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    /**
     * @brief 获取音频编解码器对象
     * @return 返回音频编解码器对象指针
     * 
     * 根据编译配置选择单工或双工音频编解码器
     * 支持音频输入输出处理
     */
    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        // 单工模式：扬声器和麦克风使用不同的I2S接口
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, 
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        // 双工模式：扬声器和麦克风共享同一个I2S接口
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    /**
     * @brief 获取显示对象
     * @return 返回显示对象指针
     * 
     * 提供OLED显示屏的访问接口，支持文本显示和状态更新
     */
    virtual Display* GetDisplay() override {
        return display_;
    }
};

// 声明RobotBoard为当前开发板类型
DECLARE_BOARD(RobotBoard);
