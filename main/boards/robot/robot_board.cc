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

#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <wifi_station.h>

#include "application.h"
#include "assets/lang_config.h"
#include "audio_codecs/no_audio_codec.h"
#include "button.h"
#include "config.h"
#include "display/oled_display.h"
#include "iot/thing_manager.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "system_reset.h"
#include "wifi_board.h"

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
    // SSD1306显示屏相关变量，实用IIC通信
    i2c_master_bus_handle_t
        display_i2c_bus_;  // I2C总线句柄，用于连接SSD1306显示屏
    esp_lcd_panel_io_handle_t panel_io_ =
        nullptr;                              // LCD面板IO句柄，用于与显示屏通信
    esp_lcd_panel_handle_t panel_ = nullptr;  // LCD面板句柄，控制显示屏操作
    Display* display_ = nullptr;              // 显示对象指针，提供显示接口

    /**
     * @brief 初始化显示屏I2C总线
     *
     * 配置I2C总线参数，包括GPIO引脚、时钟频率等
     * 为SSD1306显示屏提供通信通道
     */
    void InitializeDisplayI2c() {
        // I2C总线配置
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,          // 使用I2C0端口
            .sda_io_num = DISPLAY_SDA_PIN,      // SDA数据线引脚
            .scl_io_num = DISPLAY_SCL_PIN,      // SCL时钟线引脚
            .clk_source = I2C_CLK_SRC_DEFAULT,  // 默认时钟源
            .glitch_ignore_cnt = 7,             // 抗干扰计数
            .intr_priority = 0,                 // 中断优先级
            .trans_queue_depth = 0,             // 传输队列深度
            .flags =
                {
                    .enable_internal_pullup = 1,  // 启用内部上拉电阻
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
            .dev_addr = DISPLAY_I2C_ADDR,    // 设备I2C地址
            .on_color_trans_done = nullptr,  // 传输完成回调（未使用）
            .user_ctx = nullptr,             // 用户上下文（未使用）
            .control_phase_bytes = 1,        // 控制阶段字节数
            .dc_bit_offset = 6,              // 数据/命令位偏移
            .lcd_cmd_bits = 8,               // LCD命令位数
            .lcd_param_bits = 8,             // LCD参数位数
            .flags =
                {
                    .dc_low_on_data = 0,         // 数据时DC为低电平
                    .disable_control_phase = 0,  // 不禁用控制阶段
                },
            .scl_speed_hz = 400 * 1000,  // I2C时钟频率400kHz
        };

        // 创建I2C面板IO对象
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(display_i2c_bus_,
                                                    &io_config, &panel_io_));

        ESP_LOGI(TAG, "安装SSD1306驱动");

        // 面板设备配置
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = -1;  // 不使用复位引脚
        panel_config.bits_per_pixel = 1;   // 1位像素（黑白显示）

        // SSD1306特定配置
        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT),  // 显示屏高度
        };
        panel_config.vendor_config = &ssd1306_config;

        // 创建SSD1306面板对象
        ESP_ERROR_CHECK(
            esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
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
        display_ = new OledDisplay(
            panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_MIRROR_X,
            DISPLAY_MIRROR_Y, {&font_puhui_14_1, &font_awesome_14_1});
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecDuplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN);
        return &audio_codec;
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
    RobotBoard() {
        ESP_LOGI(TAG, "=== 初始化 RobotBoard 开发板 ===");
        ESP_LOGI(TAG, "板子类型: RobotBoard");
        ESP_LOGI(TAG, "目标芯片: ESP32-S3");
        ESP_LOGI(TAG, "显示屏: SSD1306 128x64 OLED");
        ESP_LOGI(TAG, "音频编解码器: USB");
        ESP_LOGI(TAG, "=====================================");

        // 按顺序初始化各个硬件模块
        InitializeDisplayI2c();      // 初始化显示屏I2C总线
        InitializeSsd1306Display();  // 初始化SSD1306显示屏
    }

    /**
     * @brief 获取显示对象
     * @return 返回显示对象指针
     *
     * 提供OLED显示屏的访问接口，支持文本显示和状态更新
     */
    virtual Display* GetDisplay() override { return display_; }
};

// 声明RobotBoard为当前开发板类型
DECLARE_BOARD(RobotBoard);
