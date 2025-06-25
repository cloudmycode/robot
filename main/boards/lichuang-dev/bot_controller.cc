/*
    Bot机器人控制器 - MCP协议版本
*/

#include "bot_controller.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "mcp_server.h"
#include "pca9685.h"

#define TAG "BotController"

// 初始化静态成员变量
BotController *BotController::instance_ = nullptr;

BotController::BotController() {
    ESP_LOGI(TAG, "开始初始化BotController...");
    RegisterMcpTools();
    ESP_LOGI(TAG, "BotController初始化完成");
}

BotController::~BotController() {}

BotController *BotController::GetInstance() {
    if (instance_ == nullptr) {
        instance_ = new BotController();
    }
    return instance_;
}

void BotController::RegisterMcpTools() {
    auto &mcp_server = McpServer::GetInstance();

    ESP_LOGI(TAG, "开始注册Bot MCP工具...");

    // 腿部履带运动控制
    mcp_server.AddTool(
        "self.electron.servo_control", "旋转舵机，angle: 角度值(0~180)",
        PropertyList({Property("angle", kPropertyTypeInteger, 0, 0, 180)}),
        [this](const PropertyList &properties) -> ReturnValue {
            int angle = properties["angle"].value<int>();
            ESP_LOGI(TAG, "舵机旋转 angle: %d", angle);

            // 使用单例模式的PCA9685控制舵机
            Pca9685 *pca9685 = Pca9685::GetInstance();
            if (pca9685) {
                // 检查设备状态
                if (!pca9685->IsDeviceReady()) {
                    ESP_LOGE(TAG, "PCA9685设备状态异常，无法控制舵机");
                    return false;
                }

                try {
                    // 使用结构体方式，让两个舵机同时运动（自动计算大小）
                    ServoControl servos[] = {
                        {0, angle},  // 通道0
                        {1, angle}   // 通道1
                    };

                    ESP_LOGI(TAG, "同时设置通道0和1舵机角度: %d°", angle);
                    pca9685->SetServoAngles(servos);  // 自动计算大小为2
                    // pca9685->SetServoAngle(0, angle);

                    ESP_LOGI(TAG, "舵机控制成功，通道0和1同时设置为 %d°",
                             angle);
                } catch (const std::exception &e) {
                    ESP_LOGE(TAG, "舵机控制异常: %s", e.what());
                } catch (...) {
                    ESP_LOGE(TAG, "舵机控制发生未知异常");
                }
            } else {
                ESP_LOGE(TAG, "无法获取PCA9685实例，请检查设备连接");
            }

            return true;
        });

    ESP_LOGI(TAG, "Bot MCP工具注册完成");
}