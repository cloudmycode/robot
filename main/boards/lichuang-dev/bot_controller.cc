/*
    Bot机器人控制器 - MCP协议版本
*/

#include "bot_controller.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "l298n_motor_controller.h"
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
            // 使用结构体方式，让两个舵机同时运动（自动计算大小）
            ServoControl servos[] = {
                {0, angle},  // 通道0
                {1, angle}   // 通道1
            };

            ESP_LOGI(TAG, "同时设置通道0和1舵机角度: %d°", angle);
            pca9685->SetServoAngles(servos);  // 自动计算大小为2
            // pca9685->SetServoAngle(0, angle);

            ESP_LOGI(TAG, "舵机控制成功，通道0和1同时设置为 %d°", angle);

            return true;
        });

    // L298N电机A控制
    mcp_server.AddTool(
        "self.electron.motor_a_control", "控制电机A，direction: 方向(0=正转,1=反转,2=停止), speed: 速度(0~100)",
        PropertyList({
            Property("direction", kPropertyTypeInteger, 2, 0, 2),
            Property("speed", kPropertyTypeInteger, 0, 0, 100)
        }),
        [this](const PropertyList &properties) -> ReturnValue {
            int direction = properties["direction"].value<int>();
            int speed = properties["speed"].value<int>();
            ESP_LOGI(TAG, "电机A控制: 方向=%d, 速度=%d", direction, speed);

            // 获取L298N电机控制器实例
            L298nMotorController* motor_controller = L298nMotorController::GetInstance();
            if (motor_controller == nullptr) {
                ESP_LOGE(TAG, "L298N电机控制器未初始化");
                return false;
            }

            // 设置电机A
            motor_controller->SetMotorA(static_cast<MotorDirection>(direction), speed);
            ESP_LOGI(TAG, "电机A控制成功: 方向=%d, 速度=%d", direction, speed);

            return true;
        });

    // L298N电机B控制
    mcp_server.AddTool(
        "self.electron.motor_b_control", "控制电机B，direction: 方向(0=正转,1=反转,2=停止), speed: 速度(0~100)",
        PropertyList({
            Property("direction", kPropertyTypeInteger, 2, 0, 2),
            Property("speed", kPropertyTypeInteger, 0, 0, 100)
        }),
        [this](const PropertyList &properties) -> ReturnValue {
            int direction = properties["direction"].value<int>();
            int speed = properties["speed"].value<int>();
            ESP_LOGI(TAG, "电机B控制: 方向=%d, 速度=%d", direction, speed);

            // 获取L298N电机控制器实例
            L298nMotorController* motor_controller = L298nMotorController::GetInstance();
            if (motor_controller == nullptr) {
                ESP_LOGE(TAG, "L298N电机控制器未初始化");
                return false;
            }

            // 设置电机B
            motor_controller->SetMotorB(static_cast<MotorDirection>(direction), speed);
            ESP_LOGI(TAG, "电机B控制成功: 方向=%d, 速度=%d", direction, speed);

            return true;
        });

    // 同时控制两个电机
    mcp_server.AddTool(
        "self.electron.motors_control", "同时控制两个电机，motor_a_direction: 电机A方向(0=正转,1=反转,2=停止), motor_a_speed: 电机A速度(0~100), motor_b_direction: 电机B方向(0=正转,1=反转,2=停止), motor_b_speed: 电机B速度(0~100)",
        PropertyList({
            Property("motor_a_direction", kPropertyTypeInteger, 2, 0, 2),
            Property("motor_a_speed", kPropertyTypeInteger, 0, 0, 100),
            Property("motor_b_direction", kPropertyTypeInteger, 2, 0, 2),
            Property("motor_b_speed", kPropertyTypeInteger, 0, 0, 100)
        }),
        [this](const PropertyList &properties) -> ReturnValue {
            int motor_a_direction = properties["motor_a_direction"].value<int>();
            int motor_a_speed = properties["motor_a_speed"].value<int>();
            int motor_b_direction = properties["motor_b_direction"].value<int>();
            int motor_b_speed = properties["motor_b_speed"].value<int>();
            
            ESP_LOGI(TAG, "同时控制电机: A(方向=%d,速度=%d), B(方向=%d,速度=%d)", 
                     motor_a_direction, motor_a_speed, motor_b_direction, motor_b_speed);

            // 获取L298N电机控制器实例
            L298nMotorController* motor_controller = L298nMotorController::GetInstance();
            if (motor_controller == nullptr) {
                ESP_LOGE(TAG, "L298N电机控制器未初始化");
                return false;
            }

            // 创建电机控制结构体
            MotorControl motor_a(0, static_cast<MotorDirection>(motor_a_direction), motor_a_speed);
            MotorControl motor_b(1, static_cast<MotorDirection>(motor_b_direction), motor_b_speed);

            // 同时设置两个电机
            motor_controller->SetMotors(motor_a, motor_b);
            ESP_LOGI(TAG, "电机控制成功: A(方向=%d,速度=%d), B(方向=%d,速度=%d)", 
                     motor_a_direction, motor_a_speed, motor_b_direction, motor_b_speed);

            return true;
        });

    // 停止所有电机
    mcp_server.AddTool(
        "self.electron.stop_all_motors", "停止所有电机",
        PropertyList(),
        [this](const PropertyList &properties) -> ReturnValue {
            ESP_LOGI(TAG, "停止所有电机");

            // 获取L298N电机控制器实例
            L298nMotorController* motor_controller = L298nMotorController::GetInstance();
            if (motor_controller == nullptr) {
                ESP_LOGE(TAG, "L298N电机控制器未初始化");
                return false;
            }

            // 停止所有电机
            motor_controller->StopAllMotors();
            ESP_LOGI(TAG, "所有电机已停止");

            return true;
        });

    // 获取电机状态
    mcp_server.AddTool(
        "self.electron.get_motor_status", "获取电机状态",
        PropertyList(),
        [this](const PropertyList &properties) -> ReturnValue {
            // 获取L298N电机控制器实例
            L298nMotorController* motor_controller = L298nMotorController::GetInstance();
            if (motor_controller == nullptr) {
                ESP_LOGE(TAG, "L298N电机控制器未初始化");
                return "{\"error\": \"L298N电机控制器未初始化\"}";
            }

            // 获取电机状态
            MotorControl motor_a = motor_controller->GetMotorAStatus();
            MotorControl motor_b = motor_controller->GetMotorBStatus();

            // 返回JSON格式的状态信息
            char status_json[256];
            snprintf(status_json, sizeof(status_json), 
                     "{\"motor_a\": {\"direction\": %d, \"speed\": %d}, \"motor_b\": {\"direction\": %d, \"speed\": %d}}",
                     static_cast<int>(motor_a.direction), motor_a.speed,
                     static_cast<int>(motor_b.direction), motor_b.speed);

            ESP_LOGI(TAG, "电机状态: %s", status_json);
            return status_json;
        });

    ESP_LOGI(TAG, "Bot MCP工具注册完成");
}