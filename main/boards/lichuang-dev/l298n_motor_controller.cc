#include "l298n_motor_controller.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "L298nMotor"

// 初始化静态成员变量
L298nMotorController* L298nMotorController::instance_ = nullptr;

L298nMotorController::L298nMotorController()
    : motor_a_status_(0, MotorDirection::STOP, 0),
      motor_b_status_(1, MotorDirection::STOP, 0) {
    ESP_LOGI(TAG, "L298N电机控制器初始化...");
    
    // 获取PCA9685实例
    pca9685_ = Pca9685::GetInstance();
    if (pca9685_ == nullptr) {
        ESP_LOGE(TAG, "PCA9685实例未初始化，请先初始化PCA9685");
        return;
    }
    
    // 初始化所有通道为低电平
    pca9685_->SetPWM(MOTOR_A_ENABLE, 0, 0);
    pca9685_->SetPWM(MOTOR_A_IN1, 0, 0);
    pca9685_->SetPWM(MOTOR_A_IN2, 0, 0);
    pca9685_->SetPWM(MOTOR_B_ENABLE, 0, 0);
    pca9685_->SetPWM(MOTOR_B_IN3, 0, 0);
    pca9685_->SetPWM(MOTOR_B_IN4, 0, 0);
    
    ESP_LOGI(TAG, "L298N电机控制器初始化完成");
}

L298nMotorController::~L298nMotorController() {
    // 停止所有电机
    StopAllMotors();
}

L298nMotorController* L298nMotorController::GetInstance() {
    if (instance_ == nullptr) {
        ESP_LOGE(TAG, "L298N电机控制器实例未初始化，请先调用Initialize()");
        return nullptr;
    }
    return instance_;
}

void L298nMotorController::Initialize() {
    if (instance_ != nullptr) {
        ESP_LOGW(TAG, "L298N电机控制器实例已存在，销毁旧实例");
        delete instance_;
    }
    
    instance_ = new L298nMotorController();
    if (instance_ != nullptr) {
        ESP_LOGI(TAG, "L298N电机控制器单例实例创建成功");
    } else {
        ESP_LOGE(TAG, "L298N电机控制器单例实例创建失败");
    }
}

void L298nMotorController::SetMotorA(MotorDirection direction, uint8_t speed) {
    ESP_LOGI(TAG, "设置电机A: 方向=%d, 速度=%d", static_cast<int>(direction), speed);
    
    // 更新电机A状态
    motor_a_status_.direction = direction;
    motor_a_status_.speed = speed;
    
    // 设置电机A的PWM输出
    SetMotorPWM(MOTOR_A_ENABLE, MOTOR_A_IN1, MOTOR_A_IN2, direction, speed);
}

void L298nMotorController::SetMotorB(MotorDirection direction, uint8_t speed) {
    ESP_LOGI(TAG, "设置电机B: 方向=%d, 速度=%d", static_cast<int>(direction), speed);
    
    // 更新电机B状态
    motor_b_status_.direction = direction;
    motor_b_status_.speed = speed;
    
    // 设置电机B的PWM输出
    SetMotorPWM(MOTOR_B_ENABLE, MOTOR_B_IN3, MOTOR_B_IN4, direction, speed);
}

void L298nMotorController::SetMotors(const MotorControl& motor_a, const MotorControl& motor_b) {
    ESP_LOGI(TAG, "同时设置两个电机: A(方向=%d,速度=%d), B(方向=%d,速度=%d)", 
             static_cast<int>(motor_a.direction), motor_a.speed,
             static_cast<int>(motor_b.direction), motor_b.speed);
    
    // 更新状态
    motor_a_status_ = motor_a;
    motor_b_status_ = motor_b;
    
    // 设置电机A
    SetMotorPWM(MOTOR_A_ENABLE, MOTOR_A_IN1, MOTOR_A_IN2, motor_a.direction, motor_a.speed);
    
    // 设置电机B
    SetMotorPWM(MOTOR_B_ENABLE, MOTOR_B_IN3, MOTOR_B_IN4, motor_b.direction, motor_b.speed);
}

void L298nMotorController::StopMotorA() {
    ESP_LOGI(TAG, "停止电机A");
    SetMotorA(MotorDirection::STOP, 0);
}

void L298nMotorController::StopMotorB() {
    ESP_LOGI(TAG, "停止电机B");
    SetMotorB(MotorDirection::STOP, 0);
}

void L298nMotorController::StopAllMotors() {
    ESP_LOGI(TAG, "停止所有电机");
    StopMotorA();
    StopMotorB();
}

void L298nMotorController::SetMotorASpeed(uint8_t speed) {
    ESP_LOGI(TAG, "设置电机A速度: %d", speed);
    SetMotorA(motor_a_status_.direction, speed);
}

void L298nMotorController::SetMotorBSpeed(uint8_t speed) {
    ESP_LOGI(TAG, "设置电机B速度: %d", speed);
    SetMotorB(motor_b_status_.direction, speed);
}

uint16_t L298nMotorController::SpeedToPWM(uint8_t speed_percent) {
    // 限制速度范围
    if (speed_percent > SPEED_MAX) {
        speed_percent = SPEED_MAX;
    }
    
    // 将百分比转换为PWM值 (0-4095)
    uint16_t pwm_value = (speed_percent * PWM_MAX) / SPEED_MAX;
    
    ESP_LOGD(TAG, "速度转换: %d%% -> PWM值: %d", speed_percent, pwm_value);
    return pwm_value;
}

void L298nMotorController::SetMotorPWM(uint8_t enable_channel, uint8_t in1_channel, 
                                       uint8_t in2_channel, MotorDirection direction, uint8_t speed) {
    if (pca9685_ == nullptr) {
        ESP_LOGE(TAG, "PCA9685实例为空，无法设置PWM");
        return;
    }
    
    uint16_t pwm_value = SpeedToPWM(speed);
    
    switch (direction) {
        case MotorDirection::FORWARD:
            // 正转: IN1=HIGH, IN2=LOW
            ESP_LOGD(TAG, "电机正转: ENA=%d, IN1=HIGH, IN2=LOW", pwm_value);
            pca9685_->SetPWM(enable_channel, 0, pwm_value);  // 使能PWM
            pca9685_->SetPWM(in1_channel, 0, PWM_MAX);       // IN1=HIGH
            pca9685_->SetPWM(in2_channel, 0, 0);             // IN2=LOW
            break;
            
        case MotorDirection::BACKWARD:
            // 反转: IN1=LOW, IN2=HIGH
            ESP_LOGD(TAG, "电机反转: ENA=%d, IN1=LOW, IN2=HIGH", pwm_value);
            pca9685_->SetPWM(enable_channel, 0, pwm_value);  // 使能PWM
            pca9685_->SetPWM(in1_channel, 0, 0);             // IN1=LOW
            pca9685_->SetPWM(in2_channel, 0, PWM_MAX);       // IN2=HIGH
            break;
            
        case MotorDirection::STOP:
            // 停止: 所有信号为LOW
            ESP_LOGD(TAG, "电机停止: 所有信号为LOW");
            pca9685_->SetPWM(enable_channel, 0, 0);  // 禁用使能
            pca9685_->SetPWM(in1_channel, 0, 0);     // IN1=LOW
            pca9685_->SetPWM(in2_channel, 0, 0);     // IN2=LOW
            break;
            
        default:
            ESP_LOGE(TAG, "未知的电机方向: %d", static_cast<int>(direction));
            break;
    }
    
    // 添加短暂延迟确保PWM设置生效
    vTaskDelay(pdMS_TO_TICKS(10));
} 