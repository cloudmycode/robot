#include "l298n_motor_controller.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "L298nMotor"

// 初始化静态成员变量
L298nMotorController* L298nMotorController::instance_ = nullptr;

L298nMotorController::L298nMotorController()
    : motor_a_status_(0, MotorDirection::STOP, 0),
      motor_b_status_(1, MotorDirection::STOP, 0),
      smooth_transition_task_(nullptr),
      transition_in_progress_(false),
      motor_a_transitioning_(false),
      motor_b_transitioning_(false),
      motor_a_target_direction_(MotorDirection::STOP),
      motor_a_target_speed_(0),
      motor_b_target_direction_(MotorDirection::STOP),
      motor_b_target_speed_(0) {
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
    
    // 创建平滑过渡任务
    xTaskCreate(SmoothTransitionTask, "smooth_transition", 4096, this, 5, &smooth_transition_task_);
    
    ESP_LOGI(TAG, "L298N电机控制器初始化完成");
}

L298nMotorController::~L298nMotorController() {
    // 停止所有电机
    StopAllMotors(false);
    
    // 删除平滑过渡任务
    if (smooth_transition_task_ != nullptr) {
        vTaskDelete(smooth_transition_task_);
        smooth_transition_task_ = nullptr;
    }
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

void L298nMotorController::SetMotorA(MotorDirection direction, uint8_t speed, bool smooth_transition) {
    ESP_LOGI(TAG, "设置电机A: 方向=%d, 速度=%d, 平滑过渡=%s", 
             static_cast<int>(direction), speed, smooth_transition ? "是" : "否");
    
    if (smooth_transition && smooth_config_.enable_smooth_transition) {
        // 使用平滑过渡
        motor_a_target_direction_ = direction;
        motor_a_target_speed_ = speed;
        motor_a_transitioning_ = true;
        transition_in_progress_ = true;
        ESP_LOGI(TAG, "电机A平滑过渡已启动");
    } else {
        // 直接设置
        motor_a_status_.direction = direction;
        motor_a_status_.speed = speed;
        SetMotorPWM(MOTOR_A_ENABLE, MOTOR_A_IN1, MOTOR_A_IN2, direction, speed);
    }
}

void L298nMotorController::SetMotorB(MotorDirection direction, uint8_t speed, bool smooth_transition) {
    ESP_LOGI(TAG, "设置电机B: 方向=%d, 速度=%d, 平滑过渡=%s", 
             static_cast<int>(direction), speed, smooth_transition ? "是" : "否");
    
    if (smooth_transition && smooth_config_.enable_smooth_transition) {
        // 使用平滑过渡
        motor_b_target_direction_ = direction;
        motor_b_target_speed_ = speed;
        motor_b_transitioning_ = true;
        transition_in_progress_ = true;
        ESP_LOGI(TAG, "电机B平滑过渡已启动");
    } else {
        // 直接设置
        motor_b_status_.direction = direction;
        motor_b_status_.speed = speed;
        SetMotorPWM(MOTOR_B_ENABLE, MOTOR_B_IN3, MOTOR_B_IN4, direction, speed);
    }
}

void L298nMotorController::SetMotors(const MotorControl& motor_a, const MotorControl& motor_b, bool smooth_transition) {
    ESP_LOGI(TAG, "同时设置两个电机: A(方向=%d,速度=%d), B(方向=%d,速度=%d), 平滑过渡=%s", 
             static_cast<int>(motor_a.direction), motor_a.speed,
             static_cast<int>(motor_b.direction), motor_b.speed,
             smooth_transition ? "是" : "否");
    
    if (smooth_transition && smooth_config_.enable_smooth_transition) {
        // 使用平滑过渡
        motor_a_target_direction_ = motor_a.direction;
        motor_a_target_speed_ = motor_a.speed;
        motor_b_target_direction_ = motor_b.direction;
        motor_b_target_speed_ = motor_b.speed;
        motor_a_transitioning_ = true;
        motor_b_transitioning_ = true;
        transition_in_progress_ = true;
        ESP_LOGI(TAG, "两个电机平滑过渡已启动");
    } else {
        // 直接设置
        motor_a_status_ = motor_a;
        motor_b_status_ = motor_b;
        SetMotorPWM(MOTOR_A_ENABLE, MOTOR_A_IN1, MOTOR_A_IN2, motor_a.direction, motor_a.speed);
        SetMotorPWM(MOTOR_B_ENABLE, MOTOR_B_IN3, MOTOR_B_IN4, motor_b.direction, motor_b.speed);
    }
}

void L298nMotorController::StopMotorA(bool smooth_stop) {
    ESP_LOGI(TAG, "停止电机A, 平滑停止=%s", smooth_stop ? "是" : "否");
    SetMotorA(MotorDirection::STOP, 0, smooth_stop);
}

void L298nMotorController::StopMotorB(bool smooth_stop) {
    ESP_LOGI(TAG, "停止电机B, 平滑停止=%s", smooth_stop ? "是" : "否");
    SetMotorB(MotorDirection::STOP, 0, smooth_stop);
}

void L298nMotorController::StopAllMotors(bool smooth_stop) {
    ESP_LOGI(TAG, "停止所有电机, 平滑停止=%s", smooth_stop ? "是" : "否");
    StopMotorA(smooth_stop);
    StopMotorB(smooth_stop);
}

void L298nMotorController::SetMotorASpeed(uint8_t speed, bool smooth_transition) {
    ESP_LOGI(TAG, "设置电机A速度: %d, 平滑过渡=%s", speed, smooth_transition ? "是" : "否");
    SetMotorA(motor_a_status_.direction, speed, smooth_transition);
}

void L298nMotorController::SetMotorBSpeed(uint8_t speed, bool smooth_transition) {
    ESP_LOGI(TAG, "设置电机B速度: %d, 平滑过渡=%s", speed, smooth_transition ? "是" : "否");
    SetMotorB(motor_b_status_.direction, speed, smooth_transition);
}

void L298nMotorController::SetSmoothTransitionConfig(const SmoothTransitionConfig& config) {
    smooth_config_ = config;
    ESP_LOGI(TAG, "平滑过渡配置已更新: 过渡时间=%dms, 步进间隔=%dms, 最小速度=%d, 启用=%s",
             config.ramp_time_ms, config.step_interval_ms, config.min_speed,
             config.enable_smooth_transition ? "是" : "否");
}

bool L298nMotorController::WaitMotorATransition(uint32_t timeout_ms) {
    uint32_t start_time = xTaskGetTickCount();
    while (motor_a_transitioning_) {
        if ((xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS > timeout_ms) {
            ESP_LOGW(TAG, "等待电机A过渡超时");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return true;
}

bool L298nMotorController::WaitMotorBTransition(uint32_t timeout_ms) {
    uint32_t start_time = xTaskGetTickCount();
    while (motor_b_transitioning_) {
        if ((xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS > timeout_ms) {
            ESP_LOGW(TAG, "等待电机B过渡超时");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return true;
}

bool L298nMotorController::WaitAllMotorsTransition(uint32_t timeout_ms) {
    uint32_t start_time = xTaskGetTickCount();
    while (motor_a_transitioning_ || motor_b_transitioning_) {
        if ((xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS > timeout_ms) {
            ESP_LOGW(TAG, "等待所有电机过渡超时");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return true;
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

void L298nMotorController::SetMotorPWM(uint8_t enable_channel, uint8_t in1_channel, uint8_t in2_channel,
                                       MotorDirection direction, uint8_t speed) {
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

void L298nMotorController::SmoothTransitionTask(void* parameter) {
    L298nMotorController* controller = static_cast<L298nMotorController*>(parameter);
    
    while (true) {
        if (controller->transition_in_progress_) {
            // 处理电机A的平滑过渡
            if (controller->motor_a_transitioning_) {
                controller->ExecuteMotorASmoothTransition(
                    controller->motor_a_target_direction_, 
                    controller->motor_a_target_speed_
                );
            }
            
            // 处理电机B的平滑过渡
            if (controller->motor_b_transitioning_) {
                controller->ExecuteMotorBSmoothTransition(
                    controller->motor_b_target_direction_, 
                    controller->motor_b_target_speed_
                );
            }
            
            // 检查是否所有过渡都完成
            if (!controller->motor_a_transitioning_ && !controller->motor_b_transitioning_) {
                controller->transition_in_progress_ = false;
                ESP_LOGI(TAG, "所有电机平滑过渡完成");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(controller->smooth_config_.step_interval_ms));
    }
}

void L298nMotorController::ExecuteMotorASmoothTransition(MotorDirection target_direction, uint8_t target_speed) {
    MotorDirection current_direction = motor_a_status_.direction;
    uint8_t current_speed = motor_a_status_.speed;
    
    // 如果当前电机正在运行且需要改变状态（包括停止或速度变化），先减速到停止
    if (current_direction != MotorDirection::STOP && 
        (current_direction != target_direction || target_direction == MotorDirection::STOP || 
         current_speed != target_speed)) {
        ESP_LOGI(TAG, "电机A状态改变，先减速到停止");
        
        // 减速到停止
        for (int speed = current_speed; speed >= smooth_config_.min_speed; speed -= 2) {
            SetMotorPWM(MOTOR_A_ENABLE, MOTOR_A_IN1, MOTOR_A_IN2, current_direction, speed);
            motor_a_status_.speed = speed;
            vTaskDelay(pdMS_TO_TICKS(smooth_config_.step_interval_ms));
        }
        
        // 完全停止
        SetMotorPWM(MOTOR_A_ENABLE, MOTOR_A_IN1, MOTOR_A_IN2, MotorDirection::STOP, 0);
        motor_a_status_.direction = MotorDirection::STOP;
        motor_a_status_.speed = 0;
        vTaskDelay(pdMS_TO_TICKS(100)); // 短暂停止时间
    }
    
    // 如果目标不是停止，则加速到目标速度
    if (target_direction != MotorDirection::STOP) {
        ESP_LOGI(TAG, "电机A加速到目标状态: 方向=%d, 速度=%d", 
                 static_cast<int>(target_direction), target_speed);
        
        // 设置方向
        motor_a_status_.direction = target_direction;
        
        // 从最小速度加速到目标速度
        for (int speed = smooth_config_.min_speed; speed <= target_speed; speed += 2) {
            SetMotorPWM(MOTOR_A_ENABLE, MOTOR_A_IN1, MOTOR_A_IN2, target_direction, speed);
            motor_a_status_.speed = speed;
            vTaskDelay(pdMS_TO_TICKS(smooth_config_.step_interval_ms));
        }
        
        // 确保达到目标速度
        SetMotorPWM(MOTOR_A_ENABLE, MOTOR_A_IN1, MOTOR_A_IN2, target_direction, target_speed);
        motor_a_status_.speed = target_speed;
    } else {
        // 目标为停止，确保完全停止（已经在上面处理过了）
        ESP_LOGI(TAG, "电机A已平滑停止");
    }
    
    motor_a_transitioning_ = false;
    ESP_LOGI(TAG, "电机A平滑过渡完成: 方向=%d, 速度=%d", 
             static_cast<int>(motor_a_status_.direction), motor_a_status_.speed);
}

void L298nMotorController::ExecuteMotorBSmoothTransition(MotorDirection target_direction, uint8_t target_speed) {
    MotorDirection current_direction = motor_b_status_.direction;
    uint8_t current_speed = motor_b_status_.speed;
    
    // 如果当前电机正在运行且需要改变状态（包括停止或速度变化），先减速到停止
    if (current_direction != MotorDirection::STOP && 
        (current_direction != target_direction || target_direction == MotorDirection::STOP || 
         current_speed != target_speed)) {
        ESP_LOGI(TAG, "电机B状态改变，先减速到停止");
        
        // 减速到停止
        for (int speed = current_speed; speed >= smooth_config_.min_speed; speed -= 2) {
            SetMotorPWM(MOTOR_B_ENABLE, MOTOR_B_IN3, MOTOR_B_IN4, current_direction, speed);
            motor_b_status_.speed = speed;
            vTaskDelay(pdMS_TO_TICKS(smooth_config_.step_interval_ms));
        }
        
        // 完全停止
        SetMotorPWM(MOTOR_B_ENABLE, MOTOR_B_IN3, MOTOR_B_IN4, MotorDirection::STOP, 0);
        motor_b_status_.direction = MotorDirection::STOP;
        motor_b_status_.speed = 0;
        vTaskDelay(pdMS_TO_TICKS(100)); // 短暂停止时间
    }
    
    // 如果目标不是停止，则加速到目标速度
    if (target_direction != MotorDirection::STOP) {
        ESP_LOGI(TAG, "电机B加速到目标状态: 方向=%d, 速度=%d", 
                 static_cast<int>(target_direction), target_speed);
        
        // 设置方向
        motor_b_status_.direction = target_direction;
        
        // 从最小速度加速到目标速度
        for (int speed = smooth_config_.min_speed; speed <= target_speed; speed += 2) {
            SetMotorPWM(MOTOR_B_ENABLE, MOTOR_B_IN3, MOTOR_B_IN4, target_direction, speed);
            motor_b_status_.speed = speed;
            vTaskDelay(pdMS_TO_TICKS(smooth_config_.step_interval_ms));
        }
        
        // 确保达到目标速度
        SetMotorPWM(MOTOR_B_ENABLE, MOTOR_B_IN3, MOTOR_B_IN4, target_direction, target_speed);
        motor_b_status_.speed = target_speed;
    } else {
        // 目标为停止，确保完全停止（已经在上面处理过了）
        ESP_LOGI(TAG, "电机B已平滑停止");
    }
    
    motor_b_transitioning_ = false;
    ESP_LOGI(TAG, "电机B平滑过渡完成: 方向=%d, 速度=%d", 
             static_cast<int>(motor_b_status_.direction), motor_b_status_.speed);
}

void L298nMotorController::ExecuteSmoothStop(uint8_t motor_id) {
    if (motor_id == 0) {
        // 电机A平滑停止
        uint8_t current_speed = motor_a_status_.speed;
        MotorDirection current_direction = motor_a_status_.direction;
        
        for (int speed = current_speed; speed >= smooth_config_.min_speed; speed -= 2) {
            SetMotorPWM(MOTOR_A_ENABLE, MOTOR_A_IN1, MOTOR_A_IN2, current_direction, speed);
            motor_a_status_.speed = speed;
            vTaskDelay(pdMS_TO_TICKS(smooth_config_.step_interval_ms));
        }
        
        SetMotorPWM(MOTOR_A_ENABLE, MOTOR_A_IN1, MOTOR_A_IN2, MotorDirection::STOP, 0);
        motor_a_status_.direction = MotorDirection::STOP;
        motor_a_status_.speed = 0;
    } else if (motor_id == 1) {
        // 电机B平滑停止
        uint8_t current_speed = motor_b_status_.speed;
        MotorDirection current_direction = motor_b_status_.direction;
        
        for (int speed = current_speed; speed >= smooth_config_.min_speed; speed -= 2) {
            SetMotorPWM(MOTOR_B_ENABLE, MOTOR_B_IN3, MOTOR_B_IN4, current_direction, speed);
            motor_b_status_.speed = speed;
            vTaskDelay(pdMS_TO_TICKS(smooth_config_.step_interval_ms));
        }
        
        SetMotorPWM(MOTOR_B_ENABLE, MOTOR_B_IN3, MOTOR_B_IN4, MotorDirection::STOP, 0);
        motor_b_status_.direction = MotorDirection::STOP;
        motor_b_status_.speed = 0;
    }
} 