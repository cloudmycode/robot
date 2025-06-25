#include "pca9685.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cmath>

#define TAG "PCA9685"

// 初始化静态成员变量
Pca9685 *Pca9685::instance_ = nullptr;

Pca9685::Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
    : I2cDevice(i2c_bus, addr), pwm_frequency_(DEFAULT_PWM_FREQ) {
    ESP_LOGI(TAG, "PCA9685初始化，地址: 0x%02X", addr);

    // 检查设备是否存在
    esp_err_t ret = i2c_master_probe(i2c_bus, addr, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685设备未找到，地址: 0x%02X, 错误: %s", addr,
                 esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "PCA9685设备检测成功");
    Initialize();
}

Pca9685 *Pca9685::GetInstance() {
    if (instance_ == nullptr) {
        ESP_LOGE(TAG, "PCA9685实例未初始化，请先调用Initialize()");
        return nullptr;
    }
    return instance_;
}

void Pca9685::Initialize(i2c_master_bus_handle_t i2c_bus, uint8_t addr) {
    if (instance_ != nullptr) {
        ESP_LOGW(TAG, "PCA9685实例已存在，销毁旧实例");
        delete instance_;
    }

    instance_ = new Pca9685(i2c_bus, addr);
    if (instance_ != nullptr) {
        ESP_LOGI(TAG, "PCA9685单例实例创建成功");
    } else {
        ESP_LOGE(TAG, "PCA9685单例实例创建失败");
    }
}

void Pca9685::Initialize() {
    ESP_LOGI(TAG, "开始初始化PCA9685...");

    // 等待设备稳定
    vTaskDelay(pdMS_TO_TICKS(20));

    // 重置PCA9685 - 先读取当前状态
    uint8_t old_mode = ReadReg(MODE1);
    ESP_LOGI(TAG, "当前MODE1寄存器值: 0x%02X", old_mode);

    // 设置SLEEP位以允许修改预分频值
    WriteReg(MODE1, (old_mode & 0x7F) | 0x10);
    vTaskDelay(pdMS_TO_TICKS(5));

    // 设置25Hz PWM频率
    float prescale_val = (25000000.0f / (4096.0f * pwm_frequency_)) - 1.0f;
    uint8_t prescale = static_cast<uint8_t>(round(prescale_val));
    WriteReg(PRESCALE, prescale);
    vTaskDelay(pdMS_TO_TICKS(5));

    // 恢复MODE1寄存器
    WriteReg(MODE1, old_mode);
    vTaskDelay(pdMS_TO_TICKS(5));

    // 等待振荡器稳定
    vTaskDelay(pdMS_TO_TICKS(10));

    // 清除SLEEP位并设置AUTO_INCREMENT
    WriteReg(MODE1, old_mode | 0xA1);
    vTaskDelay(pdMS_TO_TICKS(5));

    // 验证初始化是否成功
    uint8_t new_mode = ReadReg(MODE1);
    ESP_LOGI(TAG, "初始化后MODE1寄存器值: 0x%02X", new_mode);

    ESP_LOGI(TAG, "PCA9685初始化完成，PWM频率: %.1f Hz, 预分频值: %d",
             pwm_frequency_, prescale);
}

void Pca9685::SetServoAngle(uint8_t channel, int angle) {
    if (channel > 15) {
        ESP_LOGE(TAG, "通道号超出范围 (0-15): %d", channel);
        return;
    }

    if (angle < SERVO_MIN_ANGLE || angle > SERVO_MAX_ANGLE) {
        ESP_LOGW(TAG, "角度超出范围 (%d-%d): %d", SERVO_MIN_ANGLE,
                 SERVO_MAX_ANGLE, angle);
        angle = (angle < SERVO_MIN_ANGLE) ? SERVO_MIN_ANGLE : SERVO_MAX_ANGLE;
    }

    // 将角度转换为脉冲宽度
    uint16_t pulse_us =
        SERVO_MIN_PULSE +
        ((angle - SERVO_MIN_ANGLE) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) /
            (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);

    SetServoPulse(channel, pulse_us);
    ESP_LOGI(TAG, "通道 %d 舵机角度设置为 %d° (脉冲宽度: %d us)", channel,
             angle, pulse_us);
}

void Pca9685::SetPWM(uint8_t channel, uint16_t on, uint16_t off) {
    if (channel > 15) {
        ESP_LOGE(TAG, "通道号超出范围 (0-15): %d", channel);
        return;
    }

    uint8_t reg_base = LED0_ON_L + (channel * 4);

    // 添加重试机制
    int retry_count = 0;
    const int max_retries = 3;

    while (retry_count < max_retries) {
        // 添加短暂延迟，避免I2C通信过快
        vTaskDelay(pdMS_TO_TICKS(10));

        WriteReg(reg_base, on & 0xFF);
        vTaskDelay(pdMS_TO_TICKS(5));
        WriteReg(reg_base + 1, (on >> 8) & 0xFF);
        vTaskDelay(pdMS_TO_TICKS(5));
        WriteReg(reg_base + 2, off & 0xFF);
        vTaskDelay(pdMS_TO_TICKS(5));
        WriteReg(reg_base + 3, (off >> 8) & 0xFF);
        vTaskDelay(pdMS_TO_TICKS(5));

        // 验证写入是否成功
        uint8_t on_low = ReadReg(reg_base);
        uint8_t on_high = ReadReg(reg_base + 1);
        uint8_t off_low = ReadReg(reg_base + 2);
        uint8_t off_high = ReadReg(reg_base + 3);

        uint16_t actual_on = (on_high << 8) | on_low;
        uint16_t actual_off = (off_high << 8) | off_low;

        if (actual_on == on && actual_off == off) {
            ESP_LOGD(TAG, "通道%d PWM设置成功: ON=0x%04X, OFF=0x%04X", channel,
                     on, off);
            return;
        } else {
            ESP_LOGW(TAG, "通道%d PWM设置验证失败，重试 %d/%d", channel,
                     retry_count + 1, max_retries);
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    ESP_LOGE(TAG, "通道%d PWM设置失败，已重试%d次", channel, max_retries);
}

void Pca9685::SetServoPulse(uint8_t channel, uint16_t pulse_us) {
    if (channel > 15) {
        ESP_LOGE(TAG, "通道号超出范围 (0-15): %d", channel);
        return;
    }

    if (pulse_us < SERVO_MIN_PULSE || pulse_us > SERVO_MAX_PULSE) {
        ESP_LOGW(TAG, "脉冲宽度超出范围 (%d-%d us): %d", SERVO_MIN_PULSE,
                 SERVO_MAX_PULSE, pulse_us);
        pulse_us =
            (pulse_us < SERVO_MIN_PULSE) ? SERVO_MIN_PULSE : SERVO_MAX_PULSE;
    }

    float pwm_period_us = 1000000.0f / pwm_frequency_;
    uint16_t pwm_units =
        static_cast<uint16_t>((pulse_us * 4096.0f) / pwm_period_us);

    ESP_LOGD(TAG, "通道%d: 脉冲宽度=%dus, PWM周期=%fus, PWM单位=%d", channel,
             pulse_us, pwm_period_us, pwm_units);

    SetPWM(channel, 0, pwm_units);
}

bool Pca9685::IsDeviceReady() {
    // 先进行一次预热读取，避免第一次读取失败
    vTaskDelay(pdMS_TO_TICKS(5));
    ReadReg(MODE1);  // 预热读取，不检查结果

    // 尝试多次读取MODE1寄存器来检查设备是否响应
    int retry_count = 0;
    const int max_retries = 3;
    uint8_t mode1_value = 0;

    while (retry_count < max_retries) {
        vTaskDelay(pdMS_TO_TICKS(10));  // 等待设备稳定

        mode1_value = ReadReg(MODE1);
        ESP_LOGI(TAG, "PCA9685设备状态检查 - 尝试 %d/%d, MODE1寄存器值: 0x%02X",
                 retry_count + 1, max_retries, mode1_value);

        // 检查设备是否响应
        if (mode1_value != 0xFF && mode1_value != 0x00) {
            break;  // 设备响应正常
        }

        retry_count++;
        if (retry_count < max_retries) {
            ESP_LOGW(TAG, "设备无响应，等待后重试...");
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    if (retry_count >= max_retries) {
        ESP_LOGE(TAG, "PCA9685设备无响应，尝试重新初始化...");

        // 尝试重新初始化
        Initialize();
        vTaskDelay(pdMS_TO_TICKS(100));

        // 再次检查
        mode1_value = ReadReg(MODE1);
        ESP_LOGI(TAG, "重新初始化后MODE1寄存器值: 0x%02X", mode1_value);

        if (mode1_value == 0xFF || mode1_value == 0x00) {
            ESP_LOGE(TAG, "PCA9685设备重新初始化失败");
            return false;
        }
    }

    // 检查设备是否在正常状态（不是SLEEP模式）
    if ((mode1_value & 0x10) == 0x10) {
        ESP_LOGW(TAG, "PCA9685处于SLEEP模式，尝试唤醒...");
        // 清除SLEEP位
        WriteReg(MODE1, mode1_value & 0xEF);
        vTaskDelay(pdMS_TO_TICKS(1));

        // 再次检查
        mode1_value = ReadReg(MODE1);
        ESP_LOGI(TAG, "唤醒后MODE1寄存器值: 0x%02X", mode1_value);
    }

    ESP_LOGI(TAG, "PCA9685设备状态正常");
    return true;
}