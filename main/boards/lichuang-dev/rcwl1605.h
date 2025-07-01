#ifndef RCWL1605_H
#define RCWL1605_H

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

class Rcwl1605 {
public:
    // 构造函数，传入触发引脚和回波引脚
    Rcwl1605(gpio_num_t trigger_pin, gpio_num_t echo_pin);
    
    // 析构函数
    ~Rcwl1605();
    
    // 初始化传感器
    esp_err_t Initialize();
    
    // 获取距离值（单位：厘米）
    float GetDistance();
    
    // 检查传感器是否正常工作
    bool IsValid();
    
    // 设置超时时间（微秒）
    void SetTimeout(uint32_t timeout_us);
    
    // 获取最后一次测量的时间戳
    uint64_t GetLastMeasurementTime();
    
    // 获取精确距离（10次测量，去除最大2个和最小2个，取中间6个的平均值）
    float GetDistanceAccurate();

private:
    // 发送触发信号
    void SendTrigger();
    
    // 测量回波时间
    uint32_t MeasureEchoTime();
    
    // 计算距离
    float CalculateDistance(uint32_t echo_time_us);

private:
    gpio_num_t trigger_pin_;      // 触发引脚
    gpio_num_t echo_pin_;         // 回波引脚
    uint32_t timeout_us_;         // 超时时间（微秒）
    bool is_initialized_;         // 初始化状态
    uint64_t last_measurement_time_; // 最后一次测量时间戳
    
    // 常量定义（根据厂家规格参数）
    static constexpr uint32_t DEFAULT_TIMEOUT_US = 30000;  // 默认超时30ms
    static constexpr uint32_t TRIGGER_PULSE_US = 10;       // 触发脉冲宽度10us
    static constexpr float SOUND_SPEED_MPS = 342.62f;       // 声速（米/秒），与厂家参考程序一致
    static constexpr float MIN_VALID_DISTANCE = 26.0f;     // 最小有效距离（厘米），大于盲区26cm
    static constexpr float MAX_VALID_DISTANCE = 550.0f;    // 最大有效距离（厘米），厂家规格450-550cm
    static constexpr uint32_t MEASUREMENT_CYCLE_MS = 50;   // 测量周期时间（毫秒），厂家规格
    
    static const char* TAG;
};

#endif // RCWL1605_H 