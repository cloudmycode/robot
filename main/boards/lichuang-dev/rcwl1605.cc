#include "rcwl1605.h"
#include <vector>
#include <algorithm>

const char* Rcwl1605::TAG = "RCWL1605";

Rcwl1605::Rcwl1605(gpio_num_t trigger_pin, gpio_num_t echo_pin)
    : trigger_pin_(trigger_pin), 
      echo_pin_(echo_pin), 
      timeout_us_(DEFAULT_TIMEOUT_US),
      is_initialized_(false),
      last_measurement_time_(0) {
}

Rcwl1605::~Rcwl1605() {
    // 清理GPIO配置
    if (is_initialized_) {
        gpio_reset_pin(trigger_pin_);
        gpio_reset_pin(echo_pin_);
    }
}

esp_err_t Rcwl1605::Initialize() {
    esp_err_t ret = ESP_OK;
    
    // 配置触发引脚为输出
    gpio_config_t trigger_config = {
        .pin_bit_mask = (1ULL << trigger_pin_),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&trigger_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure trigger pin: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 配置回波引脚为输入
    gpio_config_t echo_config = {
        .pin_bit_mask = (1ULL << echo_pin_),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&echo_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure echo pin: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 初始化触发引脚为低电平
    gpio_set_level(trigger_pin_, 0);
    
    // 等待一段时间确保传感器稳定
    vTaskDelay(pdMS_TO_TICKS(100));
    
    is_initialized_ = true;
    ESP_LOGI(TAG, "RCWL-1605 initialized successfully. Trigger: GPIO%d, Echo: GPIO%d", 
             trigger_pin_, echo_pin_);
    
    return ESP_OK;
}

void Rcwl1605::SendTrigger() {
    if (!is_initialized_) {
        ESP_LOGE(TAG, "Sensor not initialized");
        return;
    }
    
    // 发送10微秒的触发脉冲
    gpio_set_level(trigger_pin_, 1);
    
    // 使用esp_timer_get_time()来实现精确延时
    uint64_t start_time = esp_timer_get_time();
    while ((esp_timer_get_time() - start_time) < TRIGGER_PULSE_US) {
        // 等待10微秒
    }
    
    gpio_set_level(trigger_pin_, 0);
}

uint32_t Rcwl1605::MeasureEchoTime() {
    if (!is_initialized_) {
        ESP_LOGE(TAG, "Sensor not initialized");
        return 0;
    }
    
    uint64_t start_time, end_time;
    uint32_t echo_time_us = 0;
    
    // 等待回波引脚变为高电平
    start_time = esp_timer_get_time();
    while (gpio_get_level(echo_pin_) == 0) {
        if ((esp_timer_get_time() - start_time) > timeout_us_) {
            ESP_LOGW(TAG, "Echo timeout waiting for high level");
            return 0;
        }
    }
    
    // 记录回波开始时间
    start_time = esp_timer_get_time();
    
    // 等待回波引脚变为低电平
    while (gpio_get_level(echo_pin_) == 1) {
        if ((esp_timer_get_time() - start_time) > timeout_us_) {
            ESP_LOGW(TAG, "Echo timeout waiting for low level");
            return 0;
        }
    }
    
    // 记录回波结束时间
    end_time = esp_timer_get_time();
    
    // 计算回波时间
    echo_time_us = (uint32_t)(end_time - start_time);
    
    return echo_time_us;
}

float Rcwl1605::CalculateDistance(uint32_t echo_time_us) {
    if (echo_time_us == 0) {
        return 0.0f;
    }
    
    // 使用与厂家参考程序一致的计算公式
    // distance = echo_time_us * 340 / 2 / 10000
    // 其中：340 m/s 是声速，除以2是因为来回距离，除以10000是将微秒转换为厘米
    float distance_cm = (echo_time_us * SOUND_SPEED_MPS) / (2.0f * 10000.0f);
    
    // 检查距离是否在有效范围内（包含盲区检查）
    if (distance_cm < MIN_VALID_DISTANCE || distance_cm > MAX_VALID_DISTANCE) {
        ESP_LOGW(TAG, "Distance %.2f cm is out of valid range [%.1f, %.1f] cm", 
                     distance_cm, MIN_VALID_DISTANCE, MAX_VALID_DISTANCE);
        return 0.0f;  // 返回0表示无效测量
    }
    
    return distance_cm;
}

float Rcwl1605::GetDistance() {
    if (!is_initialized_) {
        ESP_LOGE(TAG, "Sensor not initialized");
        return 0.0f;
    }
    
    // 发送触发信号
    SendTrigger();
    
    // 测量回波时间
    uint32_t echo_time_us = MeasureEchoTime();
    
    // 计算距离
    float distance = CalculateDistance(echo_time_us);
    
    // 更新最后测量时间
    last_measurement_time_ = esp_timer_get_time();
    
    ESP_LOGI(TAG, "Distance: %.2f cm, Echo time: %lu us", distance, echo_time_us);
    
    return distance;
}

bool Rcwl1605::IsValid() {
    return is_initialized_;
}

void Rcwl1605::SetTimeout(uint32_t timeout_us) {
    timeout_us_ = timeout_us;
    ESP_LOGI(TAG, "Timeout set to %lu us", timeout_us);
}

uint64_t Rcwl1605::GetLastMeasurementTime() {
    return last_measurement_time_;
}

float Rcwl1605::GetDistanceAccurate() {
    if (!is_initialized_) {
        ESP_LOGE(TAG, "Sensor not initialized");
        return 0.0f;
    }
    
    const uint8_t total_measurements = 10;
    const uint8_t remove_count = 2;  // 去除最大和最小的各2个
    const uint8_t valid_count = total_measurements - (remove_count * 2);  // 中间6个
    
    std::vector<float> distances;
    distances.reserve(total_measurements);
    
    ESP_LOGI(TAG, "开始进行 %d 次精确测量，去除最大%d个和最小%d个，取中间%d个的平均值", 
             total_measurements, remove_count, remove_count, valid_count);
    
    // 进行10次测量
    for (uint8_t i = 0; i < total_measurements; i++) {
        float distance = GetDistance();
        
        if (distance > 0) {
            distances.push_back(distance);
            ESP_LOGI(TAG, "测量 %d: %.2f cm", i + 1, distance);
        } else {
            ESP_LOGW(TAG, "测量 %d: 无效", i + 1);
        }
        
        // 测量间隔，符合厂家测量周期时间规格
        if (i < total_measurements - 1) {
            vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_CYCLE_MS));
        }
    }
    
    // 检查有效测量数量
    if (distances.size() < valid_count) {
        ESP_LOGE(TAG, "有效测量数量不足，需要至少 %d 个有效测量，实际只有 %zu 个", 
                 valid_count, distances.size());
        return 0.0f;
    }
    
    // 排序
    std::sort(distances.begin(), distances.end());
    
    // 去除最大和最小的各2个，取中间6个
    float total_distance = 0.0f;
    for (uint8_t i = remove_count; i < remove_count + valid_count; i++) {
        total_distance += distances[i];
        ESP_LOGI(TAG, "使用测量值 %d: %.2f cm", i + 1, distances[i]);
    }
    
    float accurate_distance = total_distance / valid_count;
    
    ESP_LOGI(TAG, "精确距离: %.2f cm (使用 %d 个有效测量值的平均值)", 
             accurate_distance, valid_count);
    
    return accurate_distance;
}
