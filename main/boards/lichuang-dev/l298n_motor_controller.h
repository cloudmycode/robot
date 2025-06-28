#ifndef L298N_MOTOR_CONTROLLER_H
#define L298N_MOTOR_CONTROLLER_H

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include "pca9685.h"

/**
 * @brief 电机方向枚举
 */
enum class MotorDirection {
    FORWARD = 0,   // 正转
    BACKWARD = 1,  // 反转
    STOP = 2       // 停止
};

/**
 * @brief 电机控制结构体
 */
struct MotorControl {
    uint8_t channel;           // PCA9685通道号
    MotorDirection direction;  // 电机方向
    uint8_t speed;            // 电机速度 (0-100)
    
    MotorControl(uint8_t ch, MotorDirection dir, uint8_t spd) 
        : channel(ch), direction(dir), speed(spd) {}
};

/**
 * @brief 平滑过渡配置结构体
 */
struct SmoothTransitionConfig {
    uint16_t ramp_time_ms;     // 加速/减速时间（毫秒）
    uint16_t step_interval_ms; // 速度更新间隔（毫秒）
    uint8_t min_speed;         // 最小速度（防止电机卡死）
    bool enable_smooth_transition; // 是否启用平滑过渡
    
    SmoothTransitionConfig() 
        : ramp_time_ms(500), step_interval_ms(20), min_speed(5), enable_smooth_transition(true) {}
};

/**
 * @brief L298N电机驱动控制器类
 * 
 * 使用PCA9685控制L298N驱动器：
 * - PWM11 -> ENA (电机A使能)
 * - PWM12 -> IN1 (电机A方向1)
 * - PWM13 -> IN2 (电机A方向2)
 * - PWM14 -> ENB (电机B使能)
 * - PWM15 -> IN3 (电机B方向1)
 * - PWM16 -> IN4 (电机B方向2)
 */
class L298nMotorController {
public:
    /**
     * @brief 获取L298N电机控制器单例实例
     */
    static L298nMotorController* GetInstance();
    
    /**
     * @brief 初始化L298N电机控制器
     */
    static void Initialize();
    
    /**
     * @brief 设置电机A的速度和方向（带平滑过渡）
     * @param direction 电机方向
     * @param speed 电机速度 (0-100)
     * @param smooth_transition 是否使用平滑过渡
     */
    void SetMotorA(MotorDirection direction, uint8_t speed, bool smooth_transition = true);
    
    /**
     * @brief 设置电机B的速度和方向（带平滑过渡）
     * @param direction 电机方向
     * @param speed 电机速度 (0-100)
     * @param smooth_transition 是否使用平滑过渡
     */
    void SetMotorB(MotorDirection direction, uint8_t speed, bool smooth_transition = true);
    
    /**
     * @brief 同时控制两个电机（带平滑过渡）
     * @param motor_a 电机A控制参数
     * @param motor_b 电机B控制参数
     * @param smooth_transition 是否使用平滑过渡
     */
    void SetMotors(const MotorControl& motor_a, const MotorControl& motor_b, bool smooth_transition = true);
    
    /**
     * @brief 停止电机A（平滑停止）
     * @param smooth_stop 是否平滑停止
     */
    void StopMotorA(bool smooth_stop = true);
    
    /**
     * @brief 停止电机B（平滑停止）
     * @param smooth_stop 是否平滑停止
     */
    void StopMotorB(bool smooth_stop = true);
    
    /**
     * @brief 停止所有电机（平滑停止）
     * @param smooth_stop 是否平滑停止
     */
    void StopAllMotors(bool smooth_stop = true);
    
    /**
     * @brief 设置电机A的速度（保持当前方向，平滑过渡）
     * @param speed 电机速度 (0-100)
     * @param smooth_transition 是否使用平滑过渡
     */
    void SetMotorASpeed(uint8_t speed, bool smooth_transition = true);
    
    /**
     * @brief 设置电机B的速度（保持当前方向，平滑过渡）
     * @param speed 电机速度 (0-100)
     * @param smooth_transition 是否使用平滑过渡
     */
    void SetMotorBSpeed(uint8_t speed, bool smooth_transition = true);
    
    /**
     * @brief 获取电机A当前状态
     */
    MotorControl GetMotorAStatus() const { return motor_a_status_; }
    
    /**
     * @brief 获取电机B当前状态
     */
    MotorControl GetMotorBStatus() const { return motor_b_status_; }
    
    /**
     * @brief 设置平滑过渡配置
     * @param config 平滑过渡配置
     */
    void SetSmoothTransitionConfig(const SmoothTransitionConfig& config);
    
    /**
     * @brief 获取平滑过渡配置
     */
    SmoothTransitionConfig GetSmoothTransitionConfig() const { return smooth_config_; }
    
    /**
     * @brief 等待电机A过渡完成
     * @param timeout_ms 超时时间（毫秒）
     * @return 是否成功完成过渡
     */
    bool WaitMotorATransition(uint32_t timeout_ms = 5000);
    
    /**
     * @brief 等待电机B过渡完成
     * @param timeout_ms 超时时间（毫秒）
     * @return 是否成功完成过渡
     */
    bool WaitMotorBTransition(uint32_t timeout_ms = 5000);
    
    /**
     * @brief 等待所有电机过渡完成
     * @param timeout_ms 超时时间（毫秒）
     * @return 是否成功完成过渡
     */
    bool WaitAllMotorsTransition(uint32_t timeout_ms = 5000);

private:
    // 私有构造函数，实现单例模式
    L298nMotorController();
    ~L298nMotorController();
    
    // 禁用拷贝构造和赋值
    L298nMotorController(const L298nMotorController&) = delete;
    L298nMotorController& operator=(const L298nMotorController&) = delete;
    
    /**
     * @brief 将速度百分比转换为PWM值
     * @param speed_percent 速度百分比 (0-100)
     * @return PWM值 (0-4095)
     */
    uint16_t SpeedToPWM(uint8_t speed_percent);
    
    /**
     * @brief 设置单个电机的PWM输出
     * @param enable_channel 使能通道
     * @param in1_channel IN1通道
     * @param in2_channel IN2通道
     * @param direction 方向
     * @param speed 速度
     */
    void SetMotorPWM(uint8_t enable_channel, uint8_t in1_channel, uint8_t in2_channel,
                     MotorDirection direction, uint8_t speed);
    
    /**
     * @brief 平滑过渡任务
     * @param parameter 任务参数
     */
    static void SmoothTransitionTask(void* parameter);
    
    /**
     * @brief 执行电机A的平滑过渡
     * @param target_direction 目标方向
     * @param target_speed 目标速度
     */
    void ExecuteMotorASmoothTransition(MotorDirection target_direction, uint8_t target_speed);
    
    /**
     * @brief 执行电机B的平滑过渡
     * @param target_direction 目标方向
     * @param target_speed 目标速度
     */
    void ExecuteMotorBSmoothTransition(MotorDirection target_direction, uint8_t target_speed);
    
    /**
     * @brief 执行平滑减速到停止
     * @param motor_id 电机ID (0=A, 1=B)
     */
    void ExecuteSmoothStop(uint8_t motor_id);

private:
    static L298nMotorController* instance_;
    
    // PCA9685实例
    Pca9685* pca9685_;
    
    // 电机A和B的当前状态
    MotorControl motor_a_status_;
    MotorControl motor_b_status_;
    
    // 平滑过渡配置
    SmoothTransitionConfig smooth_config_;
    
    // 平滑过渡任务相关
    TaskHandle_t smooth_transition_task_;
    bool transition_in_progress_;
    bool motor_a_transitioning_;
    bool motor_b_transitioning_;
    
    // 目标状态（用于平滑过渡）
    MotorDirection motor_a_target_direction_;
    uint8_t motor_a_target_speed_;
    MotorDirection motor_b_target_direction_;
    uint8_t motor_b_target_speed_;
    
    // L298N通道定义
    static constexpr uint8_t MOTOR_A_ENABLE = 10;  // ENA
    static constexpr uint8_t MOTOR_A_IN1 = 11;     // IN1
    static constexpr uint8_t MOTOR_A_IN2 = 12;     // IN2
    static constexpr uint8_t MOTOR_B_ENABLE = 13;  // ENB
    static constexpr uint8_t MOTOR_B_IN3 = 14;     // IN3
    static constexpr uint8_t MOTOR_B_IN4 = 15;     // IN4
    
    // PWM相关常量
    static constexpr uint16_t PWM_MAX = 4095;      // PCA9685最大PWM值
    static constexpr uint16_t PWM_MIN = 0;         // PCA9685最小PWM值
    static constexpr uint8_t SPEED_MAX = 100;      // 最大速度百分比
    static constexpr uint8_t SPEED_MIN = 0;        // 最小速度百分比
    
    // 平滑过渡相关常量
    static constexpr uint16_t DEFAULT_RAMP_TIME_MS = 500;     // 默认过渡时间
    static constexpr uint16_t DEFAULT_STEP_INTERVAL_MS = 20;  // 默认步进间隔
    static constexpr uint8_t DEFAULT_MIN_SPEED = 5;           // 默认最小速度
};

#endif // L298N_MOTOR_CONTROLLER_H 