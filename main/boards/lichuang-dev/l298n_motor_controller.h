#ifndef L298N_MOTOR_CONTROLLER_H
#define L298N_MOTOR_CONTROLLER_H

#include <esp_log.h>
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
 * @brief L298N电机驱动控制器类
 * 
 * 使用PCA9685控制L298N驱动器：
 * - PWM10 -> ENA (电机A使能)
 * - PWM11 -> IN1 (电机A方向1)
 * - PWM12 -> IN2 (电机A方向2)
 * - PWM13 -> ENB (电机B使能)
 * - PWM14 -> IN3 (电机B方向1)
 * - PWM15 -> IN4 (电机B方向2)
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
     * @brief 设置电机A的速度和方向
     * @param direction 电机方向
     * @param speed 电机速度 (0-100)
     */
    void SetMotorA(MotorDirection direction, uint8_t speed);
    
    /**
     * @brief 设置电机B的速度和方向
     * @param direction 电机方向
     * @param speed 电机速度 (0-100)
     */
    void SetMotorB(MotorDirection direction, uint8_t speed);
    
    /**
     * @brief 同时控制两个电机
     * @param motor_a 电机A控制参数
     * @param motor_b 电机B控制参数
     */
    void SetMotors(const MotorControl& motor_a, const MotorControl& motor_b);
    
    /**
     * @brief 停止电机A
     */
    void StopMotorA();
    
    /**
     * @brief 停止电机B
     */
    void StopMotorB();
    
    /**
     * @brief 停止所有电机
     */
    void StopAllMotors();
    
    /**
     * @brief 设置电机A的速度（保持当前方向）
     * @param speed 电机速度 (0-100)
     */
    void SetMotorASpeed(uint8_t speed);
    
    /**
     * @brief 设置电机B的速度（保持当前方向）
     * @param speed 电机速度 (0-100)
     */
    void SetMotorBSpeed(uint8_t speed);
    
    /**
     * @brief 获取电机A当前状态
     */
    MotorControl GetMotorAStatus() const { return motor_a_status_; }
    
    /**
     * @brief 获取电机B当前状态
     */
    MotorControl GetMotorBStatus() const { return motor_b_status_; }

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

private:
    static L298nMotorController* instance_;
    
    // PCA9685实例
    Pca9685* pca9685_;
    
    // 电机A和B的当前状态
    MotorControl motor_a_status_;
    MotorControl motor_b_status_;
    
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
};

#endif // L298N_MOTOR_CONTROLLER_H 