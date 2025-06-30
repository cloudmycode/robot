#ifndef PCA9685_H
#define PCA9685_H

#include <esp_log.h>
#include <map>

#include "i2c_device.h"

// 宏定义PCA9685实例ID
#define GPIO_10_PCA9685 0       // 使用GPIO10上的PCA9685
#define GPIO_I2C_PCA9685 1      // 使用I2C总线上的PCA9685
#define MAX_PCA9685_INSTANCES 2 // 最大实例数
/**
 * @brief 舵机控制结构体
 */
struct ServoControl {
    uint8_t channel;  // 通道号（0-15）
    int angle;        // 舵机角度（0-180度）

    ServoControl(uint8_t ch, int ang) : channel(ch), angle(ang) {}
};

/**
 * @brief PCA9685 PWM控制器类
 *
 * PCA9685是一个16通道12位PWM控制器，常用于控制舵机、LED等设备。
 * 该类支持多实例管理，可以通过预定义的ID获取不同的PCA9685实例。
 */
class Pca9685 : public I2cDevice {
   public:
    /**
     * @brief 获取指定ID的PCA9685实例
     *
     * @param id PCA9685实例ID（使用宏定义，如GPIO_10_PCA9685）
     * @return Pca9685* 返回PCA9685实例指针，如果不存在则返回nullptr
     *
     * 用法：
     * Pca9685* pca1 = Pca9685::GetInstance(GPIO_10_PCA9685);
     * Pca9685* pca2 = Pca9685::GetInstance(GPIO_11_PCA9685);
     *
     * 注意：使用前必须先调用Initialize()方法进行初始化
     */
    static Pca9685* GetInstance(uint8_t id);

    /**
     * @brief 初始化指定ID的PCA9685设备
     *
     * @param id PCA9685实例ID（使用宏定义，如GPIO_10_PCA9685）
     * @param i2c_bus I2C总线句柄
     * @param addr PCA9685设备地址，默认为0x40
     *
     * 功能：
     * - 创建指定ID的PCA9685实例
     * - 配置I2C通信
     * - 设置PWM频率为25Hz（适合舵机控制）
     *
     * 用法：
     * Pca9685::Initialize(GPIO_10_PCA9685, i2c_bus_handle, 0x40);
     * Pca9685::Initialize(GPIO_11_PCA9685, i2c_bus_handle, 0x41);
     */
    static void Initialize(uint8_t id, i2c_master_bus_handle_t i2c_bus,
                          uint8_t addr = 0x40);

    /**
     * @brief 销毁指定ID的PCA9685实例
     *
     * @param id PCA9685实例ID
     *
     * 功能：
     * - 释放指定ID的PCA9685实例
     * - 清理相关资源
     *
     * 用法：
     * Pca9685::Destroy(GPIO_10_PCA9685);
     */
    static void Destroy(uint8_t id);

    /**
     * @brief 销毁所有PCA9685实例
     *
     * 功能：
     * - 释放所有PCA9685实例
     * - 清理所有相关资源
     *
     * 用法：
     * Pca9685::DestroyAll();
     */
    static void DestroyAll();

    /**
     * @brief 检查指定ID的PCA9685实例是否存在
     *
     * @param id PCA9685实例ID
     * @return true 实例存在，false 实例不存在
     *
     * 用法：
     * if (Pca9685::IsInstanceExists(GPIO_10_PCA9685)) {
     *     // 实例存在，可以安全使用
     * }
     */
    static bool IsInstanceExists(uint8_t id);

    /**
     * @brief 设置舵机角度
     *
     * @param channel 通道号（0-15）
     * @param angle 舵机角度（0-180度）
     *
     * 功能：
     * - 将角度值转换为PWM脉冲宽度
     * - 0度对应500微秒脉冲
     * - 180度对应2500微秒脉冲
     * - 自动调用SetPWM()设置具体的PWM参数
     *
     * 用法：
     * pca->SetServoAngle(0, 90);  // 设置通道0的舵机到90度位置
     * pca->SetServoAngle(1, 0);   // 设置通道1的舵机到0度位置
     */
    void SetServoAngle(uint8_t channel, int angle);

    /**
     * @brief 批量设置多个舵机角度（同时执行）
     *
     * @param servos 舵机控制结构体数组
     * @param count 舵机数量
     *
     * 功能：
     * - 同时设置多个舵机的角度
     * - 使用PCA9685的AUTO_INCREMENT功能
     * - 所有舵机会同时开始运动
     *
     * 用法：
     * ServoControl servos[] = {
     *     {0, 90},   // 通道0，90度
     *     {1, 180}   // 通道1，180度
     * };
     * pca->SetServoAngles(servos, 2);  // 同时设置通道0和1
     */
    void SetServoAngles(const ServoControl *servos, size_t count);

    /**
     * @brief 批量设置多个舵机角度（自动计算大小）- 模板版本
     *
     * @param servos 舵机控制结构体数组
     *
     * 功能：
     * - 同时设置多个舵机的角度
     * - 自动计算数组大小，无需手动指定count
     * - 使用PCA9685的AUTO_INCREMENT功能
     * - 所有舵机会同时开始运动
     *
     * 用法：
     * ServoControl servos[] = {
     *     {0, 90},   // 通道0，90度
     *     {1, 180}   // 通道1，180度
     * };
     * pca->SetServoAngles(servos);  // 自动计算大小为2
     */
    template <size_t N>
    void SetServoAngles(const ServoControl (&servos)[N]) {
        SetServoAngles(servos, N);
    }

    /**
     * @brief 设置PWM输出
     *
     * @param channel 通道号（0-15）
     * @param on 开启时间（0-4095）
     * @param off 关闭时间（0-4095）
     *
     * 功能：
     * - 直接设置PCA9685的PWM寄存器值
     * - on和off都是12位值（0-4095）
     * - 占空比 = (off - on) / 4096
     *
     * 用法：
     * pca->SetPWM(0, 0, 2048);      // 设置通道0为50%占空比
     * pca->SetPWM(1, 0, 1024);      // 设置通道1为25%占空比
     * pca->SetPWM(2, 0, 4095);      // 设置通道2为100%占空比（常开）
     */
    void SetPWM(uint8_t channel, uint16_t on, uint16_t off);

   private:
    // 私有构造函数
    Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr);

    // 禁用拷贝构造和赋值
    Pca9685(const Pca9685 &) = delete;
    Pca9685 &operator=(const Pca9685 &) = delete;

    // 静态实例映射表
    static std::map<uint8_t, Pca9685*> instances_;

    // 寄存器地址常量
    static constexpr uint8_t MODE1 = 0x00;
    static constexpr uint8_t MODE2 = 0x01;
    static constexpr uint8_t SUBADR1 = 0x02;
    static constexpr uint8_t SUBADR2 = 0x03;
    static constexpr uint8_t SUBADR3 = 0x04;
    static constexpr uint8_t ALLCALLADR = 0x05;
    static constexpr uint8_t LED0_ON_L = 0x06;
    static constexpr uint8_t LED0_ON_H = 0x07;
    static constexpr uint8_t LED0_OFF_L = 0x08;
    static constexpr uint8_t LED0_OFF_H = 0x09;
    static constexpr uint8_t ALL_LED_ON_L = 0xFA;
    static constexpr uint8_t ALL_LED_ON_H = 0xFB;
    static constexpr uint8_t ALL_LED_OFF_L = 0xFC;
    static constexpr uint8_t ALL_LED_OFF_H = 0xFD;
    static constexpr uint8_t PRESCALE = 0xFE;

    // 舵机参数
    static constexpr uint16_t SERVO_MIN_PULSE = 500;   // 最小脉冲宽度 (微秒)
    static constexpr uint16_t SERVO_MAX_PULSE = 2500;  // 最大脉冲宽度 (微秒)
    static constexpr uint16_t SERVO_MIN_ANGLE = 0;     // 最小角度
    static constexpr uint16_t SERVO_MAX_ANGLE = 180;   // 最大角度
    static constexpr float DEFAULT_PWM_FREQ = 25.0f;   // 默认PWM频率 (Hz)

    float pwm_frequency_;

    // 内部方法
    void Initialize();
    void SetMultiplePWM(const uint8_t *channels, const uint16_t *off_values, size_t count);
    bool IsDeviceReady();
};

#endif  // PCA9685_H