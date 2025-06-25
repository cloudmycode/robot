#ifndef PCA9685_H
#define PCA9685_H

#include <esp_log.h>

#include "i2c_device.h"

/**
 * @brief PCA9685 PWM控制器类
 *
 * PCA9685是一个16通道12位PWM控制器，常用于控制舵机、LED等设备。
 * 该类实现了单例模式，确保全局只有一个PCA9685实例。
 */
class Pca9685 : public I2cDevice {
   public:
    /**
     * @brief 获取PCA9685单例实例
     *
     * @return Pca9685* 返回PCA9685实例指针
     *
     * 用法：
     * Pca9685* pca = Pca9685::GetInstance();
     *
     * 注意：使用前必须先调用Initialize()方法进行初始化
     */
    static Pca9685 *GetInstance();

    /**
     * @brief 初始化PCA9685设备
     *
     * @param i2c_bus I2C总线句柄
     * @param addr PCA9685设备地址，默认为0x40
     *
     * 功能：
     * - 创建PCA9685实例
     * - 配置I2C通信
     * - 设置PWM频率为25Hz（适合舵机控制）
     *
     * 用法：
     * Pca9685::Initialize(i2c_bus_handle, 0x40);
     */
    static void Initialize(i2c_master_bus_handle_t i2c_bus,
                           uint8_t addr = 0x40);

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
     * - 自动调用SetServoPulse()设置脉冲宽度
     *
     * 用法：
     * pca->SetServoAngle(0, 90);  // 设置通道0的舵机到90度位置
     * pca->SetServoAngle(1, 0);   // 设置通道1的舵机到0度位置
     */
    void SetServoAngle(uint8_t channel, int angle);

    /**
     * @brief 设置舵机脉冲宽度
     *
     * @param channel 通道号（0-15）
     * @param pulse_us 脉冲宽度（微秒，通常500-2500）
     *
     * 功能：
     * - 直接设置PWM脉冲宽度
     * - 将微秒值转换为PCA9685的12位PWM值
     * - 调用SetPWM()设置具体的PWM参数
     *
     * 用法：
     * pca->SetServoPulse(0, 1500);  // 设置通道0为1500微秒脉冲（舵机中间位置）
     * pca->SetServoPulse(1, 2000);  // 设置通道1为2000微秒脉冲
     */
    void SetServoPulse(uint8_t channel, uint16_t pulse_us);

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

    /**
     * @brief 检查设备是否就绪
     *
     * @return bool 设备就绪返回true，否则返回false
     *
     * 功能：
     * - 通过I2C通信检查PCA9685是否响应
     * - 用于设备连接状态检测
     *
     * 用法：
     * if (pca->IsDeviceReady()) {
     *     // 设备正常，可以操作
     * } else {
     *     // 设备未连接或故障
     * }
     */
    bool IsDeviceReady();

   private:
    // 私有构造函数
    Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr);

    // 禁用拷贝构造和赋值
    Pca9685(const Pca9685 &) = delete;
    Pca9685 &operator=(const Pca9685 &) = delete;

    // 静态实例指针
    static Pca9685 *instance_;

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
};

#endif  // PCA9685_H