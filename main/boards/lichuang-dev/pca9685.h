#ifndef PCA9685_H
#define PCA9685_H

#include "i2c_device.h"
#include <esp_log.h>

class Pca9685 : public I2cDevice
{
public:
    // 单例获取方法
    static Pca9685 *GetInstance();

    // 初始化方法
    static void Initialize(i2c_master_bus_handle_t i2c_bus, uint8_t addr = 0x40);

    // 设置舵机角度 (0-180度)
    void SetServoAngle(uint8_t channel, int angle);

    void SetServoPulse(uint8_t channel, uint16_t pulse_us);
    void SetPWM(uint8_t channel, uint16_t on, uint16_t off);
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
    static constexpr uint16_t SERVO_MIN_PULSE = 500;  // 最小脉冲宽度 (微秒)
    static constexpr uint16_t SERVO_MAX_PULSE = 2500; // 最大脉冲宽度 (微秒)
    static constexpr uint16_t SERVO_MIN_ANGLE = 0;    // 最小角度
    static constexpr uint16_t SERVO_MAX_ANGLE = 180;  // 最大角度
    static constexpr float DEFAULT_PWM_FREQ = 25.0f;  // 默认PWM频率 (Hz)

    float pwm_frequency_;

    // 内部方法
    void Initialize();
};

#endif // PCA9685_H