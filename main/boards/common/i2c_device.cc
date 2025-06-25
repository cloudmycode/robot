#include "i2c_device.h"

#include <esp_log.h>

#define TAG "I2cDevice"
#define I2C_TIMEOUT_MS (1000)

I2cDevice::I2cDevice(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
{
    i2c_device_config_t i2c_device_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100 * 1000,
        .scl_wait_us = 100,
        .flags = {
            .disable_ack_check = 0,
        },
    };
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &i2c_device_cfg, &i2c_device_);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "添加I2C设备失败，地址: 0x%02X, 错误: %s", addr, esp_err_to_name(ret));
        i2c_device_ = nullptr;
    }
    else
    {
        assert(i2c_device_ != NULL);
    }
}

void I2cDevice::WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    esp_err_t ret = i2c_master_transmit(i2c_device_, buffer, 2, I2C_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C写寄存器失败，寄存器: 0x%02X, 值: 0x%02X, 错误: %s",
                 reg, value, esp_err_to_name(ret));
    }
}

uint8_t I2cDevice::ReadReg(uint8_t reg)
{
    uint8_t buffer[1];
    esp_err_t ret = i2c_master_transmit_receive(i2c_device_, &reg, 1, buffer, 1, I2C_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C读寄存器失败，寄存器: 0x%02X, 错误: %s",
                 reg, esp_err_to_name(ret));
        return 0;
    }
    return buffer[0];
}

void I2cDevice::ReadRegs(uint8_t reg, uint8_t *buffer, size_t length)
{
    if (i2c_device_ == nullptr)
    {
        ESP_LOGE(TAG, "I2C设备未初始化");
        return;
    }

    esp_err_t ret = i2c_master_transmit_receive(i2c_device_, &reg, 1, buffer, length, I2C_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C读多个寄存器失败，寄存器: 0x%02X, 长度: %d, 错误: %s",
                 reg, length, esp_err_to_name(ret));
    }
}