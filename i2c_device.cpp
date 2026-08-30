/*
 * i2c_device.cpp
 *
 *      Author: christophoberle
 *
 * this work is licenced under the Apache 2.0 licence
 */

#include <string>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_idf_version.h"

#include "i2c_device.hpp"

I2cDevice::I2cDevice(std::string tag,
                     std::string deviceName,
                     i2c_addr_bit_len_t devAddrLength,
                     uint16_t deviceAddress,
                     uint32_t sclSpeedHz) {
	this->tag = tag;
	this->deviceName = deviceName;

    this->devConfig = {
        (i2c_addr_bit_len_t) devAddrLength,
        (uint16_t) deviceAddress,
        (uint32_t) sclSpeedHz,
        (uint32_t) 0, // scl_wait_us
        {
            (uint32_t) false, // disable_ack_check
        }
    };
}

I2cDevice::~I2cDevice() {
}

i2c_device_config_t I2cDevice::GetConfig() {
    return this->devConfig;
}

std::string I2cDevice::GetDeviceName() {
    return this->deviceName;
}

void I2cDevice::SetHandle(i2c_master_dev_handle_t devHandle) {
    this->devHandle = devHandle;
}

i2c_master_dev_handle_t I2cDevice::GetHandle() {
    return this->devHandle;
}

// ReadRegister method
// reads one byte from a register
uint8_t I2cDevice::ReadRegister(uint8_t reg_addr) {
    uint8_t data_val = 0;

    // Transmit register target address, then immediately capture 1 returned byte
    esp_err_t err = i2c_master_transmit_receive(
        this->devHandle,
        &reg_addr, 1,
        &data_val, 1,
        -1
    );

    if (err != ESP_OK) {
        ESP_LOGE(this->tag.c_str(), "I2C read failed at reg 0x%02X: %s", reg_addr, esp_err_to_name(err));
        return 0;
    }
    return data_val;
}


