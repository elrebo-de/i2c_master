/*
 * i2c_device.cpp
 *
 *  Created on 14.12.2025
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

