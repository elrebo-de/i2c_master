/*
        (i2c_addr_bit_len_t) I2C_ADDR_BIT_LEN_7, // dev_addr_length
        (uint16_t) 0x18, // device_address of mcp9808
        (uint32_t) I2C_MASTER_FREQ_HZ, // scl_speed_hz
        (uint32_t) 0, // scl_wait_us
        {
            (uint32_t) true, // disable_ack_check
        }
    };
*/

/*
 * i2c_master.cpp
 *
 *  Created on 10.12.2025
 *      Author: christophoberle
 *
 * this work is licenced under the Apache 2.0 licence
 */

#include <string>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_idf_version.h"

#include "i2c_master.hpp"

I2cMaster::	I2cMaster(std::string tag,
           	          i2c_port_num_t i2cPort,
           	          gpio_num_t sclPin,
           	          gpio_num_t sdaPin) {
	this->tag = tag;
	this->i2cPort = i2cPort;
    this->sclPin = sclPin;
    this->sdaPin = sdaPin;

    i2c_master_bus_config_t busConfig = {
        (i2c_port_num_t) this->i2cPort,
        (gpio_num_t) this->sdaPin,
        (gpio_num_t) this->sclPin,
        (i2c_clock_source_t) I2C_CLK_SRC_DEFAULT, // clk_source
        (uint8_t) 7, // glitch_ignore_cnt
        (int) 0, // intr_priority
        (size_t) 2, // trans_queue_depth
        { // flags
            (uint32_t) true, // enable_internal_pullup
            (uint32_t) false, // allow_pd
        }
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&busConfig, &(this->busHandle)));
}

I2cMaster::~I2cMaster() {
    // Iterate over devices to remove them
    for (const auto& [deviceName, deviceHandle] : this->devices) {
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(deviceHandle));
    }
    // delete master bus
    ESP_ERROR_CHECK(i2c_del_master_bus(this->busHandle));
}

i2c_master_dev_handle_t I2cMaster::AddDevice(
        std::string deviceName,
        i2c_addr_bit_len_t devAddrLength,
        uint16_t deviceAddress,
        uint32_t sclSpeedHz) {

    i2c_master_dev_handle_t devHandle;

    i2c_device_config_t devConfig = {
        (i2c_addr_bit_len_t) devAddrLength,
        (uint16_t) deviceAddress,
        (uint32_t) sclSpeedHz,
        (uint32_t) 0, // scl_wait_us
        {
            (uint32_t) false, // disable_ack_check
        }
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(this->busHandle, &devConfig, &devHandle));

    this->devices[deviceName] = devHandle;

    return devHandle;
}

void I2cMaster::RemoveDevice(std::string deviceName) {
    this->devices.erase(deviceName);
}

i2c_master_dev_handle_t I2cMaster::GetDeviceHandle(std::string deviceName) {
    return this->devices.at(deviceName);
}

