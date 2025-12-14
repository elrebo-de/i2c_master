/*
 * i2c_master.hpp
 *
 *  Created on: 10.12.2025
 *      Author: christophoberle
 *
 * this work is licenced under the Apache 2.0 licence
 */

#ifndef I2C_MASTER_HPP_
#define I2C_MASTER_HPP_

#include <string>
#include <map>
#include "i2c_device.hpp"

#include "driver/i2c_master.h"

/* class I2cMaster
   Implementation of an I2C master bus for ESP IDF 5.5+
*/
class I2cMaster {
public:
    // Constructor of I2cMaster
	I2cMaster(std::string tag, i2c_port_num_t i2cPort, gpio_num_t sclPin, gpio_num_t sdaPin);
	virtual ~I2cMaster();

    i2c_master_dev_handle_t AddDevice(I2cDevice *device);
    void RemoveDevice(std::string deviceName);
    i2c_master_dev_handle_t GetDeviceHandle(std::string deviceName);

private:
    std::string tag = "I2cMaster";
    i2c_port_num_t i2cPort;
    gpio_num_t sclPin;
    gpio_num_t sdaPin;

    i2c_master_bus_config_t busConfig;
    i2c_master_bus_handle_t busHandle;

    // Map of devices<deviceName, deviceHandle>
    std::map<std::string, I2cDevice *> devices{};
};

#endif /* I2C_MASTER_HPP_ */
