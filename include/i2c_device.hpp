/*
 * i2c_device.hpp
 *
 *      Author: christophoberle
 *
 * this work is licenced under the Apache 2.0 licence
 */

#ifndef I2C_DEVICE_HPP_
#define I2C_DEVICE_HPP_

#include <string>

#include "driver/i2c_master.h"

/* class I2cDevice
   Implementation of an I2C device for I2C master bus for ESP IDF 5.5+
*/
class I2cDevice {
public:
    // Constructor of I2cDevice
	I2cDevice(std::string tag, std::string deviceName, i2c_addr_bit_len_t devAddrLength, uint16_t deviceAddress,
	          uint32_t sclSpeedHz);
	virtual ~I2cDevice();

	i2c_device_config_t GetConfig();
	std::string GetDeviceName();

	void SetHandle(i2c_master_dev_handle_t devHandle);
	i2c_master_dev_handle_t GetHandle();

private:
    std::string tag = "I2cDevice";
    std::string deviceName;
    i2c_device_config_t devConfig;
    i2c_master_dev_handle_t devHandle;
};

#endif /* I2C_DEVICE_HPP_ */
