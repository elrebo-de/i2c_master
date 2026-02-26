# I2cMaster component

This repository contains an ESP-IDF component for the OnBoard LED. It runs on
any ESP32 processor and is built using the ESP-IDF build system in version 5.5.0+.

The component is implemented as C++ classes `I2cMaster` and `I2cDevice`.

## Connecting the component

The constructor of class `I2cMaster` has four parameters:

| Parameter | Type of Parameter | Usage                             |
|:----------|:------------------|:----------------------------------|
| tag       | std::string       | the tag to be used in the ESP log |
| i2cPort   | i2c_port_num_t    | the i2c port number               |
| sclPin    | gpio_num_t        | the GPIO pin for SCL              |
| sdaPin    | gpio_num_t        | the GPIO pin for SDA              |

The constructor of class `I2cDevice` has five parameters:

| Parameter     | Type of Parameter   | Usage                             |
|:--------------|:--------------------|:----------------------------------|
| tag           | std::string         | the tag to be used in the ESP log |
| deviceName    | std::string         | the device name                   |
| devAddrLength | i2c_addr_bit_len_t  | the device address length         |
| deviceAddr    | uint16_t            | the device address                |
| sclSpeedHz    | uint32_t            | the clock speed in Hz             |

# Usage

You need to include `i2c_master.hpp`.

Then you define the I2C bus master by defining an instance of class `I2cMaster`.

For every I2C device you define an instance of class `I2cDevice` and add it to `I2cMaster` with method `AddDevice`.

Now the device handles are available through method `GetDeviceHandle(std::string deviceName)` of class `I2cMaster`.

To transmit data to an I2C device and to receive data from an I2C device you use the original ESP functions.

## API
The API of the component is located in the include directory files ```include/i2c_master.hpp``` and  ```include/i2c_device.hpp``` and defines the
C++ classes ```I2cMaster``` and ```I2cDevice```

```C
/* class I2cMaster
   Implementation of an I2C master bus for ESP IDF 5.5+
*/
class I2cMaster {
public:
    // Constructor of I2cMaster
	I2cMaster(std::string tag, i2c_port_num_t i2cPort, gpio_num_t sclPin, gpio_num_t sdaPin);
	virtual ~I2cMaster();

    i2c_master_bus_config_t GetConfig();
    i2c_master_bus_handle_t GetHandle();

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
```

# License

This component is provided under the Apache 2.0 license.
