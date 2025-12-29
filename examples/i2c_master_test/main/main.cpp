/*
 * Example program to use an I2C LCD master bus with elrebo-de/i2c_master
 */

#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_master.hpp"

#include "sdkconfig.h"
#include "esp_log.h"

static const char *tag = "i2c master test";

extern "C" void app_main(void)
{
    ESP_LOGI(tag, "I2C Master Bus Test Program");

    /* Configure the I2C Master Bus */
    ESP_LOGI(tag, "I2cMaster");
    I2cMaster i2c(
		std::string("I2C Master Bus"), // tag
		(i2c_port_num_t) 0, // I2C_MASTER_NUM, // i2cPort
		#ifdef CONFIG_IDF_TARGET_ESP32C3
		(gpio_num_t) 6, // sclPin
		(gpio_num_t) 5 // sdaPin
		#elif CONFIG_IDF_TARGET_ESP32
		(gpio_num_t) 32, // sclPin
		(gpio_num_t) 26 // sdaPin
		#endif
    );

    // Add the MCP9808 device
    ESP_LOGI(tag, "I2cDevice Thermometer Device");
    // i2c_master_dev_handle_t thermometerHandle =
    i2c.AddDevice(new I2cDevice(
        std::string("thermometerDevice"), // tag
        std::string("Thermometer"), // deviceName
        (i2c_addr_bit_len_t) I2C_ADDR_BIT_LEN_7, // devAddrLength
        (uint16_t) 0x18, // deviceAddress
        (uint32_t) 50000 // sclSpeedHz
        )
    );

    // Add the LCD device
    //i2c_master_dev_handle_t lcdHandle =
    i2c.AddDevice(new I2cDevice(
        std::string("lcdDevice"), // tag
        std::string("LCD"), // deviceName
        (i2c_addr_bit_len_t) I2C_ADDR_BIT_LEN_7, // devAddrLength
        (uint16_t) 0x3c, // deviceAddress
        (uint32_t) 50000 // sclSpeedHz
        )
    );

//    int i = 0;

    // MCP9808
    float temperature;
    uint8_t set_resolution_buffer[2] = {0x08, 0x03};
    uint8_t request_temperature_buffer = 0x05;
    uint8_t temperature_buffer[2];

    // Set resolution to 0.0625°C
    ESP_ERROR_CHECK(i2c_master_transmit(i2c.GetDeviceHandle(std::string("Thermometer")), set_resolution_buffer, 2, -1));

    while(true) {
        //mcp9808
        // Read the temperature from MCP9808
        ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c.GetDeviceHandle(std::string("Thermometer")), &request_temperature_buffer, 1, temperature_buffer, 2, -1));

        temperature_buffer[0] = temperature_buffer[0] & 0x1F; //Clear flag bits
        if ((temperature_buffer[0] & 0x10) == 0x10){ //TA < 0°C
            temperature_buffer[0] = temperature_buffer[0] & 0x0F; //Clear SIGN
            temperature = 256. - (temperature_buffer[0] * 16. + temperature_buffer[1] / 16.);
        } else //TA > 0°C
        temperature = (temperature_buffer[0] * 16. + temperature_buffer[1] / 16.);
        //Temperature = Ambient Temperature (°C)

        printf("Temperature: %8.5f\n", temperature);

        vTaskDelay((1000 - 80) / portTICK_PERIOD_MS); // delay 1 second
    }
}
