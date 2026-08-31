/*
 * Example program to use an I2C LCD master bus with elrebo-de/i2c_master
 * to read data from a LC76G GNSS receiver
 * on an M5ATOM LITE
 */

#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp/esp-bsp.h"

#include "i2c_master.hpp"

#include "sdkconfig.h"
#include "esp_log.h"

std::string tag = "AXP2101 PMU test";

extern "C" void app_main(void)
{
    ESP_LOGI(tag.c_str(), "M5STACK CORE2 V1.1 Test Program");

    /* Configure the I2C Master Bus */
    ESP_LOGI(tag.c_str(), "I2cMaster");
    // set i2c_master_bus_handle from already initialized i2c_master_bus
    I2cMaster i2c(std::string("I2C Master Bus"), bsp_i2c_get_handle());

    // Add the AXP2101 PMU device
    ESP_LOGI(tag.c_str(), "I2cDevice AXP2101 PMU");
    i2c.AddDevice(new I2cDevice(
        std::string("AXP2101 PMU"), // tag
        std::string("AXP2101"), // deviceName
        (i2c_addr_bit_len_t) I2C_ADDR_BIT_LEN_7, // devAddrLength
        (uint16_t) 0x34, // deviceAddress
        (uint32_t) 400000 // sclSpeedHz
        )
    );

    I2cDevice* device = NULL;

    for(int i = 0; i < 10; i++) {
        // AXP2101
        device = i2c.GetDevice("AXP2101");

        uint8_t msb = device->ReadRegister(0x34);
        uint8_t lsb = device->ReadRegister(0x35);

        uint16_t battery_voltage_mv = (msb << 8) | (lsb);

        uint8_t battery_percentage = device->ReadRegister(0xA4); // Value directly represents %

        uint8_t status_reg = device->ReadRegister(0x01);
        uint8_t charge_status = status_reg & 0x07;

        // It is actively charging if the status state is between 0 and 3
        bool is_charging = (charge_status <= 3);

        ESP_LOGI(tag.c_str(), "%d: Battery Voltage: %d mV, Charge State: %s(%d) (%d %%)", i+1, battery_voltage_mv, is_charging ? "Charging" : "Discharging", charge_status, battery_percentage);

        vTaskDelay(10000 / portTICK_PERIOD_MS); // delay 10 seconds
    }
    // AXP2101: disconnect VBUS
    ESP_LOGI(tag.c_str(), "AXP2101: disconnect VBUS");
    uint8_t disconnectVbus[2] = {0x18, 0x01};
    ESP_ERROR_CHECK(device->Write(disconnectVbus, 2));
    vTaskDelay(100 / portTICK_PERIOD_MS); // delay 0.1 seconds

    // AXP2101: power off
    ESP_LOGI(tag.c_str(), "AXP2101: power off");
    vTaskDelay(100 / portTICK_PERIOD_MS); // delay 0.1 seconds
    uint8_t powerOff[2] = {0x10, 0x31};
    ESP_ERROR_CHECK(device->Write(powerOff, 2));
}
