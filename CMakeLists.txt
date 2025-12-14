idf_component_register(SRCS "i2c_master.cpp" "i2c_device.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES "driver"
                             "esp_driver_gpio"
                             "esp_driver_i2c"
                             "esp_timer"
                    )

