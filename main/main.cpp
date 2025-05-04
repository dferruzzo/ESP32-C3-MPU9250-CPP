// main.cpp

#include <iostream>
#include <cstdio>
#include <cinttypes>
#include "sdkconfig.h"
#include "I2CManager.h"
#include "MPU9250.h"

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_chip_info.h"
    //#include "esp_flash.h"
    #include "esp_system.h"
    #include "esp_log.h"
    #include "esp_err.h"
    #include "driver/i2c_master.h"

}

extern "C" void app_main(void)
{
    // Inicializa o barramento I2C
    I2CManager i2cManager;
    //i2cManager.scanI2CDevices();
    i2cManager.addDevice(MPU9250_ADDRESS); 
    MPU9250 mpu9250(&i2cManager);

    // i2cManager.deInit();
}
