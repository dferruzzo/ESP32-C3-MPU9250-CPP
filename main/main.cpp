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
    // Testando as configuracões do giro.
    mpu9250.gyroConfig(MPU9250_GYRO_FS_SEL_1000,  MPU9250_FCHOICE_B_GYRO_FILTER_ENABLED, MPU9250_GYRO_DLPF_CFG_20HZ);
    mpu9250.gyroCalibrate();

    while(1){
        mpu9250.gyroRead();
        mpu9250.gyroGetRead();
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    // mpu9250.getGyroFullScale();
    //  i2cManager.deInit();
}
