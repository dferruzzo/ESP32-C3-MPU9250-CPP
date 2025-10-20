// main.cpp
#include "I2CManager.h"
#include "MPU9250.h"
//#include "NVSWrapper.h"
#include "sdkconfig.h"
#include <cinttypes>
#include <cstdio>
#include <iostream>
#include "pl_nvs.h"
#include "nvs_flash.h"
#include "NVSUtils.h"


extern "C" {
#include "esp_chip_info.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "esp_flash.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
}

extern "C" void app_main(void) {

    // Inicia o Non-Volatile Storage (NVS)
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    
    // cria a namespace "storage" no NVS
    PL::NvsNamespace nvs("storage", PL::NvsAccessMode::readWrite);
    
    // Inicializa o barramento I2C
    I2CManager i2cManager;
    // i2cManager.scanI2CDevices();
    i2cManager.addDevice(MPU9250_ADDRESS);
    i2cManager.addDevice(MPU9250_MAGNETOMETER_ADDR);

    MPU9250 mpu9250(&i2cManager);

    // Testando as configuracões do giro.
    
    mpu9250.gyrConfig(MPU9250_GYRO_FS_SEL_1000,
                    MPU9250_FCHOICE_B_GYRO_FILTER_ENABLED,
                    MPU9250_GYRO_DLPF_CFG_20HZ);

    mpu9250.forceGyroCalibration = false; // Força a calibração do giroscópio
    mpu9250.gyrCalibrate(nvs);

    // Testando conf do acelerômetro.
    mpu9250.accConfig(MPU9250_ACCEL_FS_SEL_4,
        MPU9250_ACCEL_NO_FIL_BW_1046Hz);

    mpu9250.forceAccCalibration = false; // Força a calibração do acelerômetro        
    mpu9250.accCalibrate(nvs);

    mpu9250.magCalibrate(nvs);
    
        while(1){
            mpu9250.gyrRead();
            //mpu9250.gyrGetRead();
            mpu9250.accRead();
            //mpu9250.accGetRead();
            mpu9250.temRead();
            //mpu9250.temGetRead();
            mpu9250.magRead();
            //mpu9250.magGetRead();
            mpu9250.printDataToTerminal();

            vTaskDelay(pdMS_TO_TICKS(250));
        }

   //write_and_read_3f_vector();
   //write_and_read_eigen_matrix();

    // Now loadedMat contains the matrix
    //  i2cManager.deInit();

}
