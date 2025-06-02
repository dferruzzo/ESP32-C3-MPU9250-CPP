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

void write_and_read_3f_vector() {
    // Inicializa o NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Open or create the NVS namespace "storage" in read/write mode
    PL::NvsNamespace nvs("storage", PL::NvsAccessMode::readWrite);

    // The vector to store
    float myVec[3] = {1.23f, 4.56f, 7.89f};

    // Write the vector as a blob
    esp_err_t err = nvs.Write("my_3f_vec", myVec, sizeof(myVec));
    if (err != ESP_OK) {
        printf("Failed to write vector to NVS: %d\n", err);
        return;
    }

    // Commit changes to ensure data is saved
    err = nvs.Commit();
    if (err != ESP_OK) {
        printf("Failed to commit NVS: %d\n", err);
        return;
    }

    // Read the vector back
    float readVec[3] = {0};
    size_t dataSize = sizeof(readVec);
    err = nvs.Read("my_3f_vec", readVec, sizeof(readVec), &dataSize);
    if (err != ESP_OK) {
        printf("Failed to read vector from NVS: %d\n", err);
        return;
    }

    printf("Read vector: %f, %f, %f\n", readVec[0], readVec[1], readVec[2]);
}

void write_and_read_eigen_matrix() {

    // Check if NVS is already initialized by calling nvs_flash_init()
    esp_err_t ret = nvs_flash_init();
    // If already initialized, nvs_flash_init() will return ESP_ERR_NVS_BASE or ESP_OK
    if (ret == ESP_ERR_NVS_BASE || ret == ESP_OK) {
        // NVS is already initialized or just initialized now
        ret = ESP_OK;
    } else if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    PL::NvsNamespace nvs("storage", PL::NvsAccessMode::readWrite);
    /*
    // Writing
    Eigen::MatrixXf mat(2, 3);
    mat << 1, 2, 3, 4, 5, 6;
    // Store the matrix in NVS 
    NVSUtils::WriteEigenMatrix(nvs, "my_matrix", mat);
    nvs.Commit();
    */
    
    // Reading
    Eigen::MatrixXf loadedMat;
    NVSUtils::ReadEigenMatrix(nvs, "my_matrix", loadedMat);

    std::cout << "loadedMat = " << std::endl << loadedMat << std::endl;
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
    /*
    /
    // le a matriz Eigen do NVS
    Eigen::MatrixXf loadedMat;
    NVSUtils::ReadEigenMatrix(nvs, "my_matrix", loadedMat);
    // Exibe a matriz carregada
    std::cout << "loadedMat = " << std::endl << loadedMat << std::endl;
    */

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
