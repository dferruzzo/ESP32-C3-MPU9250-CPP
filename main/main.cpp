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
#include "esp_timer.h"

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

    /* Inicia o Non-Volatile Storage (NVS) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
        	nvs_flash_erase();
        	nvs_flash_init();
    }
    /* cria a namespace "storage" no NVS */
    PL::NvsNamespace nvs("storage", PL::NvsAccessMode::readWrite);
    /* Inicializa o barramento I2C */
    I2CManager i2cManager;
    // i2cManager.scanI2CDevices();
    i2cManager.addDevice(MPU9250_ADDRESS);
    /* Init MPU9250 */ 
    MPU9250 mpu9250(&i2cManager);
    /* Configura os sensores */
    mpu9250.sensorsConfig(); 
    /* Força calibração: giroscópio, acelerômetro, magnetômetro */
    //mpu9250.forceCalibration(true, true, true);
    /* Calibra sensores */
    mpu9250.sensorsCalibrate(nvs); 
    /* Loop principal */
    int64_t start_time, end_time;
    while(true){
	    start_time = esp_timer_get_time();
	    mpu9250.sensorsRead();
	    end_time = esp_timer_get_time();
	    printf("Tempo decorrido: %lld (us), freq: %.6fx1e6 (hz)\n", (end_time - start_time), 1.0f/(end_time - start_time));
	    mpu9250.printDataToTerminal();
	    vTaskDelay(pdMS_TO_TICKS(250));
    }
    // Now loadedMat contains the matrix
    //  i2cManager.deInit();
}
