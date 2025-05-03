// main.cpp

#include <iostream>
#include <cstdio>
#include <cinttypes>
#include "sdkconfig.h"
#include "I2CManager.h"


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
    i2cManager.init();
    i2cManager.scanI2CDevices();
    i2cManager.addDevice(0x68); // Exemplo de endereço de dispositivo I2C
    // Lê dados do dispositivo I2C
    uint8_t data; // Buffer para armazenar os dados lidos
    i2cManager.readRegFromDevice(0x68, 0x75, &data, 1);
    // i2cManager.deInit();
}
