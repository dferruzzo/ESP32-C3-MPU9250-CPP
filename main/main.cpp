// main.cpp

#include <iostream>
#include <cstdio>
#include <cinttypes>
#include "sdkconfig.h"

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_chip_info.h"
    //#include "esp_flash.h"
    #include "esp_system.h"
    #include "driver/i2c.h"
}

#define I2C_MASTER_SDA_IO GPIO_NUM_6   
#define I2C_MASTER_SCL_IO GPIO_NUM_7
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_PORT I2C_NUM_0

void initI2C()
{
    // Configuração do driver I2C
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;                      // Modo mestre
    conf.sda_io_num = I2C_MASTER_SDA_IO;              // Pino SDA
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;          // Pull-up habilitado para SDA
    conf.scl_io_num = I2C_MASTER_SCL_IO;              // Pino SCL
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;          // Pull-up habilitado para SCL
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;       // Frequência do barramento

    // Instalação do driver I2C
    esp_err_t err = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (err != ESP_OK) {
        std::cerr << "Erro ao configurar parâmetros do I2C: " << esp_err_to_name(err) << std::endl;
        return;
    }

    err = i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        std::cerr << "Erro ao instalar driver I2C: " << esp_err_to_name(err) << std::endl;
        return;
    }

    std::cout << "I2C inicializado com sucesso!" << std::endl;
}

extern "C" void app_main(void)
{
    std::cout << "Hello world!" << std::endl;

    // Inicializa o barramento I2C
    initI2C();

}
