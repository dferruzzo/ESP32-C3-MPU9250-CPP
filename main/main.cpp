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
    #include "esp_log.h"
    #include "esp_err.h"
    #include "driver/i2c_master.h"
}

#define I2C_MASTER_SDA_IO GPIO_NUM_6   
#define I2C_MASTER_SCL_IO GPIO_NUM_7
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_PORT I2C_NUM_0

class I2CManager {
public:
    I2CManager() = default;

    i2c_master_bus_handle_t handle;
    i2c_master_bus_config_t conf = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 10,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false,},
    };

    void init() {
        // Inicialização do driver I2C
        esp_err_t err = i2c_new_master_bus(&conf, &handle);
        if (err != ESP_OK) {
            ESP_LOGI("I2C", "Erro ao inicializar o I2C: %s", esp_err_to_name(err));    
            return;
        }
        ESP_LOGI("I2C", "I2C master bus initialized successfully");
    }
};

extern "C" void app_main(void)
{
    // Inicializa o barramento I2C
    I2CManager i2cManager;
    i2cManager.init();

}
