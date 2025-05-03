#include "I2CManager.h"

#define I2C_MASTER_SDA_IO GPIO_NUM_6   
#define I2C_MASTER_SCL_IO GPIO_NUM_7
#define I2C_MASTER_PORT I2C_NUM_0

I2CManager::I2CManager() {
    // Construtor
    // Inicialização do driver I2C
    conf.i2c_port = I2C_MASTER_PORT;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.clk_source = I2C_CLK_SRC_DEFAULT;
    conf.glitch_ignore_cnt = 7;
    conf.intr_priority = 0;
    conf.trans_queue_depth = 0;
    conf.flags.enable_internal_pullup = true;
    conf.flags.allow_pd = false;
}

I2CManager::~I2CManager() {
    // Destrutor
    // Libere os recursos do barramento I2C, se necessário
    if (handle) {
        esp_err_t err = i2c_del_master_bus(handle);
        if (err != ESP_OK) {
            ESP_LOGE("I2C", "Erro ao liberar o I2C: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI("I2C", "I2C master bus released successfully");
        }
    }
}

void I2CManager::init() {
        // Inicialização do driver I2C
        esp_err_t err = i2c_new_master_bus(&conf, &handle);
        if (err != ESP_OK) {
            ESP_LOGE("I2C", "Erro ao inicializar o I2C: %s", esp_err_to_name(err));    
            return;
        }
        ESP_LOGI("I2C", "I2C master bus initialized successfully");
    }

void I2CManager::scanI2CDevices() {

    ESP_LOGI("I2C_SCAN", "Scanning I2C bus...");

    esp_err_t ret;
    int devices_found = 0;

    for (uint8_t addr = 1; addr < 127; addr++) { // Endereços válidos no barramento I2C
        ret = i2c_master_probe(handle, addr, I2C_MASTER_TIMEOUT_MS);
        if (ret == ESP_ERR_INVALID_ARG) {
            ESP_LOGW("I2C_SCAN", "Invalid address: 0x%02X", addr);
            continue;
        } else if (ret == ESP_ERR_TIMEOUT) {
            // Timeout, no device found at this address
            continue;
        }
        // If we reach here, the device is found
        if (ret == ESP_OK) {
            ESP_LOGI("I2C_SCAN", "Device found at address 0x%02X", addr);
            devices_found++;
		}// else if (ret != ESP_ERR_TIMEOUT) {
		//	ESP_LOGW("I2C_SCAN", "Error at address 0x%02X: %s", addr, esp_err_to_name(ret));
		//}
	}

    if (devices_found == 0) {
        ESP_LOGI("I2C_SCAN", "No I2C devices found");
    } else {
        ESP_LOGI("I2C_SCAN", "Scan complete, %d device(s) found", devices_found);
    }
}
