#include "I2CManager.h"

I2CManager::I2CManager() {
    // Construtor
    // Inicialização do driver I2C
    bus_config.i2c_port = I2C_MASTER_PORT;
    bus_config.sda_io_num = I2C_MASTER_SDA_IO;
    bus_config.scl_io_num = I2C_MASTER_SCL_IO;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.intr_priority = 0;
    bus_config.trans_queue_depth = 0;
    bus_config.flags.enable_internal_pullup = true;
    bus_config.flags.allow_pd = false;
    init();
}

I2CManager::~I2CManager() {
    // Destrutor
    // Libere os recursos do barramento I2C, se necessário
    if (bus_handle) {
        esp_err_t err = i2c_del_master_bus(bus_handle);
        if (err != ESP_OK) {
            ESP_LOGE("I2C", "Erro ao liberar o I2C: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI("I2C", "I2C master bus released successfully");
        }
    }
}

void I2CManager::init() {
        // Inicialização do driver I2C
        esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
        if (err != ESP_OK) {
            ESP_LOGE("I2C", "Erro ao inicializar o I2C: %s", esp_err_to_name(err));    
            return;
        }

        // Inicializa o vetor de configurações de dispositivos
        for (int i = 0; i < deviceConfigs.size(); i++) {
            deviceConfigs[i].dev_addr_length = I2C_ADDR_BIT_LEN_7; // Endereço de 7 bits
            deviceConfigs[i].scl_speed_hz = I2C_MASTER_FREQ_HZ;   // Frequência do barramento
            ESP_LOGI("I2C", "I2C pre-config device %d", i); 
        }

        ESP_LOGI("I2C", "I2C master bus initialized successfully");
    }

void I2CManager::deInit() {
    // Libere os recursos do barramento I2C, se necessário
    
    for (auto& handle : deviceHandles) {
        if (handle != nullptr) {
            esp_err_t ret = i2c_master_bus_rm_device(handle);
            if (ret != ESP_OK) {
                ESP_LOGE("I2C", "Failed to remove device handle: %s", esp_err_to_name(ret));
            } else {
                ESP_LOGI("I2C", "Device handle removed successfully");
            }
            handle = nullptr;
        }
    }

    if (bus_handle) {
        esp_err_t err = i2c_del_master_bus(bus_handle);
        if (err != ESP_OK) {
            ESP_LOGE("I2C", "Erro ao liberar o I2C: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI("I2C", "I2C master bus released successfully");
        }
    }
}   

void I2CManager::scanI2CDevices() {

    ESP_LOGI("I2C_SCAN", "Scanning I2C bus...");

    esp_err_t ret;
    int devices_found = 0;

    for (uint8_t addr = 1; addr < 127; addr++) { // Endereços válidos no barramento I2C
        ret = i2c_master_probe(bus_handle, addr, I2C_MASTER_TIMEOUT_MS);
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

void I2CManager::addDevice(uint8_t address) {
    // Adiciona um dispositivo ao barramento I2C
    // Implementar a lógica para adicionar o dispositivo, se necessário
    if (devices_added >= I2C_NUM_MAX_DEVICES) {
        ESP_LOGE("I2C", "Maximum number of devices reached");
        return;
    }
    if (address < 1 || address > 127) {
        ESP_LOGE("I2C", "Invalid I2C address: 0x%02X", address);
        return;
    }
    if (deviceHandles[devices_added] != nullptr) {
        ESP_LOGE("I2C", "Device already added at address 0x%02X", address);
        return;
    }

    ESP_LOGI("I2C", "Adding device at address 0x%02X", address);
    
    deviceConfigs[devices_added].device_address = address;

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &deviceConfigs[devices_added], &deviceHandles[devices_added]);

    if (ret == ESP_OK) {

        ESP_LOGI("I2C", "Device at address 0x%02X added successfully", address);

        devices_added++;

    } else {
        
        ESP_LOGE("I2C", "Failed to add device at address 0x%02X: %s", address, esp_err_to_name(ret));

    }
}

esp_err_t I2CManager::readRegFromDevice(uint8_t dev_address, uint8_t reg_address, uint8_t* data, size_t length){
    // Lê dados de um dispositivo I2C
    if (dev_address < 1 || dev_address > 127) {
        ESP_LOGE("I2C", "Invalid I2C address: 0x%02X", dev_address);
        return ESP_ERR_INVALID_ARG;
    }
    if (length == 0) {
        ESP_LOGE("I2C", "Invalid length: %zu", length);
        return ESP_ERR_INVALID_ARG;
    }

    i2c_master_dev_handle_t deviceHandle = *isDeviceInConfig(dev_address);
    if (deviceHandle == nullptr) {
        ESP_LOGE("I2C", "Device with address 0x%02X not found in deviceConfigs", dev_address);
        return ESP_ERR_NOT_FOUND;
    }

    return readRegFromDeviceWithHandle(deviceHandle, reg_address, data, length);
}

esp_err_t I2CManager::readRegFromDeviceWithHandle(i2c_master_dev_handle_t dev_handle, uint8_t reg_address, uint8_t* data, size_t length){
    
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_address, 1, data, length, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    if (ret == ESP_OK) {
        ESP_LOGI("I2C", "Data 0x%02X read at register 0x%02X", *data, reg_address);
        return ret;

    } else {
        ESP_LOGE("I2C", "Failed to read data from device at address: %s", esp_err_to_name(ret));
        return ret;
    }
}

void I2CManager::writeRegToDevice(uint8_t dev_address, uint8_t reg_address, uint8_t* data, size_t length) {
    // Escreve dados em um dispositivo I2C
    if (dev_address < 1 || dev_address > 127) {
        ESP_LOGE("I2C", "Invalid I2C address: 0x%02X", dev_address);
        return;
    }
    if (length == 0) {
        ESP_LOGE("I2C", "Invalid length: %zu", length);
        return;
    }

    i2c_master_dev_handle_t deviceHandle = *isDeviceInConfig(dev_address);
    if (deviceHandle == nullptr) {
        ESP_LOGE("I2C", "Device with address 0x%02X not found in deviceConfigs", dev_address);
        return;
    }
    /*
    esp_err_t mpu9250_write_register(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data) {
        uint8_t write_buf[2] = {reg_addr, data};
        return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    }
    */
    uint8_t write_buf[2] = {reg_address, *data};
    esp_err_t ret = i2c_master_transmit(deviceHandle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to write register address to device at address 0x%02X: %s", dev_address, esp_err_to_name(ret));
        return;
    }

    if (ret == ESP_OK) {
        ESP_LOGI("I2C", "Data written to device at address 0x%02X at register 0x%02X", dev_address, reg_address);
    } else {
        ESP_LOGE("I2C", "Failed to write data to device at address 0x%02X: %s", dev_address, esp_err_to_name(ret));
    }
}
/*
esp_err_t I2CManager::writeRegToDeviceWithHandle(i2c_master_dev_handle_t dev_handle, uint8_t reg_address, uint8_t* data, size_t length){
}
*/
i2c_master_dev_handle_t* I2CManager::isDeviceInConfig(uint8_t address) {
    for (size_t i = 0; i < deviceConfigs.size(); i++) {
        if (deviceConfigs[i].device_address == address) {
            ESP_LOGI("I2C", "Device with address 0x%02X found in deviceHandle %p", address, &deviceHandles[i]);
            return &deviceHandles[i];
        }
    }
    ESP_LOGW("I2C", "Device with address 0x%02X not found in deviceConfigs", address);
    return nullptr;
}