#include "MPU9250.h"

MPU9250::MPU9250(I2CManager* i2cManager)
    : i2cManager(i2cManager) {
    init();
}

MPU9250::~MPU9250() {
    // Destrutor
    // Libera recursos, se necessário
}

esp_err_t MPU9250::init() {

    // Inicializa o dispositivo MPU9250
    MPU9250_handle_ptr = i2cManager->isDeviceInConfig(deviceAddress);
    if (MPU9250_handle_ptr == nullptr) {
        ESP_LOGE("MPU9250", "Device not found in I2C configuration");
        return ESP_FAIL;
    } else {
        ESP_LOGI("MPU9250", "Device found in I2C configuration %p:", MPU9250_handle_ptr);
    }

    // Inicializa o sensor MPU9250
    uint8_t data[1];
    esp_err_t ret = i2cManager->readRegFromDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_WHO_AM_I, data, 1);

    if (ret != ESP_OK) {
        ESP_LOGE("MPU9250", "Failed to read WHO_AM_I register");
        return ret;
    }
    if (data[0] != 0x71) { // Verifica o ID do dispositivo
        ESP_LOGE("MPU9250", "Invalid device ID: 0x%02X", data[0]);
        return ESP_FAIL;
    }
    ESP_LOGI("MPU9250", "MPU9250 initialized successfully");
    return ESP_OK;
}

esp_err_t MPU9250::gyroReset(){
    // Reseta o giroscópio
  
    uint8_t data = 1 << MPU9250_RESET_BIT; // Bit de reset
    esp_err_t ret = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_PWR_MGMT_1, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU9250", "Failed to reset gyroscope");
        return ret;
    }
    ESP_LOGI("MPU9250", "Gyroscope reset successfully");
    return ESP_OK;

}

esp_err_t MPU9250::gyroConfig(uint8_t fullScaleSel, uint8_t enableFilter, uint8_t gyroDlpfCfg){

	if (fullScaleSel > 3) {
		ESP_LOGE(TAG, "Invalid FS_SEL value: %d. Must be between 0 and 3.", fullScaleSel);
		return ESP_FAIL;
	}

	if (enableFilter > 2) {
		ESP_LOGE(TAG, "Invalid ENABLE FILTER value: %d. Must be between 0 and 2.", enableFilter);
		return ESP_FAIL;
	}
    	
	// Reset the gyroscope

	gyroReset();

	// Store the configuration value of the fullScaleSel for future use

    	gyroFullScale = fullScaleSel;
    
	// Set the gyroscope configuration, combine the full-scale range selection
    	
	// with the FCHOICE_B bits (0x00 for DLPF enabled)

	uint8_t gyro_conf = (gyroFullScale << 3) | enableFilter;

	esp_err_t ret1, ret2;

    	//esp_err_t writeRegToDeviceWithHandle(i2c_master_dev_handle_t dev_handle, uint8_t reg_address, uint8_t* data, size_t length);
	
	ret1 = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_GYRO_CONFIG, &gyro_conf, 1);	// configura escala do giroscópio
    	
	ret2 = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_CONFIG, &gyroDlpfCfg, 1);		// configura DLPF

	if (ret1 == ESP_OK && ret2 == ESP_OK) {
		ESP_LOGI(TAG, "Gyroscope configured with config = 0x%02X", gyro_conf);
		ESP_LOGI(TAG, "DLPF configured with config = 0x%02X", gyroDlpfCfg);
		return ESP_OK;
	} else {
		ESP_LOGE(TAG, "Failed to configure gyroscope or DLPF");
		return ESP_FAIL;
    }

}


void MPU9250::getGyroFullScale(){

	ESP_LOGI(TAG, "Gyroscope full scale: %d", gyroFullScale);

}

/*
esp_err_t MPU9250::readMPU9250Data(uint8_t reg_address, uint8_t* data, size_t length) {
    if (MPU9250_handle_ptr == nullptr) {
        ESP_LOGE("MPU9250", "Device not found in I2C configuration");
        return ESP_FAIL;
    }
    if (data == nullptr || length == 0) {
        ESP_LOGE("MPU9250", "Invalid data buffer or length");
        return ESP_ERR_INVALID_ARG;
    }
    if (reg_address < 0x00 || reg_address > 0x7F) {
        ESP_LOGE("MPU9250", "Invalid register address: 0x%02X", reg_address);
        return ESP_ERR_INVALID_ARG;
    }
    // Lê dados do sensor MPU9250
    esp_err_t ret = i2cManager->readRegFromDeviceWithHandle(*MPU9250_handle_ptr, reg_address, data, length);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU9250", "Failed to read sensor data");
        return ret;
    }
    ESP_LOGI("MPU9250", "Data read from register 0x%02X: 0x%02X", reg_address, *data);
    return ESP_OK;
    
}
  
esp_err_t MPU9250::writeSensorData(uint8_t reg_address, uint8_t* data, size_t length) {
    // Escreve dados no sensor MPU9250
    esp_err_t ret = i2cManager->writeRegToDevice(deviceAddress, reg_address, data, length);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU9250", "Failed to write sensor data");
        return ret;
    }
    return ESP_OK;
}
    */
