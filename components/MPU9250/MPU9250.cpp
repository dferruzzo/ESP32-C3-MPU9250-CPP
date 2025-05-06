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

    if (fullScaleSel == 0) {
        scale = 250.0f / 32768.0f; // 250 degrees/s
    } else if (fullScaleSel == 1) {
        scale = 500.0f / 32768.0f; // 500 degrees/s
    } else if (fullScaleSel == 2) {
        scale = 1000.0f / 32768.0f; // 1000 degrees/s
    } else if (fullScaleSel == 3) {
        scale = 2000.0f / 32768.0f; // 2000 degrees/s
    }
    
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

esp_err_t MPU9250::gyroRead(){

     // Suppress spell-check warnings for specific words
    if (!gyroCalibrated && !gyroCalibrationInProgress) {
        ESP_LOGW(TAG, "Gyroscope calibration not completed. Please calibrate the gyroscope first."); // "Gyroscope", "calibration", and "calibrate" are valid terms
        //return ESP_FAIL;
    }
    
    // Lê os dados do giroscópio
    uint8_t raw_data[6];

    if (i2cManager->readRegFromDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_GYRO_XOUT_H, raw_data, 6)==ESP_OK){

        if (gyroCalibrationInProgress){

            gyroData.x = (float)(((int16_t)((raw_data[0] << 8) | raw_data[1])) * scale);
            gyroData.y = (float)(((int16_t)((raw_data[2] << 8) | raw_data[3])) * scale);
            gyroData.z = (float)(((int16_t)((raw_data[4] << 8) | raw_data[5])) * scale);        

        } else {

            gyroData.x = -gyroBias.x + (float)(((int16_t)((raw_data[0] << 8) | raw_data[1])) * scale);
            gyroData.y = -gyroBias.y + (float)(((int16_t)((raw_data[2] << 8) | raw_data[3])) * scale);
            gyroData.z = -gyroBias.z + (float)(((int16_t)((raw_data[4] << 8) | raw_data[5])) * scale);
    
        }
    } else {
        ESP_LOGE("MPU9250", "Failed to read gyroscope data");
        return ESP_FAIL;
    }
    
    //ESP_LOGI("MPU9250", "Gyroscope Data: X: %.2f Y: %.2f Z: %.2f", gyroData.x, gyroData.y, gyroData.z);
    return ESP_OK;
}

esp_err_t MPU9250::gyroCalibrate(){

    // Inicia o processo de calibração do giroscópio
    if (gyroCalibrationInProgress) {
        ESP_LOGW(TAG, "Gyroscope calibration already in progress.");
        return ESP_FAIL;
    }

    gyroCalibrationInProgress = true;
    gyroBias.x = 0.0f;
    gyroBias.y = 0.0f;
    gyroBias.z = 0.0f;

    for (int i = 0; i < 100; i++) {
        gyroRead();
        gyroBias.x += gyroData.x;
        gyroBias.y += gyroData.y;
        gyroBias.z += gyroData.z;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    gyroBias.x /= 100.0f;
    gyroBias.y /= 100.0f;
    gyroBias.z /= 100.0f;

    ESP_LOGI(TAG, "Gyroscope bias: X: %.2f Y: %.2f Z: %.2f", gyroBias.x, gyroBias.y, gyroBias.z);

    // Finaliza o processo de calibração
    gyroCalibrationInProgress = false;
    gyroCalibrated = true;

    return ESP_OK;  

}

esp_err_t MPU9250::gyroGetRead(){
    
    ESP_LOGI(TAG, "Gyroscope Data (°/sec): X: %.2f Y: %.2f Z: %.2f", gyroData.x, gyroData.y, gyroData.z);
    return ESP_OK;

}


