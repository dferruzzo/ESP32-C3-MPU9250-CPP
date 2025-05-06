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
        ESP_LOGE("MPU9250", "Device not found in I2C configuration. Address: 0x%02X", deviceAddress);
        return ESP_FAIL;
    } else {
        ESP_LOGI("MPU9250", "Device  0x%02X found in I2C configuration %p:", deviceAddress, MPU9250_handle_ptr);
    }

    MPU9250_mag_handle_ptr = i2cManager->isDeviceInConfig(deviceAddressMag);

    if (MPU9250_mag_handle_ptr == nullptr) {
        ESP_LOGE("MPU9250", "Device not found in I2C configuration. Address: 0x%02X", deviceAddressMag);
        return ESP_FAIL;
    } else {
        ESP_LOGI("MPU9250", "Device  0x%02X found in I2C configuration %p:", deviceAddressMag, MPU9250_mag_handle_ptr);
    }

    // Reset MPU9250
    MPU9250Reset();

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

esp_err_t MPU9250::MPU9250Reset(){
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

esp_err_t MPU9250::gyrConfig(uint8_t fullScaleSel, uint8_t enableFilter, uint8_t gyroDlpfCfg){

	if (fullScaleSel > 3) {
		ESP_LOGE(TAG, "Invalid FS_SEL value: %d. Must be between 0 and 3.", fullScaleSel);
		return ESP_FAIL;
	}

	if (enableFilter > 2) {
		ESP_LOGE(TAG, "Invalid ENABLE FILTER value: %d. Must be between 0 and 2.", enableFilter);
		return ESP_FAIL;
	}

	// Store the configuration value of the fullScaleSel for future use

    gyrFullScale = fullScaleSel;

    if (fullScaleSel == 0) {
        gyrScale = 250.0f / 32768.0f; // 250 degrees/s
    } else if (fullScaleSel == 1) {
        gyrScale = 500.0f / 32768.0f; // 500 degrees/s
    } else if (fullScaleSel == 2) {
        gyrScale = 1000.0f / 32768.0f; // 1000 degrees/s
    } else if (fullScaleSel == 3) {
        gyrScale = 2000.0f / 32768.0f; // 2000 degrees/s
    }
    
	// Set the gyroscope configuration, combine the full-scale range selection
    
	// with the FCHOICE_B bits (0x00 for DLPF enabled)

	uint8_t gyro_conf = (gyrFullScale << 3) | enableFilter;

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


void MPU9250::getGyrFullScale(){

	ESP_LOGI(TAG, "Gyroscope full scale: %d", gyrFullScale);

}

esp_err_t MPU9250::gyrRead(){

     // Suppress spell-check warnings for specific words
    if (!gyrCalibrated && !gyrCalibrationInProgress) {
        ESP_LOGW(TAG, "Gyroscope calibration not completed. Please calibrate the gyroscope first."); // "Gyroscope", "calibration", and "calibrate" are valid terms
        //return ESP_FAIL;
    }
    
    // Lê os dados do giroscópio
    uint8_t raw_data[6];

    if (i2cManager->readRegFromDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_GYRO_XOUT_H, raw_data, 6)==ESP_OK){

        if (gyrCalibrationInProgress){

            gyroData.x = (float)(((int16_t)((raw_data[0] << 8) | raw_data[1])) * gyrScale);
            gyroData.y = (float)(((int16_t)((raw_data[2] << 8) | raw_data[3])) * gyrScale);
            gyroData.z = (float)(((int16_t)((raw_data[4] << 8) | raw_data[5])) * gyrScale);        

        } else {

            gyroData.x = -gyroBias.x + (float)(((int16_t)((raw_data[0] << 8) | raw_data[1])) * gyrScale);
            gyroData.y = -gyroBias.y + (float)(((int16_t)((raw_data[2] << 8) | raw_data[3])) * gyrScale);
            gyroData.z = -gyroBias.z + (float)(((int16_t)((raw_data[4] << 8) | raw_data[5])) * gyrScale);
    
        }
    } else {
        ESP_LOGE("MPU9250", "Failed to read gyroscope data");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t MPU9250::gyrCalibrate(){

    // Inicia o processo de calibração do giroscópio
    if (gyrCalibrationInProgress) {
        ESP_LOGW(TAG, "Gyroscope calibration already in progress.");
        return ESP_FAIL;
    }

    gyrCalibrationInProgress = true;
    gyroBias.x = 0.0f;
    gyroBias.y = 0.0f;
    gyroBias.z = 0.0f;

    for (int i = 0; i < 100; i++) {
        gyrRead();
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
    gyrCalibrationInProgress = false;
    gyrCalibrated = true;

    return ESP_OK;  

}

esp_err_t MPU9250::gyrGetRead(){
    
    ESP_LOGI(TAG, "Gyroscope Data (°/sec): X: %.2f Y: %.2f Z: %.2f", gyroData.x, gyroData.y, gyroData.z);
    return ESP_OK;

}

esp_err_t MPU9250::accConfig(uint8_t fullScaleSel, uint8_t accelDlpfCfg){

    if (fullScaleSel > 3) {
        ESP_LOGE(TAG, "Invalid FS_SEL value: %d. Must be between 0 and 3.", fullScaleSel);
        return ESP_FAIL;
    }

    if (accelDlpfCfg > 15) {
        ESP_LOGE(TAG, "Invalid ENABLE FILTER value: %d. Must be between 0 and 15.", accelDlpfCfg);
        return ESP_FAIL;
    }

    accFullScale = fullScaleSel;

    if (fullScaleSel == 0) {
        accScale = 2.0f / 32768.0f; // 2g
    } else if (fullScaleSel == 1) {
        accScale = 4.0f / 32768.0f; // 4g
    } else if (fullScaleSel == 2) {
        accScale = 8.0f / 32768.0f; // 8g
    } else if (fullScaleSel == 3) {
        accScale = 16.0f / 32768.0f; // 16g
    }

    accFullScale = accFullScale << 3; // Configura o full scale do acelerômetro
    accDlpfSel = accelDlpfCfg; // Configura o DLPF do acelerômetro

    esp_err_t ret1, ret2;

    ret1 = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_ACCEL_CONFIG, &accFullScale, 1); // configura escala do acelerômetro

    ret2 = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_ACCEL_CONFIG_2, &accDlpfSel, 1); // configura DLPF
    
    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        ESP_LOGI(TAG, "Accelerometer configured with config = 0x%02X", accFullScale);
        ESP_LOGI(TAG, "DLPF configured with config = 0x%02X", accelDlpfCfg);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to configure accelerometer or DLPF");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t MPU9250::accRead() { 
    
    // Lê os dados do acelerômetro
    uint8_t raw_data[6];
    if (i2cManager->readRegFromDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_ACCEL_XOUT_H, raw_data, 6) == ESP_OK) {
        accData.x = (float)(((int16_t)((raw_data[0] << 8) | raw_data[1])) * accScale);
        accData.y = (float)(((int16_t)((raw_data[2] << 8) | raw_data[3])) * accScale);
        accData.z = (float)(((int16_t)((raw_data[4] << 8) | raw_data[5])) * accScale);
    } else {
        ESP_LOGE("MPU9250", "Failed to read accelerometer data");
        return ESP_FAIL;
    }

    return ESP_OK; 
}

esp_err_t MPU9250::accGetRead() { 
    
    ESP_LOGI(TAG, "Accelerometer Data (g): X: %.2f Y: %.2f Z: %.2f", accData.x, accData.y, accData.z);
    
    return ESP_OK; }


esp_err_t MPU9250::accCalibrate() { 
    
    /* 
    Falta implementar
    Precisa EIGEN
    */

    return ESP_OK; 
}

esp_err_t MPU9250::temRead(){

    // Lê os dados do sensor de temperatura
    uint8_t raw_data[2];
    if (i2cManager->readRegFromDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_TEMP_OUT_H, raw_data, 2) == ESP_OK) {
        int16_t temp_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        temData = (temp_raw / 333.87f) + 21.00f; // Conversão para graus 
    } else {
        ESP_LOGE("MPU9250", "Failed to read temperature data");
        return ESP_FAIL;
    }

    return ESP_OK;

}

esp_err_t MPU9250::temGetRead() { 
    
    ESP_LOGI(TAG, "Temperature Data (°C): Temp: %.2f", temData);
    
    return ESP_OK; 
}

//esp_err_t MPU9250::magConfig(){return ESP_OK;}

esp_err_t MPU9250::magRead(){
uint8_t pass_through = PASS_THROUGH_MODE;
uint8_t mag_single_measure = 0x01;

esp_err_t ret1, ret2;
ret1 = i2cManager->writeRegToDeviceWithHandle(
      *MPU9250_handle_ptr, MPU9250_INT_PIN_CFG, &pass_through,
      1);  // configura o modo pass-through
ret2 = i2cManager->writeRegToDeviceWithHandle(
      *MPU9250_mag_handle_ptr, MPU9250_MAG_CNTL, &mag_single_measure,1);  // configura o endereço do AK8963

if (ret1 != ESP_OK || ret2 != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure magnetometer");
    return ESP_FAIL;
}

 uint8_t data_ready = 0x00; // Buffer to store data ready status
    do{
//esp_err_t readRegFromDeviceWithHandle(i2c_master_dev_handle_t dev_handle, uint8_t reg_address, uint8_t* data, size_t length);
        i2cManager->readRegFromDeviceWithHandle(*MPU9250_mag_handle_ptr, MPU9250_MAG_DATA_RDY, &data_ready, 1); // Read data ready status
        } while((data_ready & 0x01) == 0); // Wait for data ready bit to be set

// Read magnetometer data
uint8_t raw_data[6]; //6 for magnetometer data 
if (i2cManager->readRegFromDeviceWithHandle(*MPU9250_mag_handle_ptr, MPU9250_MAG_HXL, raw_data, 6) == ESP_OK) {
    magData.x = (float)(((int16_t)((raw_data[1] << 8) | raw_data[0])) * magScale);
    magData.y = (float)(((int16_t)((raw_data[3] << 8) | raw_data[2])) * magScale);
    magData.z = (float)(((int16_t)((raw_data[5] << 8) | raw_data[4])) * magScale);
} else {
    ESP_LOGE("MPU9250", "Failed to read magnetometer data");
    return ESP_FAIL;
}
// Disable pass-through mode

pass_through = 0x00;
i2cManager->writeRegToDeviceWithHandle(*MPU9250_mag_handle_ptr, MPU9250_INT_PIN_CFG, &pass_through, 1);    // Turn-off PASS_THROUGH mode - Para ativar o Magnetómetro
return ESP_OK;
}
		
esp_err_t MPU9250::magGetRead(){

    ESP_LOGI(TAG, "Magnetometer Data (uT): X: %.2f Y: %.2f Z: %.2f", magData.x, magData.y, magData.z);

    return ESP_OK;
}
