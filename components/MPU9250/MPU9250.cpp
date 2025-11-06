#include "MPU9250.h"

MPU9250::MPU9250(I2CManager* i2cManager)
    : i2cManager(i2cManager) {
    init();
}

MPU9250::~MPU9250() 
{
    // Destrutor
    // Libera recursos, se necessário
}

esp_err_t MPU9250::init() 
{

    // Inicializa o dispositivo MPU9250
    this->MPU9250_handle_ptr = i2cManager->isDeviceInConfig(deviceAddress);
    if (this->MPU9250_handle_ptr == nullptr) {
        ESP_LOGE(TAG, "Device not found in I2C configuration. Address: 0x%02X", deviceAddress);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "Device  0x%02X found in I2C configuration %p:", deviceAddress, MPU9250_handle_ptr);
    }
/*
    MPU9250_mag_handle_ptr = i2cManager->isDeviceInConfig(deviceAddressMag);
    if (MPU9250_mag_handle_ptr == nullptr) {
        ESP_LOGE(TAG, "Device not found in I2C configuration. Address: 0x%02X", deviceAddressMag);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "Device  0x%02X found in I2C configuration %p:", deviceAddressMag, MPU9250_mag_handle_ptr);
    }
*/
    // Reset MPU9250
    MPU9250Reset();

    // Inicializa o sensor MPU9250
    uint8_t data[1];
    esp_err_t ret = i2cManager->readRegFromDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_WHO_AM_I, data, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }
    if (data[0] != 0x71) { // Verifica o ID do dispositivo
        ESP_LOGE(TAG, "Invalid device ID: 0x%02X", data[0]);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "MPU9250 initialized successfully");

    return ESP_OK;
}

esp_err_t MPU9250::MPU9250Reset()
{
    // Reseta o MPU9250

    if (MPU9250_handle_ptr == nullptr) 
    {
        ESP_LOGE(TAG, "MPU9250_handle_ptr is null, cannot reset MPU9250");
        return ESP_FAIL;
    }

    uint8_t data = 1 << MPU9250_RESET_BIT; // Bit de reset
    esp_err_t ret = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_PWR_MGMT_1, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset MPU9250");
        return ret;
    }
    ESP_LOGI(TAG, "MPU9250 reset successfully");
    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait 100ms for reset to complete
    return ESP_OK;

}

esp_err_t MPU9250::enableI2CMaster() 
{
	// Enable I2C master mode
    	uint8_t user_ctrl = 0x20; // I2C_MST_EN bit
    	esp_err_t ret = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, 
			MPU9250_USER_CTRL, &user_ctrl, 1);
    	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to enable I2C master mode");
        	return ret;
    	}
    	// Set I2C master clock speed (400kHz)
    	uint8_t i2c_mst_ctrl = 0x0D; // 400kHz
    	ret = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, 
			MPU9250_I2C_MST_CTRL, &i2c_mst_ctrl, 1);
    	if (ret != ESP_OK) {
        	ESP_LOGE(TAG, "Failed to set I2C master clock");
        	return ret;
    	}
    	this->I2CMasterEnabled = true;
	return ESP_OK;
}

esp_err_t MPU9250::writeAK8963RegisterViaSLV0(uint8_t ak8963_reg, uint8_t data) 
{
    // Step 1: Configure SLV0 address (AK8963 address + write bit)
    uint8_t slv0_addr = MPU9250_MAGNETOMETER_ADDR; // 0x0C (write mode, bit 7 = 0)
    esp_err_t ret = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, 
                                                          MPU9250_I2C_SLV0_ADDR, &slv0_addr, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set SLV0 address");
        return ret;
    }
    
    // Step 2: Set the AK8963 register address to write to
    ret = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, 
                                                MPU9250_I2C_SLV0_REG, &ak8963_reg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set SLV0 register");
        return ret;
    }
    
    // Step 3: Write the data to the output register
    ret = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, 
                                                MPU9250_I2C_SLV0_DO, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set SLV0 data");
        return ret;
    }
    // Step 4: Enable SLV0 and set length to 1 byte
    uint8_t slv0_ctrl = 0x81; // I2C_SLV0_EN (bit 7) + 1 byte length
    ret = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, 
                                                MPU9250_I2C_SLV0_CTRL, &slv0_ctrl, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable SLV0");
        return ret;
    }
    
    // Wait for the transaction to complete
    vTaskDelay(pdMS_TO_TICKS(10));

// Disable SLV0 after the write to avoid occupying EXT_SENS data slots
    uint8_t slv0_disable = 0x00;
    ret = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr,
                                                MPU9250_I2C_SLV0_CTRL, &slv0_disable, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable SLV0 after write");
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t MPU9250::readMagnetometerASAViaSLV1()
{
    ESP_LOGI(TAG, "Reading magnetometer ASA values via SLV1...");

    // First, ensure I2C master is enabled
    esp_err_t ret;
    if (!this->I2CMasterEnabled) {
	    ret = enableI2CMaster();
	    if (ret != ESP_OK) {
		    ESP_LOGE(TAG, "Failed to enable I2C master");
		    return ret;
	    }
    }

    // Configure SLV1 for reading ASA values from AK8963
    
    // Step 1: Set SLV1 address for reading (AK8963 address + read bit)
    // MPU9250_I2C_SLV1_ADDR = 0x28
    // MPU9250_MAGNETOMETER_ADDR = 0x0C	= 00001100
    // 0x80 = 10000000
    uint8_t slv1_addr = MPU9250_MAGNETOMETER_ADDR | 0x80; // 0x8C (read mode, bit 7 = 1)
    ret = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, 
                                                MPU9250_I2C_SLV1_ADDR, &slv1_addr, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set SLV1 address for ASA read");
        return ret;
    }
    else {
	ESP_LOGI(TAG, "SLV1 address for ASA read set to 0x%02X", slv1_addr);
    }
    
    // Step 2: Set register address to start reading from (ASAX)
    uint8_t asa_start_reg = AK8963_ASAX;
    ret = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, 
                                                MPU9250_I2C_SLV1_REG, &asa_start_reg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set SLV1 register for ASA read");
        return ret;
    }
    else {
	ESP_LOGI(TAG, "SLV1 register for ASA read set to 0x%02X", asa_start_reg);
    }
    
    // Step 3: Enable SLV1 and set length to 3 bytes (ASAX, ASAY, ASAZ)
    // MPU9250_I2C_SLV1_CTRL = 0x2A
    uint8_t slv1_ctrl = 0x83;	// I2C_SLV1_EN (bit 7) + 3 bytes length
				// 0x80 = 10000000 + 0x03 = 1000011  = 10000011 = 0x83
    ret = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, 
                                                MPU9250_I2C_SLV1_CTRL, &slv1_ctrl, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable SLV1 for ASA read");
        return ret;
    }
    else {
	ESP_LOGI(TAG, "SLV1 enabled for ASA read with control byte 0x%02X", slv1_ctrl);
    }
    
    // Step 4: Wait for the I2C master to complete the transaction
    // Check I2C master status register to ensure transaction completed
    uint8_t i2c_mst_status = 0;
    int timeout_count = 0;
    const int MAX_TIMEOUT = 100; // 1 second timeout
    //  MPU9250_I2C_MST_STATUS = 0x36
    do {
        vTaskDelay(pdMS_TO_TICKS(10));
        ret = i2cManager->readRegFromDeviceWithHandle(*MPU9250_handle_ptr, 
                                                     MPU9250_I2C_MST_STATUS, &i2c_mst_status, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read I2C master status");
            return ret;
        }
	else {
		ESP_LOGI(TAG, "I2C master status: 0x%02X", i2c_mst_status);	
	}
        timeout_count++;
    } while ((i2c_mst_status & 0x02) && timeout_count < MAX_TIMEOUT); // Wait for SLV1_NACK bit to be set or clear
    
    if (timeout_count >= MAX_TIMEOUT) {
        ESP_LOGE(TAG, "Timeout waiting for SLV1 transaction to complete");
        return ESP_FAIL;
    }
    
    // Additional delay to ensure data is available
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // Step 5: Read the ASA data from external sensor data registers
    // Note: SLV1 data starts at EXT_SENS_DATA_00 (SLV0 uses 00-02)
    uint8_t asa_data[3];
    ret = i2cManager->readRegFromDeviceWithHandle(*MPU9250_handle_ptr, 
                                                 MPU9250_EXT_SENS_DATA_00, asa_data, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ASA data from external sensor registers");
        return ret;
    }
    else {
	ESP_LOGI(TAG, "ASA data read: X=%d, Y=%d, Z=%d", asa_data[0], asa_data[1], asa_data[2]);
    }
    
    // Disable SLV1 to free it for other operations
    uint8_t slv1_disable = 0x00;
    i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, 
                                          MPU9250_I2C_SLV1_CTRL, &slv1_disable, 1);
    
    // Check if ASA data is valid (should not be all zeros)
    if (asa_data[0] == 0 && asa_data[1] == 0 && asa_data[2] == 0) {
        ESP_LOGE(TAG, "ASA data is all zeros - magnetometer may not be in Fuse ROM mode");
        return ESP_FAIL;
    }
    
    // Validate ASA data (should be >= 128)
    for (int i = 0; i < 3; i++) {
        if (asa_data[i] < 128) {
            ESP_LOGW(TAG, "Suspicious ASA value[%d]: %d (expected >= 128)", i, asa_data[i]);
        }
    }
    
    // Calculate adjusted sensitivity
    float asax = (asa_data[0] - 128) / 256.0f + 1.0f;
    float asay = (asa_data[1] - 128) / 256.0f + 1.0f;
    float asaz = (asa_data[2] - 128) / 256.0f + 1.0f;
    
    // Apply to scale factors
    this->magScaleX = this->magScale * asax;
    this->magScaleY = this->magScale * asay;
    this->magScaleZ = this->magScale * asaz;
    
    ESP_LOGI(TAG, "ASA raw values: X=%d, Y=%d, Z=%d", asa_data[0], asa_data[1], asa_data[2]);
    ESP_LOGI(TAG, "ASA adjustment factors: X=%.3f, Y=%.3f, Z=%.3f", asax, asay, asaz);
    ESP_LOGI(TAG, "Adjusted scales: X=%.6f, Y=%.6f, Z=%.6f μT/LSB", 
             magScaleX, magScaleY, magScaleZ);
    
    this->magReadMagASA = true;
    return ESP_OK;
}

esp_err_t MPU9250::gyrConfig(uint8_t fullScaleSel, uint8_t enableFilter, uint8_t gyroDlpfCfg)
{

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

void MPU9250::getGyrFullScale()
{

	ESP_LOGI(TAG, "Gyroscope full scale: %d", gyrFullScale);

}

esp_err_t MPU9250::gyrRead()
{

     // Suppress spell-check warnings for specific words
    if (!gyrCalibrated && !gyrCalibrationInProgress) {
        ESP_LOGW(TAG, "Gyroscope calibration not completed. Please calibrate the gyroscope first."); // "Gyroscope", "calibration", and "calibrate" are valid terms
        //return ESP_FAIL;
    }
    
    // Lê os dados do giroscópio
    uint8_t raw_data[6];
    Vec3f gyrRawData = Vec3f(0.0f, 0.0f, 0.0f);

    if (i2cManager->readRegFromDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_GYRO_XOUT_H, raw_data, 6)==ESP_OK){

        gyrRawData.x = (float)(((int16_t)((raw_data[0] << 8) | raw_data[1])) * gyrScale);
        gyrRawData.y = (float)(((int16_t)((raw_data[2] << 8) | raw_data[3])) * gyrScale);
        gyrRawData.z = (float)(((int16_t)((raw_data[4] << 8) | raw_data[5])) * gyrScale);

    } else {
        ESP_LOGE(TAG, "Failed to read gyroscope data");
        return ESP_FAIL;
    }

    if (gyrCalibrationInProgress){

        gyroData.x = gyrRawData.x;
        gyroData.y = gyrRawData.y;
        gyroData.z = gyrRawData.z;

    } else {

        gyroData.x = -gyroBias.x + gyrRawData.x;
        gyroData.y = -gyroBias.y + gyrRawData.y;
        gyroData.z = -gyroBias.z + gyrRawData.z;

    }   
    
    return ESP_OK;
}

esp_err_t MPU9250::gyrCalibrate(PL::NvsNamespace& nvs)
{
    // Check if gyroscope bias is already stored in NVS to skip calibration
    bool gyroBiasStored = false;
    NVSUtils::ReadBool(nvs, "gyroBiasStored", gyroBiasStored); // <-- Use NVSUtils here
    if (gyroBiasStored && !forceGyroCalibration) {
        ESP_LOGI(TAG, "Gyroscope bias already stored in NVS. Skipping calibration.");
        // Read the bias values from NVS
        NVSUtils::ReadFloat(nvs, "gyroBias_x", gyroBias.x);
        NVSUtils::ReadFloat(nvs, "gyroBias_y", gyroBias.y);
        NVSUtils::ReadFloat(nvs, "gyroBias_z", gyroBias.z);
        gyrCalibrated = true; // Set the calibration flag to true
        gyrCalibrationInProgress = false;
        ESP_LOGI(TAG, "Gyroscope bias: X: %.2f Y: %.2f Z: %.2f", gyroBias.x, gyroBias.y, gyroBias.z);
        return ESP_OK; // Skip calibration if bias is already stored
    }
    // If the forceGyroCalibration flag is set, skip reading from NVS
    if (forceGyroCalibration) {
        ESP_LOGI(TAG, "Forcing gyroscope calibration.");
        gyroBiasStored = false; // Force calibration
    }
    // Inicia o processo de calibração do giroscópio
    if (gyrCalibrationInProgress) {
        ESP_LOGW(TAG, "Gyroscope calibration already in progress.");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "*******************************************************");
    ESP_LOGI(TAG, "Iniciando a calibração do giroscópio...");
    ESP_LOGI(TAG, "*******************************************************");
    ESP_LOGI(TAG, "Coloque o giroscópio em uma superfície plana e estável.");
    timer(5);

    // Reset calibration flags and bias values
    gyrCalibrated = false;
    gyrCalibrationInProgress = true;

    gyroBias.x = 0.0f;
    gyroBias.y = 0.0f;
    gyroBias.z = 0.0f;

    ESP_LOGI(TAG, "*******************************************************");
    for (int i = 0; i < 100; i++) {
    
    if (gyrRead() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read gyroscope data during calibration at sample %d", i);
            // Optionally: break or retry this iteration
            return ESP_FAIL;
        }

        gyroBias.x += gyroData.x;
        gyroBias.y += gyroData.y;
        gyroBias.z += gyroData.z;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    gyroBias.x /= 100.0f;
    gyroBias.y /= 100.0f;
    gyroBias.z /= 100.0f;

    // Store the bias values in NVS
    NVSUtils::WriteFloat(nvs, "gyroBias_x", gyroBias.x);
    NVSUtils::WriteFloat(nvs, "gyroBias_y", gyroBias.y);
    NVSUtils::WriteFloat(nvs, "gyroBias_z", gyroBias.z);
    NVSUtils::WriteBool(nvs, "gyroBiasStored", true); // Set the flag to indicate bias is stored
    //commit the changes to NVS
    nvs.Commit();
    
    ESP_LOGI(TAG, "Gyroscope bias stored in NVS.");

    ESP_LOGI(TAG, "Gyroscope bias: X: %.2f Y: %.2f Z: %.2f", gyroBias.x, gyroBias.y, gyroBias.z);
    timer(2);


    // Finaliza o processo de calibração
    gyrCalibrated = true;
    gyrCalibrationInProgress = false;
    ESP_LOGI(TAG, "Gyroscope calibration completed successfully.");
    ESP_LOGI(TAG, "*******************************************************");
    ESP_LOGI(TAG, "Gyroscope calibration data stored in NVS.");
    ESP_LOGI(TAG, "*******************************************************");
    

    return ESP_OK;  

}

esp_err_t MPU9250::gyrGetRead()
{
    
    ESP_LOGI(TAG, "Gyroscope Data (°/sec): X: %.2f Y: %.2f Z: %.2f", gyroData.x, gyroData.y, gyroData.z);
    return ESP_OK;

}

esp_err_t MPU9250::accConfig(uint8_t fullScaleSel, uint8_t accelDlpfCfg)
{

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

    // Init variables for calibration
    this->accCalibrationMatrix = Eigen::MatrixXf::Zero(4, 3);
    return ESP_OK;
}

esp_err_t MPU9250::accRead() 
{ 
    
    if (!accCalibrated && !accCalibrationInProgress) {
        ESP_LOGW(TAG, "Accelerometer calibration not completed. Please calibrate the accelerometer first.");
        //return ESP_FAIL;
    }

    // Lê os dados do acelerômetro
    uint8_t raw_data[6];
    if (accCalibrationInProgress){
        if (i2cManager->readRegFromDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_ACCEL_XOUT_H, raw_data, 6) == ESP_OK) {
            accData.x = (float)(((int16_t)((raw_data[0] << 8) | raw_data[1])) * accScale);
            accData.y = (float)(((int16_t)((raw_data[2] << 8) | raw_data[3])) * accScale);
            accData.z = (float)(((int16_t)((raw_data[4] << 8) | raw_data[5])) * accScale);
        } else {
            ESP_LOGE(TAG, "Failed to read accelerometer data");
            return ESP_FAIL;
        }
    } else{

        if (i2cManager->readRegFromDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_ACCEL_XOUT_H, raw_data, 6) == ESP_OK) {
            accData.x = (float)(((int16_t)((raw_data[0] << 8) | raw_data[1])) * accScale);
            accData.y = (float)(((int16_t)((raw_data[2] << 8) | raw_data[3])) * accScale);
            accData.z = (float)(((int16_t)((raw_data[4] << 8) | raw_data[5])) * accScale);

            Eigen::RowVector4f accVec;
            accVec << accData.x, accData.y, accData.z, 1.0f; // Última coluna igual a 1
            
            Eigen::RowVector3f result = accVec * accCalibrationMatrix;
            accData.x = result(0);
            accData.y = result(1);
            accData.z = result(2);

        } else {
            ESP_LOGE("MPU9250", "Failed to read accelerometer data");
            return ESP_FAIL;
        }

    }

    return ESP_OK; 
}

esp_err_t MPU9250::accGetRead() 
{ 
    
    ESP_LOGI(TAG, "Accelerometer Data (g): X: %.2f Y: %.2f Z: %.2f", accData.x, accData.y, accData.z);
    
    return ESP_OK; 
}

esp_err_t MPU9250::accCalibrate(PL::NvsNamespace& nvs) 
{ 
    bool accCalibrationStored = false;
    esp_err_t accCalStrError = NVSUtils::ReadBool(nvs, "accCalStr", accCalibrationStored); // Use NVSUtils to read the flag
    if ((accCalStrError==ESP_OK) && accCalibrationStored && !forceAccCalibration) {
        ESP_LOGI(TAG, "Accelerometer calibration already stored in NVS. Skipping calibration.");
        // Read the calibration matrix from NVS
        NVSUtils::ReadEigenMatrix(nvs, "accCalMat", accCalibrationMatrix);
        // Set the calibration flag to true
        accCalibrated = true; 
        accCalibrationInProgress = false;
        ESP_LOGI(TAG, "Accelerometer calibration matrix loaded from NVS.");
        return ESP_OK; // Skip calibration if matrix is already stored
    }else if (accCalStrError != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read accelerometer calibration status from NVS: %s", esp_err_to_name(accCalStrError));
        //return accCalStrError; // Return error if reading from NVS failed
    }

    if(forceAccCalibration) {
        ESP_LOGI(TAG, "Forcing accelerometer calibration.");
        accCalibrationStored = false; // Force calibration
    }

    accCalibrationInProgress = true;
    
    // Create matrix Y 
    Eigen::MatrixXf Y(6 * accNumSamplesCal, 3);
    Y.setZero(); // Optional: initialize all elements to zero
    
    // Create an empty matrix W with 6*numSamples rows and 4 columns
    Eigen::MatrixXf W(6 * accNumSamplesCal, 4);
    W.setZero(); // Optional: initialize all elements to zero

    // Populate matrix Y
    for (int i = 0; i < 6; ++i) {
        int axis = i / 2;                // 0: X, 1: Y, 2: Z
        float sign = (i % 2 == 0) ? 1.0f : -1.0f; // Even: +1, Odd: -1
        Y.block(i * accNumSamplesCal, 0, accNumSamplesCal, 3).col(axis).setConstant(sign);
    }

    //std::cout << "Y = " << std::endl << Y << std::endl;

    ESP_LOGI(TAG, "*******************************************************");
    ESP_LOGI(TAG, "Iniciando a calibração do acelerômetro...");
    ESP_LOGI(TAG, "*******************************************************");

    // Collecting N Samples with x-axis pointing downwards
    ESP_LOGI(TAG,
            "Coloque o acelerômetro com o eixo x apontando para cima e mantenha essa posição.");
    timer(5);
    ESP_LOGI(TAG, "Coletando amostras...");

    int count = 0;
    while (count < accNumSamplesCal) {

        accRead();  // Lê os dados do acelerômetro
        if ((accData.x > 0) && 
        (accData.x > (float)(accFactorCal*std::abs(accData.y))) && 
        (accData.x > (float)(accFactorCal*std::abs(accData.z))))
        {  // Adicionar que o módulo de x seja maior que o modulo de y e o modulo de z
            W(count, 0) = accData.x;
            W(count, 1) = accData.y;
            W(count, 2) = accData.z;
            W(count, 3) = 1.0f;  // Última coluna igual a 1
            count++;
        } else {
            ESP_LOGI(TAG, "Eixo x não está apontando para cima.");
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);  // Delay de 10ms entre as leituras
        // Caso contrário, rejeita a amostra e não incrementa count
    }
    ESP_LOGI(TAG, "Amostras coletadas: %d", count);
    ESP_LOGI(TAG, "*******************************************************");
    ESP_LOGI(TAG, "Coloque o acelerômetro com o eixo x apontando para baixo e mantenha essa posição.");
    timer(5);
    ESP_LOGI(TAG, "Coletando amostras...");
    count = 0;
    while (count < accNumSamplesCal) {

        accRead();  // Lê os dados do acelerômetro
        if ((accData.x < 0) && 
        (accData.x < (float)(-accFactorCal*std::abs(accData.y))) && 
        (accData.x < (float)(-accFactorCal*std::abs(accData.z))))
        {  // Adicionar que o módulo de x seja maior que o modulo de y e o modulo de z
            W(count + accNumSamplesCal, 0) = accData.x;
            W(count + accNumSamplesCal, 1) = accData.y;
            W(count + accNumSamplesCal, 2) = accData.z;
            W(count + accNumSamplesCal, 3) = 1.0f;  // Última coluna igual a 1
            count++;
        } else {
            ESP_LOGI(TAG, "Eixo x não está apontando para baixo.");
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);  // Delay de 10ms entre as leituras
        // Caso contrário, rejeita a amostra e não incrementa count
    }
    ESP_LOGI(TAG, "Amostras coletadas: %d", count);
    ESP_LOGI(TAG, "*******************************************************");
    // Collecting N Samples with y-axis pointing downwards
    ESP_LOGI(TAG,
            "Coloque o acelerômetro com o eixo y apontando para cima e mantenha essa posição.");
    timer(5);
    ESP_LOGI(TAG, "Coletando amostras...");
    count = 0;
    while (count < accNumSamplesCal) {

        accRead();  // Lê os dados do acelerômetro
        if ((accData.y > 0) && 
        (accData.y > (float)(accFactorCal*std::abs(accData.x))) && 
        (accData.y > (float)(accFactorCal*std::abs(accData.z))))
        {  // Adicionar que o módulo de y seja maior que o modulo de x e o modulo de z
            W(count + 2 * accNumSamplesCal, 0) = accData.x;
            W(count + 2 * accNumSamplesCal, 1) = accData.y;
            W(count + 2 * accNumSamplesCal, 2) = accData.z;
            W(count + 2 * accNumSamplesCal, 3) = 1.0f;  // Última coluna igual a 1
            count++;
        } else {
            ESP_LOGI(TAG, "Eixo y não está apontando para cima.");
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);  // Delay de 10ms entre as leituras
        // Caso contrário, rejeita a amostra e não incrementa count
    }
    ESP_LOGI(TAG, "Amostras coletadas: %d", count);
    ESP_LOGI(TAG, "*******************************************************");
    ESP_LOGI(TAG, "Coloque o acelerômetro com o eixo y apontando para baixo e mantenha essa posição.");
    timer(5);
    ESP_LOGI(TAG, "Coletando amostras...");
    count = 0;
    while (count < accNumSamplesCal) {

        accRead();  // Lê os dados do acelerômetro
        if ((accData.y < 0) && 
        (accData.y < (float)(-accFactorCal*std::abs(accData.x))) && 
        (accData.y < (float)(-accFactorCal*std::abs(accData.z))))
        {  // Adicionar que o módulo de y seja maior que o modulo de x e o modulo de z
            W(count + 3 * accNumSamplesCal, 0) = accData.x;
            W(count + 3 * accNumSamplesCal, 1) = accData.y;
            W(count + 3 * accNumSamplesCal, 2) = accData.z;
            W(count + 3 * accNumSamplesCal, 3) = 1.0f;  // Última coluna igual a 1
            count++;
        } else {
            ESP_LOGI(TAG, "Eixo y não está apontando para baixo.");
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);  // Delay de 10ms entre as leituras
        // Caso contrário, rejeita a amostra e não incrementa count
    }
    ESP_LOGI(TAG, "Amostras coletadas: %d", count);
    ESP_LOGI(TAG, "*******************************************************");
    // Collecting N Samples with z-axis pointing downwards
    ESP_LOGI(TAG,
            "Coloque o acelerômetro com o eixo z apontando para cima e mantenha essa posição.");
    timer(5);
    ESP_LOGI(TAG, "Coletando amostras...");
    count = 0;
    while (count < accNumSamplesCal) {

        accRead();  // Lê os dados do acelerômetro
        if ((accData.z > 0) && 
        (accData.z > (float)(accFactorCal*std::abs(accData.x))) && 
        (accData.z > (float)(accFactorCal*std::abs(accData.y))))
        {  // Adicionar que o módulo de z seja maior que o modulo de x e o modulo de y
            W(count + 4 * accNumSamplesCal, 0) = accData.x;
            W(count + 4 * accNumSamplesCal, 1) = accData.y;
            W(count + 4 * accNumSamplesCal, 2) = accData.z;
            W(count + 4 * accNumSamplesCal, 3) = 1.0f;  // Última coluna igual a 1
            count++;
        } else {
            ESP_LOGI(TAG, "Eixo z não está apontando para cima.");
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);  // Delay de 10ms entre as leituras
        // Caso contrário, rejeita a amostra e não incrementa count
    }
    ESP_LOGI(TAG, "Amostras coletadas: %d", count);
    ESP_LOGI(TAG, "*******************************************************");
    ESP_LOGI(TAG, "Coloque o acelerômetro com o eixo z apontando para baixo e mantenha essa posição.");
    timer(5);
    ESP_LOGI(TAG, "Coletando amostras...");
    count = 0;
    while (count < accNumSamplesCal) {

        accRead();  // Lê os dados do acelerômetro
        if ((accData.z < 0) && 
        (accData.z < (float)(-accFactorCal*std::abs(accData.x))) && 
        (accData.z < (float)(-accFactorCal*std::abs(accData.y))))
        {  // Adicionar que o módulo de z seja maior que o modulo de x e o modulo de y
            W(count + 5 * accNumSamplesCal, 0) = accData.x;
            W(count + 5 * accNumSamplesCal, 1) = accData.y;
            W(count + 5 * accNumSamplesCal, 2) = accData.z;
            W(count + 5 * accNumSamplesCal, 3) = 1.0f;  // Última coluna igual a 1
            count++;
        } else {
            ESP_LOGI(TAG, "Eixo z não está apontando para baixo.");
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);  // Delay de 10ms entre as leituras
        // Caso contrário, rejeita a amostra e não incrementa count
    }
    ESP_LOGI(TAG, "Amostras coletadas: %d", count);
    ESP_LOGI(TAG, "*******************************************************");
    ESP_LOGI(TAG, "Calibração do acelerômetro concluída.");
    ESP_LOGI(TAG, "*******************************************************");
    timer(5);
    // ESP_LOGI(TAG, "Matriz W:");

    // Print the matrix W using Eigen    
    //std::cout << "W = " << std::endl << W << std::endl;

    // Calcula a matriz de calibragem
    Eigen::MatrixXf Wt = W.transpose();
    Eigen::MatrixXf WtW = Wt * W;
    Eigen::MatrixXf WtW_inv = WtW.inverse();
    Eigen::MatrixXf WtY = Wt * Y;
    accCalibrationMatrix = WtW_inv * WtY;

    //Store the calibration matrix in NVS
    esp_err_t writeErrorMatrix = NVSUtils::WriteEigenMatrix(nvs, "accCalMat", accCalibrationMatrix);
    if (writeErrorMatrix != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write accelerometer calibration status to NVS: %s", esp_err_to_name(writeErrorMatrix));
        return writeErrorMatrix; // Return error if writing to NVS failed
    }
    esp_err_t writeErrorBool = NVSUtils::WriteBool(nvs, "accCalStr", true); // Set the flag to indicate calibration is stored
    if (writeErrorBool != ESP_OK){
        ESP_LOGE(TAG, "Failed to write accelerometer calibration matrix to NVS: %s", esp_err_to_name(writeErrorBool));
        return writeErrorBool; // Return error if writing to NVS failed
    }
    
    // Commit the changes to NVS
    nvs.Commit();
    ESP_LOGI(TAG, "Accelerometer calibration matrix stored in NVS.");
    
    //std::cout << "A = " << std::endl << *A << std::endl;

    accCalibrated = true;
    accCalibrationInProgress = false;
    return ESP_OK; 
}

esp_err_t MPU9250::temRead()
{

    // Lê os dados do sensor de temperatura
    uint8_t raw_data[2];
    if (i2cManager->readRegFromDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_TEMP_OUT_H, raw_data, 2) == ESP_OK) {
        int16_t temp_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        temData = (temp_raw / 333.87f) + 21.00f; // Conversão para graus 
    } else {
        ESP_LOGE(TAG, "Failed to read temperature data");
        return ESP_FAIL;
    }

    return ESP_OK;

}

esp_err_t MPU9250::temGetRead() 
{ 
    
    ESP_LOGI(TAG, "Temperature Data (°C): Temp: %.2f", temData);
    
    return ESP_OK; 
}

esp_err_t MPU9250::magConfig()
{
/* Configure AK8963 magnetometer for 100Hz continuous mode, 16-bit output
 * Steps:
 * 1. Enable I2C master mode on MPU9250 if not already enabled
 * 2. Soft-reset AK8963 by writing 0x01 to CNTL2
 * 3. Read ASA values from AK8963 via SLV1 if not already read 
 * 4. Set AK8963 to power-down mode by writing 0x00 to CNTL1
 * 5. Set AK8963 to continuous measurement mode 2 (100Hz, 16-bit) by writing 0x16 to CNTL1
 * 6. Configure MPU9250 I2C Master to read AK8963 data registers via SLV0
 * */
	//ESP_LOGI(TAG, "Configuring AK8963 magnetometer for 100Hz continuous mode...");
 	esp_err_t ret;	
	// Enable I2C master mode first
    	if (!this->I2CMasterEnabled) {
		ESP_LOGI(TAG, "I2C master mode not enabled. Enabling now...");
    		ret = enableI2CMaster();
    		if (ret != ESP_OK) {
        		ESP_LOGE(TAG, "Failed to enable I2C master");
        		return ret;
    		}
    		else {
			ESP_LOGI(TAG, "I2C master enabled to 400 kHz successfully");
    		}
    	}	
    	// Soft-Reset AK8963
	ret = writeAK8963RegisterViaSLV0(AK8963_CNTL2, 0x01);
    	if (ret != ESP_OK) {
	    	ESP_LOGE(TAG, "Failed to reset AK8963");
	    	return ret;
    	}
    	else {
	    	ESP_LOGI(TAG, "AK8963 Soft reset successfully");
    	}
    	vTaskDelay(pdMS_TO_TICKS(100)); // Wait for reset
    	// Read ASA values via SLV1
    	if (!this->magReadMagASA) {
		ESP_LOGI(TAG, "Reading magnetometer ASA values for the first time via SLV1...");
		ret = readMagnetometerASAViaSLV1();
    		if (ret != ESP_OK) {
        		ESP_LOGE(TAG, "Failed to read ASA values via SLV1");
        		return ret;
    		}
	}
	// Set to power-down mode before switching to continuous mode
    	ret = writeAK8963RegisterViaSLV0(AK8963_CNTL1, 0x00); // Power down
    	if (ret != ESP_OK) {
        	ESP_LOGE(TAG, "Failed to set power down mode");
        	return ret;
    	}
   	vTaskDelay(pdMS_TO_TICKS(100));
    	// CNTL1 register: 0x16 for continuous mode 100Hz, 16-bit, continous measurement mode 2
	// 0x18 = 00010110b
    	ret = writeAK8963RegisterViaSLV0(AK8963_CNTL1, 0x16); // Continuous mode 100Hz, 16-bit
    	if (ret != ESP_OK) {
        	ESP_LOGE(TAG, "Failed to set 100Hz continuous mode");
        	return ret;
    	}
    	ESP_LOGI(TAG, "AK8963 magnetometer configured for 100Hz, 16-bit, continuous mode 2 successfully.");
	vTaskDelay(pdMS_TO_TICKS(10));	
	// Iniciar leitura dos dados do magnetômetro
	// Configurar o MPU9250 para ler os dados do AK8963 via I2C Master
	// Configurar o Slave 0 para ler os dados do AK8963
	// AK8963 data registers start at 0x03 and we need to read 7 bytes (HXL to ST2)
	// MPU9250_I2C_SLV0_ADDR = 0x25
	// MPU9250_I2C_SLV0_REG = 0x26
	// MPU9250_I2C_SLV0_CTRL = 0x27
	// MPU9250_I2C_SLV0_DO = 0x63
	uint8_t slv1_addr = MPU9250_MAGNETOMETER_ADDR | 0x80; // Read operation
	uint8_t slv1_reg = AK8963_HXL; // Starting register for magnetometer data
	uint8_t slv1_ctrl = 0x87; // Enable, read 8 bytes // 10000111b = 0x88
	esp_err_t ret1, ret2, ret3;			  	
	ret1 = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_I2C_SLV1_ADDR, &slv1_addr, 1);	
	ret2 = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_I2C_SLV1_REG, &slv1_reg, 1);
	ret3 = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_I2C_SLV1_CTRL, &slv1_ctrl, 1);
	if (ret1 != ESP_OK || ret2 != ESP_OK || ret3 != ESP_OK) {
		ESP_LOGE(TAG, "Failed to configure MPU9250 I2C Master for AK8963 data reading");
		return ESP_FAIL;
	}
	// wait 10 ms
	vTaskDelay(pdMS_TO_TICKS(10));
	ESP_LOGI(TAG, "MPU9250 I2C Master configured to read AK8963 data successfully.");	

	this->magConfigured = true;
	
	return ESP_OK;
}

esp_err_t MPU9250::readMagnetometerASA() 
{
    /* DEPRECATED */
	// Switch to Fuse ROM access mode
    uint8_t asa_data[3];
    
    // Read sensitivity adjustment values
    esp_err_t ret = i2cManager->readRegFromDeviceWithHandle(*MPU9250_mag_handle_ptr, 
                                                           AK8963_ASAX, asa_data, 3);
    if (ret != ESP_OK) return ret;
    
    // Calculate adjusted sensitivity
    float asax = (asa_data[0] - 128) / 256.0f + 1.0f;
    float asay = (asa_data[1] - 128) / 256.0f + 1.0f;
    float asaz = (asa_data[2] - 128) / 256.0f + 1.0f;
    
    // Apply to scale factors
    this->magScaleX = this->magScale * asax;
    this->magScaleY = this->magScale * asay;
    this->magScaleZ = this->magScale * asaz;
    
    ESP_LOGI(TAG, "ASA values: X=%.3f, Y=%.3f, Z=%.3f", asax, asay, asaz);
    this->magReadMagASA = true;
    return ESP_OK;
}

esp_err_t MPU9250::magRead()
{
	bool silent = true; // for debugging
			    // 
	if (!this->magConfigured) {
		ESP_LOGW(TAG, "Magnetometer not configured. Please configure the magnetometer first.");
		return ESP_FAIL;
	}
	if (!this->magCalibrated && !this->magCalibrationInProgress && !silent) {
		ESP_LOGW(TAG, "Magnetometer calibration not completed. Please calibrate the magnetometer first.");
		//return ESP_FAIL;
	}
	// Read magnetometer data
	uint8_t raw_data[7]; //7 for magnetometer data 
    	int16_t magX_raw, magY_raw, magZ_raw;
	uint8_t st2;
	if (i2cManager->readRegFromDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_EXT_SENS_DATA_00, raw_data, 7) == ESP_OK) {
		//
		st2 = raw_data[6];
		// Verify for HOFL: Magnetic sensor overflow
		if (st2 & 0x08) {
			ESP_LOGW(TAG, "Magnetometer overflow occurred.");	
		}
		//
		magX_raw = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    		magY_raw = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    		magZ_raw = (int16_t)((raw_data[5] << 8) | raw_data[4]);
        	//
        	if (this->magCalibrationInProgress || !this->magCalibrated) {
                	this->magData.x = magX_raw * this->magScaleX;
                	this->magData.y = magY_raw * this->magScaleY;
                	this->magData.z = magZ_raw * this->magScaleZ;
		} 
		else if (this->magCalibrated) {
		        //
                	this->Bp(0) = magX_raw * this->magScaleX;
                	this->Bp(1) = magY_raw * this->magScaleY;
                	this->Bp(2) = magZ_raw * this->magScaleZ;

	    		this->Bc = this->W_inv * (this->Bp - this->V);

	    		this->magData.x = this->Bc(0);
	    		this->magData.y = this->Bc(1);
	    		this->magData.z = this->Bc(2);
		}
	} else {
		ESP_LOGE(TAG, "Failed to read magnetometer data");
    		return ESP_FAIL;
	}
	return ESP_OK;
}

esp_err_t MPU9250::magRead_old()
{
	/* DEPRECATED */
	if (!this->magCalibrated && !this->magCalibrationInProgress) {
		ESP_LOGW(TAG, "Magnetometer calibration not completed. Please calibrate the magnetometer first.");
		//return ESP_FAIL;
	}

	uint8_t pass_through = PASS_THROUGH_MODE;
	uint8_t mag_single_measure = 0x01;

	esp_err_t ret1, ret2, ret3;
	ret1 = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_INT_PIN_CFG, &pass_through, 1);  // configura o modo pass-through
	ret2 = i2cManager->writeRegToDeviceWithHandle(*MPU9250_mag_handle_ptr, MPU9250_MAG_CNTL, &mag_single_measure,1);  // configura o endereço do AK8963

	if (ret1 != ESP_OK || ret2 != ESP_OK) {
		ESP_LOGE(TAG, "Failed to configure magnetometer");
    		return ESP_FAIL;
	}

    if (!this->magReadMagASA) {
        ESP_LOGI(TAG, "Reading magnetometer ASA values for the first time...");
        ret3= readMagnetometerASA();
        if (ret3 != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read magnetometer ASA");
            return ret3;
        }
    }
    
	uint8_t data_ready = 0x00;
	const int max_attempts = 100; // e.g., 100 attempts * 10ms = 1 second timeout
	int attempts = 0;
	while ((data_ready & 0x01) == 0 && attempts < max_attempts) {
		if (i2cManager->readRegFromDeviceWithHandle(*MPU9250_mag_handle_ptr, MPU9250_MAG_DATA_RDY, &data_ready, 1) != ESP_OK) {
			ESP_LOGE(TAG, "Failed to read magnetometer data ready status");
			return ESP_FAIL;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
    	attempts++;
	}
	if ((data_ready & 0x01) == 0) {
    		ESP_LOGE(TAG, "Timeout waiting for magnetometer data ready");
    		return ESP_FAIL;
	}

	// Read magnetometer data
	uint8_t raw_data[6]; //6 for magnetometer data 
    int16_t magX_raw, magY_raw, magZ_raw;
	if (i2cManager->readRegFromDeviceWithHandle(*MPU9250_mag_handle_ptr, MPU9250_MAG_HXL, raw_data, 6) == ESP_OK) {
        magX_raw = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    	magY_raw = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    	magZ_raw = (int16_t)((raw_data[5] << 8) | raw_data[4]);
        
        if (this->magCalibrationInProgress || !this->magCalibrated) {
    	        //
                this->magData.x = magX_raw * magScaleX;
                this->magData.y = magY_raw * magScaleY;
                this->magData.z = magZ_raw * magScaleZ;
		} else if (this->magCalibrated) {
		        //
                this->Bp(0) = magX_raw * magScaleX;
                this->Bp(1) = magY_raw * magScaleY;
                this->Bp(2) = magZ_raw * magScaleZ;

	    		this->Bc = this->W_inv * (this->Bp - this->V);

	    		this->magData.x = this->Bc(0);
	    		this->magData.y = this->Bc(1);
	    		this->magData.z = this->Bc(2);
		}
	} else {
		ESP_LOGE(TAG, "Failed to read magnetometer data");
    		return ESP_FAIL;
	}
	// Disable pass-through mode
	pass_through = 0x00;
	i2cManager->writeRegToDeviceWithHandle(*MPU9250_mag_handle_ptr, MPU9250_INT_PIN_CFG, &pass_through, 1);    // Turn-off PASS_THROUGH mode - Para ativar o Magnetómetro
	return ESP_OK;
}

esp_err_t MPU9250::magGetRead_term()
{

    printf("%.2f, %.2f, %.2f, %.2f\n", this->magData.x, this->magData.y, this->magData.z, this->magFieldStrength()); 	
    return ESP_OK;
}	

esp_err_t MPU9250::magGetRead()
{

    ESP_LOGI(TAG, "Magnetometer Data (uT): X: %.2f, Y: %.2f, Z: %.2f, B: %.2f", this->magData.x, this->magData.y, this->magData.z, this->magFieldStrength());

    return ESP_OK;
}

esp_err_t MPU9250::magGetSamples(float* matrix, int rows, int cols)
{
	float magCurrentSample[3]; 
	int count = 0;
	ESP_LOGI(TAG, "Iniciando a calibração do magnetômetro...");	
	ESP_LOGI(TAG, "Gire o sensor em todas as direções para coletar os dados necessários.");
	while (count < rows){
		this->magRead();  // Lê os dados do magnetômetro
		magCurrentSample[0] = this->magData.x;
		magCurrentSample[1] = this->magData.y;
		magCurrentSample[2] = this->magData.z;
		if (isMagSampleOK(count, 3,  &magCurrentSample[0], matrix)) {
			for (int j = 0; j < cols; j++) {
				matrix[count*cols + j] = magCurrentSample[j];
			}
			count++;
			ESP_LOGI(TAG, "Amostra válida: X:%.2f, Y:%.2f, Z:%.2f", 
					magCurrentSample[0], magCurrentSample[1], magCurrentSample[2]);
			ESP_LOGI(TAG, "Amostras coletadas: %d de %d", count, rows);
		} 
		else {
			ESP_LOGI(TAG, "Amostra inválida. Descartando: X:%.2f, Y:%.2f, Z:%.2f", 
					magCurrentSample[0], magCurrentSample[1], magCurrentSample[2]);
		}
		this->timer(1.0, false);
	}
	ESP_LOGI(TAG, "Magnetometer Samples Collected for Calibration: %d", rows);
	return ESP_OK;
}

esp_err_t MPU9250::magCalibrate(PL::NvsNamespace& nvs)
{
	/* ==============================================================
	 * Magnetometer Calibration using Ellipsoid Fit Method
	 * Reference:
	 * ==============================================================
	 *
	 * 0. Set the flag magCalibrationInProgress to true 
	 * 1. Check if calibration data already exists in NVS
	 * 2. Collect 'magNumSamplesCal' samples of magnetometer data while rotating the sensor in all directions
	 * 3. Formulate the problem as a quadratic form and compute the matrix XtX
	 * 4. Perform eigen decomposition of XtX to find the minimum eigenvalue and corresponding eigenvector
	 * 5. Extract the ellipsoid fit matrix A, hard iron offset vector V, and compute the inverse soft iron matrix W_inv 
	 * 6. Store the calibration parameters in NVS
	 */
	// Step 0: Set calibration in progress flag
	this->magCalibrationInProgress = true;
	// Step 1: Check if calibration data already exists in NVS
	// Use NVSUtils to read the flag
	bool magCalibrationStored = false;
	esp_err_t ret = NVSUtils::ReadBool(nvs, "magCalStr", magCalibrationStored); 
	if ((ret==ESP_OK) && magCalibrationStored && !forceMagCalibration) {
		ESP_LOGI(TAG, "Magnetometer calibration already stored in NVS. Skipping calibration.");
		// Read the calibration data from NVS
	    	esp_err_t ret1 = NVSUtils::ReadEigenMatrix(nvs, "magCalMat", this->W_inv);
	    	if (ret1 != ESP_OK) {
			ESP_LOGE(TAG, "Failed to read magnetometer calibration matrix from NVS: %s", esp_err_to_name(ret1));
			return ret1; // Return error if writing to NVS failed
	    	}
	    	ret1 = NVSUtils::ReadEigenVector(nvs, "magCalVec", this->V);
	    	if (ret1 != ESP_OK) {
			ESP_LOGE(TAG, "Failed to read magnetometer calibration vector from NVS: %s", esp_err_to_name(ret1));
			return ret1; // Return error if writing to NVS failed
	    	}
		this->magCalibrated = true;
		this->magCalibrationInProgress = false;
		this->magCalibrationFailed = false;
		ESP_LOGI(TAG, "Magnetometer calibration parameters loaded from NVS successfully.");
		return ESP_OK;
	}
	else if (ret != ESP_OK && ret == ESP_ERR_NVS_NOT_FOUND) {
		ESP_LOGE(TAG, "Failed to read magnetometer calibration status from NVS: %s", esp_err_to_name(ret));
		this->magCalibrationInProgress = false;
		return ret; // Return error if reading from NVS failed
	}
	if (forceMagCalibration) {
		ESP_LOGI(TAG, "Force magnetometer calibration enabled. Proceeding with calibration...");
	}

	ESP_LOGI(TAG, "*******************************************************");	
	ESP_LOGI(TAG, "Iniciando a calibração do magnetômetro...");
	ESP_LOGI(TAG, "*******************************************************");
	ESP_LOGI(TAG, "Gire o sensor em todas as direções para coletar os dados necessários.");
	
	// Step 2: Collect magnetometer samples
	int rows = this->magNumSamplesCal;
	int cols = 3;
	float magSamplesMatrix[rows][cols];
	if(this->magGetSamples(&magSamplesMatrix[0][0], rows, cols) != ESP_OK){
		ESP_LOGE(TAG, "Failed to get magnetometer samples for calibration.");
		this->magCalibrationInProgress = false;
		return ESP_FAIL;
	}
	// 3. Formulate the problem and compute XtX
	Eigen::VectorXf magSample(10);
	Eigen::MatrixXf XtX(10,10);
	XtX.setZero(); 	
	int count = 0;
	while (count < this->magNumSamplesCal){
		float x = magSamplesMatrix[count][0];
		float y = magSamplesMatrix[count][1];
		float z = magSamplesMatrix[count][2];
		magSample(0) = x*x;	
		magSample(1) = 2*x*y; 
		magSample(2) = 2*x*z;
		magSample(3) = y*y;
		magSample(4) = 2*y*z;
		magSample(5) = z*z;
		magSample(6) = x; 
		magSample(7) = y; 
		magSample(8) = z;
		magSample(9) = 1.0f;
		for (int r = 0; r < 10; r++) {
			for (int s = 0; s < 10; s++) {
				XtX(r,s) = XtX(r,s) + magSample(r)*magSample(s);
			}
		}
		count++;
	}
	// Step 4: Eigen decomposition of XtX	
	Eigen::EigenSolver<Eigen::MatrixXf> solver_XtX(XtX);
	if (solver_XtX.info() != Eigen::Success) {
		ESP_LOGE(TAG, "Eigen decomposition failed!");
		return ESP_FAIL;
	}
	// Get the eigenvalues and eigenvectors
	Eigen::VectorXf eigenvalues_XtX = solver_XtX.eigenvalues().real();
	Eigen::MatrixXf eigenvectors_XtX = solver_XtX.eigenvectors().real();
	/* Encontrar o índice do menor autovalor */
	int minIndex;
	eigenvalues_XtX.minCoeff(&minIndex);
	//ESP_LOGI(TAG, "Minimum eigenvalue index: %d", minIndex);
	ESP_LOGI(TAG, "Minimum eigenvalue of XtX: %.6f", eigenvalues_XtX(minIndex));
	/* Encontrar o autovetor corresponedente ao mínimo autovalor */
	Eigen::VectorXf minEigenvector_XtX = eigenvectors_XtX.col(minIndex);
    	//printEigenVector(TAG, "Minimum eigenvector of XtX", minEigenvector_XtX);
	ESP_LOGI(TAG, "Norma do autovetor mínimo: %.6f", minEigenvector_XtX.norm());
	/* Step 5: Extract calibration parameters */	
	/* Ellipsoid Fit Matrix */
	Eigen::Matrix3f A;
	A(0,0) = minEigenvector_XtX(0); // beta0
	A(0,1) = minEigenvector_XtX(1); // beta1
	A(0,2) = minEigenvector_XtX(2); // beta2
	A(1,0) = minEigenvector_XtX(1); // beta1
	A(1,1) = minEigenvector_XtX(3); // beta3
	A(1,2) = minEigenvector_XtX(4); // beta4
	A(2,0) = minEigenvector_XtX(2); // beta2
	A(2,1) = minEigenvector_XtX(4); // beta4
	A(2,2) = minEigenvector_XtX(5); // beta5
	//printEigenMatrix(TAG, "Ellipsoid Fit Matrix A", A);	
	//float detEigenMatrix(const Eigen::Matrix3f& matrix) {
	/* Hard Iron Vector */
	this->V = -(0.5f) * A.inverse() * Eigen::Vector3f(minEigenvector_XtX(6), minEigenvector_XtX(7), minEigenvector_XtX(8));
	//printEigenVector(TAG, "Hard Iron Offset Vector V", V, 6);
	/* Inverse Soft Iron Matrix */
	Eigen::EigenSolver<Eigen::MatrixXf> solver_A(A);
	if (solver_A.info() != Eigen::Success) {
		ESP_LOGE(TAG, "Eigen decomposition of matrix 'A' failed!");
		return ESP_FAIL;
	}
	// Get the eigenvalues and eigenvectors of A
	Eigen::Vector3f eigenvalues_A = solver_A.eigenvalues().real();
    	//printEigenVector(TAG, "Eigenvalues of matrix A", eigenvalues_A, 6);	
	Eigen::Matrix3f Q = solver_A.eigenvectors().real(); // Eigenvectors matrix of A
	//printEigenMatrix(TAG, "Q - Eigenvectors of matrix A", Q, 6);
	// Cria sqrt_L com as raízes quadradas dos autovalores
	Eigen::Vector3f sqrt_eigenvalues = eigenvalues_A.unaryExpr([](float x) { return sqrtf(fmaxf(x, 0.0f)); });
	Eigen::Matrix3f sqrt_L = sqrt_eigenvalues.asDiagonal();
    	//ESP_LOGI(TAG, "Is sqrt_L diagonal matrix? %s", isDiagonalMatrix(sqrt_L) ? "Yes" : "No");
	//printEigenMatrix(TAG, "sqrt(L) - Square root of eigenvalues", sqrt_L, 6);
    	// If any of the elements of sqrt_L are zero, log a warning
	float zero_tol = 1e-6f;
    	for (int i = 0; i < sqrt_L.rows(); i++) {
        	if (sqrt_L(i, i) <= zero_tol) {
			//printEigenMatrix(TAG, "Matrix sqrt(L) causing issue", sqrt_L, 6);	
            		ESP_LOGW(TAG, "Warning: sqrt_L has zero on its diagonal at index %d", i);
            		ESP_LOGW(TAG, "É necessário coletar mais dados.");
            		ESP_LOGW(TAG, "Calibração do magnetômetro não concluída.");
            		this->magCalibrationFailed = true;
            		this->magCalibrated = false;
            		this->magCalibrationInProgress = false;
			this->forceMagCalibration = true;
            		return ESP_FAIL;
        	}
	}
	// PARA UTILIZAR O FATOR DE ESCALA, DESCOMENTAR A LINHA ABAIXO E COMENTAR A LINHA DEPOIS DELA
	//this->W_inv = Q * sqrt_L * Q.inverse();
	// PARA NÃO UTILIZAR O FATOR DE ESCALA, COMENTAR A LINHA ACIMA E DESCOMENTAR A LINHA ABAIXO
	this->W_inv = Eigen::Matrix3f::Identity();
	//printEigenMatrix(TAG, "Inverse Soft Iron Matrix W^{-1}", W_inv, 6);
	//
	/* Geomagnetic Field Strength */
	float B = sqrtf( fabs(A(0,0)*V(0)*V(0) + A(1,1)*V(1)*V(1) + A(2,2)*V(2)*V(2) + 2*A(0,1)*V(0)*V(1) + 2*A(0,2)*V(0)*V(2) + 2*A(1,2)*V(1)*V(2) - minEigenvector_XtX(9) ));
	ESP_LOGI(TAG, "Estimated Geomagnetic Field Strength: %.2f uT", B);
	/* Fit error */
	float fit_error = (1.0f/(2.0f * B*B)) * sqrtf( eigenvalues_XtX(minIndex) / this->magNumSamplesCal );
	ESP_LOGI(TAG, "Magnetometer fit error: %.6f", fit_error);
	// Store calibration data in NVS
	// Store the matrix W_inv
	// Store the vector V
	esp_err_t ret2 = NVSUtils::WriteEigenMatrix(nvs, "magCalMat", this->W_inv);
	if (ret2 != ESP_OK) {
		ESP_LOGE(TAG, "Failed to write magnetometer calibration matrix to NVS: %s", esp_err_to_name(ret2));
		return ret; // Return error if writing to NVS failed
	}
	ret2 = NVSUtils::WriteEigenVector(nvs, "magCalVec", this->V);
	if (ret2 != ESP_OK) {
		ESP_LOGE(TAG, "Failed to write magnetometer calibration vector to NVS: %s", esp_err_to_name(ret2));	
		return ret2; // Return error if writing to NVS failed
	}				 // 
	ret2 = NVSUtils::WriteBool(nvs, "magCalStr", true); // Set the flag to indicate calibration is stored
	if (ret2 != ESP_OK){
		ESP_LOGE(TAG, "Failed to write magnetometer calibration status to NVS: %s", esp_err_to_name(ret2));
		return ret2; // Return error if writing to NVS failed
	}
	nvs.Commit();
	ESP_LOGI(TAG, "Magnetometer calibration data stored in NVS.");
	ESP_LOGI(TAG, "Magnetometer calibration completed successfully.");
	this->magCalibrationInProgress = false;
	this->magCalibrated = true;
	this->magCalibrationFailed = false;
	return ESP_OK;
}

bool MPU9250::isMagSampleOK(int rows, int cols, float *vector, float* matrix)
{
	// Verifica se a amostra atual é válida (não é NaN ou infinito)
	for (int i = 0; i < cols; i++) {
		if (isnan(vector[i]) || isinf(vector[i])) {
			ESP_LOGI(TAG, "Amostra contem zeros ou NaN.");
			return false; // Amostra inválida
		}
		//else {
		//	ESP_LOGI(TAG, "Amostra não contem nem zeros nem NaN.");
		//}
	}
	// Verifica se a distancia euclidiana entre a amostra atual e as amostras anteriores é maior que um limiar
	float threshold = 0.5f; // Defina um limiar adequado
	for (int k = 0; k < rows; k++){
		float diffnorm = 0;
		for (int j = 0; j < cols; j++){
			float diff = matrix[k* cols + j] - vector[j];
			diffnorm += diff * diff;
			diffnorm = sqrtf(diffnorm);
			//printf("Diffnorm: %.2f\n", diffnorm);
			if (diffnorm < threshold) return false;
		}
	}
    	return true; // Amostra válida e única
}

esp_err_t MPU9250::magCalibrate_old(PL::NvsNamespace& nvs)
{
	/* DEPRECATED */
	this->magCalibrationInProgress = true;
	// Use NVSUtils to read the flag
	bool magCalibrationStored = false;
	esp_err_t magCalStrError = NVSUtils::ReadBool(nvs, "magCalStr", magCalibrationStored); 
	if ((magCalStrError==ESP_OK) && magCalibrationStored && !forceMagCalibration) {
		ESP_LOGI(TAG, "Magnetometer calibration already stored in NVS. Skipping calibration.");
		// Read the calibration data from NVS
	    	esp_err_t readErrorMatrix = NVSUtils::ReadEigenMatrix(nvs, "magCalMat", this->W_inv);
	    	if (readErrorMatrix != ESP_OK) {
			ESP_LOGE(TAG, "Failed to read magnetometer calibration matrix from NVS: %s", esp_err_to_name(readErrorMatrix));
			return readErrorMatrix; // Return error if writing to NVS failed
	    	}
	    	esp_err_t readErrorVector = NVSUtils::ReadEigenVector(nvs, "magCalVec", this->V);
	    	if (readErrorVector != ESP_OK) {
			ESP_LOGE(TAG, "Failed to read magnetometer calibration vector from NVS: %s", esp_err_to_name(readErrorVector));
			return readErrorVector; // Return error if writing to NVS failed
	    	}
		// Set the calibration flag to true
            	this->magCalibrated = true;
            	this->magCalibrationInProgress = false;
            	this->magCalibrationFailed = false;
		ESP_LOGI(TAG, "Magnetometer calibration data loaded from NVS.");
		return ESP_OK; // Skip calibration if data is already stored
	} else if (magCalStrError != ESP_OK) {
		ESP_LOGW(TAG, "Failed to read magnetometer calibration status from NVS: %s", esp_err_to_name(magCalStrError));
		this->forceMagCalibration = true;
		//return magCalStrError; // Return error if reading from NVS failed
	}
       	if(forceMagCalibration) {
		ESP_LOGI(TAG, "Forcing magnetometer calibration.");
	}

	ESP_LOGI(TAG, "*******************************************************");	
	ESP_LOGI(TAG, "Iniciando a calibração do magnetômetro...");
	ESP_LOGI(TAG, "*******************************************************");
	ESP_LOGI(TAG, "Gire o sensor em todas as direções para coletar os dados necessários.");
	
	// Collecting "magNumSamplesCal" Samples
	Eigen::VectorXf magSample(10);
	Eigen::MatrixXf XtX(10,10);
	XtX.setZero(); 	

	int count = 0;
	while (count < this->magNumSamplesCal){
		
		this->magRead();  // Lê os dados do magnetômetro
		magSample(0) = this->magData.x * this->magData.x;
		magSample(1) = 2 * this->magData.x * this->magData.y;
		magSample(2) = 2* this->magData.x * this->magData.z;
		magSample(3) = this->magData.y * this->magData.y;
		magSample(4) = 2 * this->magData.y * this->magData.z;
		magSample(5) = this->magData.z * this->magData.z;
		magSample(6) = this->magData.x;
		magSample(7) = this->magData.y;
		magSample(8) = this->magData.z;
		magSample(9) = 1.0f;

		for (int r = 0; r < 10; r++) {
			for (int s = 0; s < 10; s++) {
				XtX(r,s) = XtX(r,s) + magSample(r)*magSample(s);
			}
		}
		this->timer(1.0, false);
		ESP_LOGI(TAG, "Amostras coletadas: %d de %d", count+1, this->magNumSamplesCal);
		count++;
	}
	
	/* - [X] TODO: Calcular os autovalores de XtX */
	Eigen::EigenSolver<Eigen::MatrixXf> solver_XtX(XtX);
	if (solver_XtX.info() != Eigen::Success) {
		ESP_LOGE(TAG, "Eigen decomposition failed!");
		return ESP_FAIL;
	}
	// Get the eigenvalues and eigenvectors
	Eigen::VectorXf eigenvalues_XtX = solver_XtX.eigenvalues().real();
	Eigen::MatrixXf eigenvectors_XtX = solver_XtX.eigenvectors().real();

	/* Encontrar o índice do menor autovalor */
	int minIndex;
	eigenvalues_XtX.minCoeff(&minIndex);
	//ESP_LOGI(TAG, "Minimum eigenvalue index: %d", minIndex);
	ESP_LOGI(TAG, "Minimum eigenvalue of XtX: %.6f", eigenvalues_XtX(minIndex));

	/* Encontrar o autovetor corresponedente ao mínimo autovalor */
	Eigen::VectorXf minEigenvector_XtX = eigenvectors_XtX.col(minIndex);
    	printEigenVector(TAG, "Minimum eigenvector of XtX", minEigenvector_XtX, 6);

	/* Ellipsoid Fit Matrix */
	Eigen::Matrix3f A;
	A(0,0) = minEigenvector_XtX(0); // beta0
	A(0,1) = minEigenvector_XtX(1); // beta1
	A(0,2) = minEigenvector_XtX(2); // beta2
	A(1,0) = minEigenvector_XtX(1); // beta1
	A(1,1) = minEigenvector_XtX(3); // beta3
	A(1,2) = minEigenvector_XtX(4); // beta4
	A(2,0) = minEigenvector_XtX(2); // beta2
	A(2,1) = minEigenvector_XtX(4); // beta4
	A(2,2) = minEigenvector_XtX(5); // beta5
	printEigenMatrix(TAG, "Ellipsoid Fit Matrix A", A, 6);	

	/* Hard Iron Vector */
	this->V = -(0.5f) * A.inverse() * Eigen::Vector3f(minEigenvector_XtX(6), minEigenvector_XtX(7), minEigenvector_XtX(8));
	printEigenVector(TAG, "Hard Iron Offset Vector V", V, 6);
	/* Inverse Soft Iron Matrix */
	Eigen::EigenSolver<Eigen::MatrixXf> solver_A(A);
	if (solver_A.info() != Eigen::Success) {
		ESP_LOGE(TAG, "Eigen decomposition of matrix 'A' failed!");
		return ESP_FAIL;
	}
	// Get the eigenvalues and eigenvectors of A
	Eigen::Vector3f eigenvalues_A = solver_A.eigenvalues().real();
    	printEigenVector(TAG, "Eigenvalues of matrix A", eigenvalues_A, 6);	
	Eigen::Matrix3f Q = solver_A.eigenvectors().real(); // Eigenvectors matrix of A
	printEigenMatrix(TAG, "Q - Eigenvectors of matrix A", Q, 6);
	// Cria sqrt_L com as raízes quadradas dos autovalores
	Eigen::Vector3f sqrt_eigenvalues = eigenvalues_A.unaryExpr([](float x) { return sqrtf(fmaxf(x, 0.0f)); });
	Eigen::Matrix3f sqrt_L = sqrt_eigenvalues.asDiagonal();
    	//ESP_LOGI(TAG, "Is sqrt_L diagonal matrix? %s", isDiagonalMatrix(sqrt_L) ? "Yes" : "No");
	printEigenMatrix(TAG, "sqrt(L) - Square root of eigenvalues", sqrt_L, 6);

    	// If any of the elements of sqrt_L are zero, log a warning
	float zero_tol = 1e-6f;
    	for (int i = 0; i < sqrt_L.rows(); i++) {
        	if (sqrt_L(i, i) <= zero_tol) {
			printEigenMatrix(TAG, "Matrix sqrt(L) causing issue", sqrt_L, 6);	
            		ESP_LOGW(TAG, "Warning: sqrt_L has zero on its diagonal at index %d", i);
            		ESP_LOGW(TAG, "É necessário coletar mais dados.");
            		ESP_LOGW(TAG, "Calibração do magnetômetro não concluída.");
            		this->magCalibrationFailed = true;
            		this->magCalibrated = false;
            		this->magCalibrationInProgress = false;
			this->forceMagCalibration = true;
            		return ESP_FAIL;
        	}
	}
	/* - [X] TODO: Calcular W^{-1} */
	this->W_inv = Q * sqrt_L * Q.inverse();
	//printEigenMatrix(TAG, "Inverse Soft Iron Matrix W^{-1}", W_inv, 6);
	//
	/* Geomagnetic Field Strength */
	float B = sqrtf( fabs(A(0,0)*V(0)*V(0) + A(1,1)*V(1)*V(1) + A(2,2)*V(2)*V(2) + 2*A(0,1)*V(0)*V(1) + 2*A(0,2)*V(0)*V(2) + 2*A(1,2)*V(1)*V(2) - minEigenvector_XtX(9) ));
	ESP_LOGI(TAG, "Estimated Geomagnetic Field Strength: %.2f uT", B);
	/* Fit error */
	float fit_error = (1.0f/(2.0f * B*B)) * sqrtf( eigenvalues_XtX(minIndex) / this->magNumSamplesCal );
	ESP_LOGI(TAG, "Magnetometer fit error: %.6f", fit_error);

	ESP_LOGI(TAG, "Magnetometer calibration completed successfully.");

	// Store calibration data in NVS
	esp_err_t writeErrorMatrix = NVSUtils::WriteEigenMatrix(nvs, "magCalMat", this->W_inv);
	if (writeErrorMatrix != ESP_OK) {
		ESP_LOGE(TAG, "Failed to write magnetometer calibration matrix to NVS: %s", esp_err_to_name(writeErrorMatrix));
		return writeErrorMatrix; // Return error if writing to NVS failed
	}
	esp_err_t writeErrorVector = NVSUtils::WriteEigenVector(nvs, "magCalVec", this->V);
	if (writeErrorVector != ESP_OK) {
		ESP_LOGE(TAG, "Failed to write magnetometer calibration vector to NVS: %s", esp_err_to_name(writeErrorVector));	
		return writeErrorVector; // Return error if writing to NVS failed
	}				 // 
	esp_err_t writeErrorBool = NVSUtils::WriteBool(nvs, "magCalStr", true); // Set the flag to indicate calibration is stored
	if (writeErrorBool != ESP_OK){
		ESP_LOGE(TAG, "Failed to write magnetometer calibration status to NVS: %s", esp_err_to_name(writeErrorBool));
		return writeErrorBool; // Return error if writing to NVS failed
	}
	nvs.Commit();
	ESP_LOGI(TAG, "Magnetometer calibration data stored in NVS.");
        		
	this->magCalibrated = true;
	this->magCalibrationInProgress = false;
	this->magCalibrationFailed = false;

	return ESP_OK; // Placeholder for future implementation
}

float	MPU9250::magFieldStrength()
{
    return sqrtf(this->magData.x * this->magData.x + this->magData.y * this->magData.y + this->magData.z * this->magData.z);
} // Força do campo magnético em microteslas (uT)

void MPU9250::printDataToTerminal()
{
	// Print data in CSV format. The format is:
        // "GyrX,GyrY, GyrZ, AccX, AccY, AccZ, MagX, MagY, MagZ, Temp"
        printf("Gx: %.2f(°/s), Gy: %.2f(°/s), Gz: %.2f(°/s), Ax: %.2f(m/s2), Ay: %.2f(m/s2), Az: %.2f(m/s2), Mx: %.2f(uT), My: %.2f(uT), Mz: %.2f(uT), B: %.2f(uT), T: %.2f° \n",
                gyroData.x, gyroData.y, gyroData.z, 
                accData.x, accData.y, accData.z,
                magData.x, magData.y, magData.z,
                magFieldStrength(), temData);
}

void MPU9250::timer(uint8_t seconds, bool showCountdown)
{
    // Timer de 10 segundos com exibição do tempo restante
    for (int i = seconds; i > 0; --i) {
        if (showCountdown)
	    ESP_LOGI(TAG, "Aguardando... %d segundos restantes.", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
