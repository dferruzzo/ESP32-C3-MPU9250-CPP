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
    MPU9250_handle_ptr = i2cManager->isDeviceInConfig(deviceAddress);
    if (MPU9250_handle_ptr == nullptr) {
        ESP_LOGE(TAG, "Device not found in I2C configuration. Address: 0x%02X", deviceAddress);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "Device  0x%02X found in I2C configuration %p:", deviceAddress, MPU9250_handle_ptr);
    }

    MPU9250_mag_handle_ptr = i2cManager->isDeviceInConfig(deviceAddressMag);

    if (MPU9250_mag_handle_ptr == nullptr) {
        ESP_LOGE(TAG, "Device not found in I2C configuration. Address: 0x%02X", deviceAddressMag);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "Device  0x%02X found in I2C configuration %p:", deviceAddressMag, MPU9250_mag_handle_ptr);
    }

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

    // Init variables
    accCalibrationMatrix = Eigen::MatrixXf::Zero(4, 3);

    return ESP_OK;
}

esp_err_t MPU9250::MPU9250Reset()
{
    // Reseta o giroscópio

    if (MPU9250_handle_ptr == nullptr) 
    {
        ESP_LOGE(TAG, "MPU9250_handle_ptr is null, cannot reset gyroscope");
        return ESP_FAIL;
    }

    uint8_t data = 1 << MPU9250_RESET_BIT; // Bit de reset
    esp_err_t ret = i2cManager->writeRegToDeviceWithHandle(*MPU9250_handle_ptr, MPU9250_PWR_MGMT_1, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset gyroscope");
        return ret;
    }
    ESP_LOGI(TAG, "Gyroscope reset successfully");
    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait 100ms for reset to complete
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
    if ((accCalStrError==ESP_OK) &&  accCalibrationStored && !forceAccCalibration) {
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
if (i2cManager->readRegFromDeviceWithHandle(*MPU9250_mag_handle_ptr, MPU9250_MAG_HXL, raw_data, 6) == ESP_OK) {
    magData.x = (float)(((int16_t)((raw_data[1] << 8) | raw_data[0])) * magScale);
    magData.y = (float)(((int16_t)((raw_data[3] << 8) | raw_data[2])) * magScale);
    magData.z = (float)(((int16_t)((raw_data[5] << 8) | raw_data[4])) * magScale);
} else {
    ESP_LOGE(TAG, "Failed to read magnetometer data");
    return ESP_FAIL;
}
// Disable pass-through mode

pass_through = 0x00;
i2cManager->writeRegToDeviceWithHandle(*MPU9250_mag_handle_ptr, MPU9250_INT_PIN_CFG, &pass_through, 1);    // Turn-off PASS_THROUGH mode - Para ativar o Magnetómetro
return ESP_OK;
}
		
esp_err_t MPU9250::magGetRead()
{

    ESP_LOGI(TAG, "Magnetometer Data (uT): X: %.2f Y: %.2f Z: %.2f", magData.x, magData.y, magData.z);

    return ESP_OK;
}

esp_err_t MPU9250::magCalibrate(PL::NvsNamespace& nvs){
    return ESP_OK; // Placeholder for future implementation
}

void MPU9250::printDataToTerminal()
{
	// Print data in CSV format. The format is:
        // "GyrX,GyrY, GyrZ, AccX, AccY, AccZ, MagX, MagY, MagZ, Temp"
        printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                gyroData.x, gyroData.y, gyroData.z, 
                accData.x, accData.y, accData.z,
                magData.x, magData.y, magData.z,
                temData);
}

void MPU9250::timer(uint8_t seconds)
{
    // Timer de 10 segundos com exibição do tempo restante
    for (int i = seconds; i > 0; --i) {
        ESP_LOGI(TAG, "Aguardando... %d segundos restantes.", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
