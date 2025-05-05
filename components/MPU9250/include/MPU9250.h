#ifndef MPU9250_HPP
#define MPU9250_HPP

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <vector>
#include "I2CManager.h"
#include "MPU9250_Configuration_Map.h"
#include "MPU9250_Register_Map.h"

#define MPU9250_ADDRESS		0x68 // Endereço I2C do MPU9250

#define TAG "MPU9250"

#ifdef __cplusplus
extern "C" {
#endif

struct Vec3f {
    float x;
    float y;
    float z;
    Vec3f() : x(0.0f), y(0.0f), z(0.0f) {}
    Vec3f(float x, float y, float z) : x(x), y(y), z(z) {}
	};

class MPU9250 {
public:
	MPU9250(I2CManager* i2cManager);
    ~MPU9250();

    esp_err_t init();
    esp_err_t gyroReset();
    esp_err_t gyroConfig(uint8_t fullScaleSel, uint8_t enableFilter, uint8_t gyroDlpfCfg);
	void getGyroFullScale();

    
	// esp_err_t readMPU9250Data(uint8_t reg_address, uint8_t* data, size_t
	// length);
	// esp_err_t readSensorData(uint8_t reg_address, uint8_t* data, size_t
	//  length); esp_err_t writeSensorData(uint8_t reg_address, uint8_t* data,
	//  size_t length);

private:
	I2CManager* i2cManager;
	uint8_t deviceAddress = MPU9250_ADDRESS;
	i2c_master_dev_handle_t* MPU9250_handle_ptr = nullptr;
	uint8_t gyroFullScale = 0;
	Vec3f accelData;
    Vec3f gyroData;
    Vec3f magData;

	//std::vector<float> gyroData = {0.0f, 0.0f, 0.0f};
	// esp_err_t readRegFromDevice(uint8_t reg_address, uint8_t* data, size_t
	// length); esp_err_t writeRegToDevice(uint8_t reg_address, uint8_t* data,
	// size_t length); esp_err_t initDevice(); esp_err_t deInitDevice();
	};

#ifdef __cplusplus
}
#endif

#endif // I2CMANAGER_HPP

