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

#define TAG "MPU9250"

#define MPU9250_ADDRESS		0x68 // Endereço I2C do MPU9250



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
		esp_err_t gyroCalibrate();
		esp_err_t gyroRead();
		esp_err_t gyroGetRead();
		void getGyroFullScale();

	
	private:

		I2CManager* i2cManager;
		uint8_t deviceAddress = MPU9250_ADDRESS;
		i2c_master_dev_handle_t* MPU9250_handle_ptr = nullptr;

		uint8_t gyroFullScale = 0;
		bool gyroCalibrated = false;
		bool gyroCalibrationInProgress = false;
		float scale = 0.0f;
		Vec3f gyroData;
		Vec3f gyroBias;

		Vec3f accelData;
		Vec3f magData;

    };

	
	};

#ifdef __cplusplus

#endif

#endif // I2CMANAGER_HPP

