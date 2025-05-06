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
#include <eigen3/Eigen/Eigen>
#include <iostream>

#define TAG "MPU9250"

#define MPU9250_ADDRESS				0x68 // Endereço I2C do MPU9250
#define MPU9250_MAGNETOMETER_ADDR	0x0C // Endereço I2C do AK8963 (magnetômetro)

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
		esp_err_t MPU9250Reset();

		esp_err_t gyrConfig(uint8_t fullScaleSel, uint8_t enableFilter, uint8_t gyroDlpfCfg);
		esp_err_t gyrCalibrate();
		esp_err_t gyrRead();
		esp_err_t gyrGetRead();
		void getGyrFullScale();

		esp_err_t accConfig(uint8_t fullScaleSel, uint8_t accelDlpfCfg);
		esp_err_t accRead();
		esp_err_t accCalibrate();
		esp_err_t accGetRead();

        esp_err_t temRead();
		esp_err_t temGetRead();

		//esp_err_t magConfig();
		esp_err_t magRead();
		esp_err_t magGetRead();

		void printDataToTerminal();

	private:

		//	Configuração do I2C
		I2CManager* i2cManager;
		uint8_t deviceAddress = MPU9250_ADDRESS;
		uint8_t deviceAddressMag = MPU9250_MAGNETOMETER_ADDR;
		i2c_master_dev_handle_t* MPU9250_handle_ptr = nullptr;
		i2c_master_dev_handle_t* MPU9250_mag_handle_ptr = nullptr;

		// Giroscópio
		Vec3f gyroData;
		Vec3f gyroBias;
		float gyrScale = 0.0f;
		uint8_t gyrFullScale = 0;
		bool gyrCalibrated = false;
		bool gyrCalibrationInProgress = false;

		// Acelerômetro
		Vec3f accData;
		float accScale = 0.0f;
		uint8_t accFullScale = 0;
		uint8_t accDlpfSel = 0;

		// Temperatura
		float temData = 0.0f;
		
		// Magnetômetro
		float magScale = 0.1465f;
        Vec3f magData;
    };

};

#ifdef __cplusplus

#endif

#endif // I2CMANAGER_HPP

