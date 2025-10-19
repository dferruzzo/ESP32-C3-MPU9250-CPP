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
#include "nvs_flash.h"
#include "NVSUtils.h"

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

		/* Girsocópio */
		bool forceGyroCalibration = false; // Força a calibração do giroscópio
		esp_err_t gyrConfig(uint8_t fullScaleSel, uint8_t enableFilter, uint8_t gyroDlpfCfg);
		esp_err_t gyrCalibrate(PL::NvsNamespace& nvs);
		esp_err_t gyrRead();
		esp_err_t gyrGetRead();
		void getGyrFullScale();

		/* Acelerômetro */
		bool forceAccCalibration = false; // Força a calibração do acelerômetro
		esp_err_t accConfig(uint8_t fullScaleSel, uint8_t accelDlpfCfg);
		esp_err_t accRead();
		esp_err_t accCalibrate(PL::NvsNamespace& nvs);
		esp_err_t accGetRead();

		/* Magnetômetro */
		//esp_err_t magConfig();
		bool forceMagCalibration = false; // Força a calibração do acelerômetro
		esp_err_t magRead();
		esp_err_t magGetRead();
		esp_err_t magCalibrate(PL::NvsNamespace& nvs);

		/* Temperatura */
        	esp_err_t temRead();
		esp_err_t temGetRead();

		void timer(uint8_t seconds);

		void printDataToTerminal();

	private:

		/* Configuração do I2C */
		I2CManager* i2cManager;
		uint8_t deviceAddress = MPU9250_ADDRESS;
		uint8_t deviceAddressMag = MPU9250_MAGNETOMETER_ADDR;
		i2c_master_dev_handle_t* MPU9250_handle_ptr = nullptr;
		i2c_master_dev_handle_t* MPU9250_mag_handle_ptr = nullptr;
		
		/* Gyroscope data and calibration */
		Vec3f gyroData = Vec3f(0.0f, 0.0f, 0.0f);
		Vec3f gyroBias = Vec3f(0.0f, 0.0f, 0.0f);

		float	gyrScale = 0.0f;
		uint8_t gyrFullScale = 0;
		bool	gyrCalibrated = false;
		bool	gyrCalibrationInProgress = false;
		//uint16_t gyroNumSamples = 500; // Inicializado com 0

        	/* Acelerômetro */
		Vec3f	accData = Vec3f(0.0f, 0.0f, 0.0f);
		float	accScale = 0.0f;
		uint8_t	accFullScale = 0;
		uint8_t accDlpfSel = 0;
		int8_t	accNumSamplesCal = 10;	// Número de amostras para calibração do acc.
		int8_t	accFactorCal = 10; // Fator de escala para os dados do acelerômetro durante a calibragem
		bool	accCalibrated = false;
		bool	accCalibrationInProgress = false;
		Eigen::MatrixXf accCalibrationMatrix; // Matriz de calibração do acelerômetro
		
		/* Magnetômetro */
		float	magScale = 0.1465f;
        	Vec3f	magData = Vec3f(0.0f, 0.0f, 0.0f);
		int8_t	magNumSamplesCal = 100;	// Número de amostras para calibração do acc.
		bool	magCalibrated = false;
		bool 	agCalibrationInProgress = false;
                
		/* Temperatura */
		float	temData = 0.0f;
		
    };

};

#ifdef __cplusplus

#endif

#endif // I2CMANAGER_HPP

