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
#include <eigen3/Eigen/Eigenvalues>
#include <iostream>
#include "nvs_flash.h"
#include "NVSUtils.h"
#include <math.h>

#define TAG "MPU9250"

#define MPU9250_ADDRESS				0x68 // Endereço I2C do MPU9250
#define MPU9250_MAGNETOMETER_ADDR		0x0C // Endereço I2C do AK8963 (magnetômetro)

// C++ code follows - no C linkage for templates and classes
struct Vec3f {
    float x;
    float y;
    float z;
    Vec3f() : x(0.0f), y(0.0f), z(0.0f) {}
    Vec3f(float x, float y, float z) : x(x), y(y), z(z) {}
};

// Standalone helper function - checks if an Eigen matrix is diagonal

template<typename Derived>
bool isDiagonalMatrix(const Eigen::MatrixBase<Derived>& matrix) {
    // Check if all off-diagonal elements are zero
    typedef typename Derived::Scalar Scalar;
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            if (i != j && matrix(i, j) != Scalar(0)) {
                return false;
            }
        }
    }
    return true;
}

// Standalone helper function - prints Eigen matrix using ESP_LOGI
template<typename Derived>
void printEigenMatrix(const char* tag, const char* name, const Eigen::MatrixBase<Derived>& matrix, int precision = 4) {
    ESP_LOGI(tag, "%s [%dx%d]:", name, (int)matrix.rows(), (int)matrix.cols());
    
    for (int i = 0; i < matrix.rows(); ++i) {
        // Build row string
        char row_buffer[256];
        int offset = 0;
        
        offset += snprintf(row_buffer + offset, sizeof(row_buffer) - offset, "  [");
        
        for (int j = 0; j < matrix.cols(); ++j) {
            if (j > 0) {
                offset += snprintf(row_buffer + offset, sizeof(row_buffer) - offset, ", ");
            }
            offset += snprintf(row_buffer + offset, sizeof(row_buffer) - offset, "%.*f", precision, (float)matrix(i, j));
        }
        
        offset += snprintf(row_buffer + offset, sizeof(row_buffer) - offset, "]");
        
        ESP_LOGI(tag, "%s", row_buffer);
    }
}

// Standalone helper function - prints Eigen vector using ESP_LOGI
template<typename Derived>
void printEigenVector(const char* tag, const char* name, const Eigen::MatrixBase<Derived>& vec, int precision = 4) {
	int rows = vec.rows();
	int cols = vec.cols();

	if (!(rows == 1 || cols == 1)) {
		ESP_LOGI(tag, "%s is not a vector (%dx%d)", name, rows, cols);
		return;
	}

	int len = (rows == 1) ? cols : rows;
	ESP_LOGI(tag, "%s [%dx%d]:", name, rows, cols);

	char row_buffer[256];
	for (int i = 0; i < len; ++i) {
		int offset = 0;
		offset += snprintf(row_buffer + offset, sizeof(row_buffer) - offset, "  [%d] ", i);
		float val = static_cast<float>((rows == 1) ? vec(0, i) : vec(i, 0));
		offset += snprintf(row_buffer + offset, sizeof(row_buffer) - offset, "%.*f", precision, val);
		ESP_LOGI(tag, "%s", row_buffer);
	}
}

class MPU9250 
{

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
		esp_err_t magConfig();
		bool forceMagCalibration = false; // Força a calibração do magnetômetro
		esp_err_t readMagnetometerASA();
		esp_err_t magRead();
		esp_err_t magRead_old();
		esp_err_t magGetRead();
		esp_err_t magGetRead_term();
		esp_err_t magCalibrate(PL::NvsNamespace& nvs);
		esp_err_t magCalibrate_old(PL::NvsNamespace& nvs);
		esp_err_t magGetSamples(float* matrix, int rows, int cols);
		
		/* Temperatura */
        	esp_err_t temRead();
		esp_err_t temGetRead();

		void timer(uint8_t seconds, bool showCountdown = true);	

		void printDataToTerminal();

	private:

		/* Configuração do I2C */
		I2CManager* i2cManager;
		uint8_t deviceAddress = MPU9250_ADDRESS;
		uint8_t deviceAddressMag = MPU9250_MAGNETOMETER_ADDR;
		i2c_master_dev_handle_t* MPU9250_handle_ptr = nullptr;
		i2c_master_dev_handle_t* MPU9250_mag_handle_ptr = nullptr;
		esp_err_t enableI2CMaster();
    		bool I2CMasterEnabled = false;
		
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
		float 	magScaleX;
		float 	magScaleY;
		float 	magScaleZ;
		float	magScale = 4912.0f / 32760.0f; // Escala padrão do AK8963 em microteslas por LSB
       		Vec3f	magData = Vec3f(0.0f, 0.0f, 0.0f);
		int8_t	magNumSamplesCal = 50;	// Número de amostras para calibração do acc.
		bool	magCalibrated = false;
		bool 	magCalibrationInProgress = false;
		bool 	magCalibrationFailed = false;
		Eigen::MatrixXf W_inv; 	// Matriz de calibração do magnetômetro
		Eigen::VectorXf V; 	// Vetor de calibração do magnetômetro
		float	magFieldStrength(); // Força do campo magnético em microteslas (uT)
		bool 	magReadMagASA = false;
		bool 	magConfigured = false;
		esp_err_t writeAK8963RegisterViaSLV0(uint8_t ak8963_reg, uint8_t data);
		esp_err_t readMagnetometerASAViaSLV1();
		bool isMagSampleOK(int rows, int cols, float* vector, float* matrix);
		Eigen::Vector3f Bc; // Magnetometer corrected
	    	Eigen::Vector3f Bp; // Magnetometer raw
		
		/* Temperatura */
		float	temData = 0.0f;
};

#endif // I2CMANAGER_HPP

