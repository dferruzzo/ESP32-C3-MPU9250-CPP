// MPU-9250 Register Map and Descriptions
// Reference: https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf

#ifndef MPU9250_CONFIGURATION_MAP_H
#define MPU9250_CONFIGURATION_MAP_H

/* Gyroscope configuration */

#define MPU9250_GYRO_FS_SEL_250     0x00 // 250 degrees/sec 
#define MPU9250_GYRO_FS_SEL_500     0x01 // 500 degrees/sec
#define MPU9250_GYRO_FS_SEL_1000    0x02 // 1000 degrees/sec
#define MPU9250_GYRO_FS_SEL_2000    0x03 // 2000 degrees/sec

#define MPU9250_GYRO_FS_SEL_250_VALUE     ((uint16_t)250) // 250 degrees/sec 
#define MPU9250_GYRO_FS_SEL_500_VALUE     ((uint16_t)500) // 500 degrees/sec
#define MPU9250_GYRO_FS_SEL_1000_VALUE    ((uint16_t)1000) // 1000 degrees/sec
#define MPU9250_GYRO_FS_SEL_2000_VALUE    ((uint16_t)2000) // 2000 degrees/sec

// FCHOICE_B 0 0 : DLPF enabled
// FCHOICE_B X 1 : DLPF bypassed, BW:8800Hz, Fs:32KHz 
// FCHOICE_B 1 X : DLPF bypassed, BW:3600Hz, Fs:32KHz  
#define MPU9250_FCHOICE_B_GYRO_FILTER_ENABLED   0x00 // Gyroscope DLPF Filter enabled
#define MPU9250_FCHOICE_B_GYRO_NO_FIL_BW_8800Hz 0x01 // No filter, BW:8800Hz, Fs:32KHz 
#define MPU9250_FCHOICE_B_GYRO_NO_FIL_BW_3600Hz 0x02 // No filter, BW:3600Hz, Fs:32KHz  

// If FCHOICE_B = 0 0, then DLPF_CFG bits are used to set the DLPF bandwidth
#define MPU9250_GYRO_DLPF_CFG_250HZ  0x00 // Gyro DLPF, BW:250Hz,  Fs:8KHz  
#define MPU9250_GYRO_DLPF_CFG_184HZ  0x01 // Gyro DLPF, BW:184Hz,  Fs:1KHz  
#define MPU9250_GYRO_DLPF_CFG_92HZ   0x02 // Gyro DLPF, BW:92Hz,   Fs:1KHz  
#define MPU9250_GYRO_DLPF_CFG_41HZ   0x03 // Gyro DLPF, BW:41Hz,   Fs:1KHz  
#define MPU9250_GYRO_DLPF_CFG_20HZ   0x04 // Gyro DLPF, BW:20Hz,   Fs:1KHz  
#define MPU9250_GYRO_DLPF_CFG_10HZ   0x05 // Gyro DLPF, BW:10Hz,   Fs:1KHz  
#define MPU9250_GYRO_DLPF_CFG_5HZ    0x06 // Gyro DLPF, BW:5Hz,    Fs:1KHz  
#define MPU9250_GYRO_DLPF_CFG_3600HZ 0x07 // Gyro DLPF, BW:3600Hz, Fs:8KHz  

/* Accelerometer configuration */

#define MPU9250_ACCEL_FS_SEL_2     0x00 // +/- 2g
#define MPU9250_ACCEL_FS_SEL_4     0x01 // +/- 4g   
#define MPU9250_ACCEL_FS_SEL_8     0x02 // +/- 8g
#define MPU9250_ACCEL_FS_SEL_16    0x03 // +/- 16g

#define MPU9250_ACCEL_FS_SEL_2_VALUE     ((uint16_t)2) // +/- 2g
#define MPU9250_ACCEL_FS_SEL_4_VALUE     ((uint16_t)4) // +/- 4g
#define MPU9250_ACCEL_FS_SEL_8_VALUE     ((uint16_t)8) // +/- 8g
#define MPU9250_ACCEL_FS_SEL_16_VALUE    ((uint16_t)16) // +/- 16g

#define MPU9250_ACCEL_NO_FIL_BW_1046Hz    0x00 // No filter, Fs:4KHz
#define MPU9250_ACCEL_FIL_EN_BW_218Hz     0x08 // Filter enabled, Fs:1KHz
#define MPU9250_ACCEL_FIL_EN_BW_218Hz_a   0x09 // Filter enabled, Fs:1KHz
#define MPU9250_ACCEL_FIL_EN_BW_99Hz      0x0A // Filter enabled, Fs:1KHz
#define MPU9250_ACCEL_FIL_EN_BW_44Hz      0x0B // Filter enabled, Fs:1KHz
#define MPU9250_ACCEL_FIL_EN_BW_21Hz      0x0C // Filter enabled, Fs:1KHz
#define MPU9250_ACCEL_FIL_EN_BW_10Hz      0x0D // Filter enabled, Fs:1KHz
#define MPU9250_ACCEL_FIL_EN_BW_5Hz       0x0E // Filter enabled, Fs:1KHz
#define MPU9250_ACCEL_FIL_EN_BW_420Hz     0x0F // Filter enabled, Fs:1KHz

//#define MPU9250_ACCEL_CONFIG_FS_SEL_MASK 0x18 // Mask for FS_SEL bits in the accelerometer configuration register
//#define MPU9250_ACCEL_CONFIG_FS_SEL_POS  3 // Position of FS_SEL bits in the accelerometer configuration register

#endif // MPU9250_CONFIGURATION_MAP_H
//
