// MPU-9250 Register Map and Descriptions
// Reference: https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf

#ifndef MPU9250_REGISTER_MAP_H
#define MPU9250_REGISTER_MAP_H

// MPU-9250 Register Addresses
#define MPU9250_SELF_TEST_X_GYRO    0x00
#define MPU9250_SELF_TEST_Y_GYRO    0x01
#define MPU9250_SELF_TEST_Z_GYRO    0x02
#define MPU9250_SELF_TEST_X_ACCEL   0x0D
#define MPU9250_SELF_TEST_Y_ACCEL   0x0E
#define MPU9250_SELF_TEST_Z_ACCEL   0x0F
#define MPU9250_XG_OFFSET_H         0x13
#define MPU9250_XG_OFFSET_L         0x14
#define MPU9250_YG_OFFSET_H         0x15
#define MPU9250_YG_OFFSET_L         0x16
#define MPU9250_ZG_OFFSET_H         0x17
#define MPU9250_ZG_OFFSET_L         0x18
#define MPU9250_SMPLRT_DIV          0x19
#define MPU9250_CONFIG              0x1A
#define MPU9250_GYRO_CONFIG         0x1B
#define MPU9250_ACCEL_CONFIG        0x1C
#define MPU9250_ACCEL_CONFIG_2      0x1D
#define MPU9250_LP_ACCEL_ODR        0x1E
#define MPU9250_WOM_THR             0x1F
#define MPU9250_FIFO_EN             0x23
#define MPU9250_I2C_MST_CTRL        0x24
#define MPU9250_I2C_SLV0_ADDR       0x25
#define MPU9250_I2C_SLV0_REG        0x26
#define MPU9250_I2C_SLV0_CTRL       0x27
#define MPU9250_I2C_SLV1_ADDR       0x28
#define MPU9250_I2C_SLV1_REG        0x29
#define MPU9250_I2C_SLV1_CTRL       0x2A
#define MPU9250_I2C_SLV2_ADDR       0x2B
#define MPU9250_I2C_SLV2_REG        0x2C
#define MPU9250_I2C_SLV2_CTRL       0x2D
#define MPU9250_I2C_SLV3_ADDR       0x2E
#define MPU9250_I2C_SLV3_REG        0x2F
#define MPU9250_I2C_SLV3_CTRL       0x30
#define MPU9250_I2C_SLV4_ADDR       0x31
#define MPU9250_I2C_SLV4_REG        0x32
#define MPU9250_I2C_SLV4_DO         0x33
#define MPU9250_I2C_SLV4_CTRL       0x34
#define MPU9250_I2C_SLV4_DI         0x35
#define MPU9250_I2C_MST_STATUS      0x36
#define MPU9250_INT_PIN_CFG         0x37
#define MPU9250_INT_ENABLE          0x38
#define MPU9250_INT_STATUS          0x3A
#define MPU9250_ACCEL_XOUT_H        0x3B
#define MPU9250_ACCEL_XOUT_L        0x3C
#define MPU9250_ACCEL_YOUT_H        0x3D
#define MPU9250_ACCEL_YOUT_L        0x3E
#define MPU9250_ACCEL_ZOUT_H        0x3F
#define MPU9250_ACCEL_ZOUT_L        0x40
#define MPU9250_TEMP_OUT_H          0x41
#define MPU9250_TEMP_OUT_L          0x42
#define MPU9250_GYRO_XOUT_H         0x43
#define MPU9250_GYRO_XOUT_L         0x44
#define MPU9250_GYRO_YOUT_H         0x45
#define MPU9250_GYRO_YOUT_L         0x46
#define MPU9250_GYRO_ZOUT_H         0x47
#define MPU9250_GYRO_ZOUT_L         0x48
#define MPU9250_SIGNAL_PATH_RESET   0x68
#define MPU9250_ACCEL_INTEL_CTRL    0x69
#define MPU9250_USER_CTRL           0x6A
#define MPU9250_PWR_MGMT_1          0x6B
#define MPU9250_PWR_MGMT_2          0x6C
#define MPU9250_FIFO_COUNTH         0x72
#define MPU9250_FIFO_COUNTL         0x73
#define MPU9250_FIFO_R_W            0x74
#define MPU9250_WHO_AM_I            0x75
#define MPU9250_XA_OFFSET_H         0x77
#define MPU9250_XA_OFFSET_L         0x78
#define MPU9250_YA_OFFSET_H         0x7A
#define MPU9250_YA_OFFSET_L         0x7B
#define MPU9250_ZA_OFFSET_H         0x7D
#define MPU9250_ZA_OFFSET_L         0x7E

/* MAgnetometer */

#define MPU9250_MAG_WIA_ADDR        0x00 // Address to test
#define MPU9250_MAG_DATA_RDY        0x02 // Data ready bit
#define MPU9250_MAG_HXL             0x03
#define MPU9250_MAG_HXH             0x04
#define MPU9250_MAG_HYL             0x05
#define MPU9250_MAG_HYH             0x06
#define MPU9250_MAG_HZL             0x07
#define MPU9250_MAG_HZH             0x08
#define MPU9250_MAG_CNTL            0x0A



#endif // MPU9250_REGISTER_MAP_H