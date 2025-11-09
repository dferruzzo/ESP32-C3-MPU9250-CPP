idf_component_register(SRCS "main.cpp"
                    REQUIRES driver I2CManager MPU9250 NVSUtils
                    INCLUDE_DIRS ""
		            PRIV_REQUIRES I2CManager MPU9250 NVSUtils)
