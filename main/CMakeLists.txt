idf_component_register(SRCS "main.cpp"
                    REQUIRES driver
                    INCLUDE_DIRS ""
                    PRIV_REQUIRES I2CManager MPU9250)
