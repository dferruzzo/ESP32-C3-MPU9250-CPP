#ifndef I2CMANAGER_HPP
#define I2CMANAGER_HPP

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SDA_IO       GPIO_NUM_6   
#define I2C_MASTER_SCL_IO       GPIO_NUM_7
#define I2C_MASTER_PORT         I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS   1000

#ifdef __cplusplus
extern "C" {
#endif

class I2CManager {
public:
    I2CManager();
    ~I2CManager();

    void init();
    void scanI2CDevices();
    
private:
    i2c_master_bus_handle_t handle;
    i2c_master_bus_config_t conf;
};

#ifdef __cplusplus
}
#endif

#endif // I2CMANAGER_HPP