#ifndef I2CMANAGER_HPP
#define I2CMANAGER_HPP

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <vector>

#define I2C_MASTER_SDA_IO GPIO_NUM_6   
#define I2C_MASTER_SCL_IO GPIO_NUM_7
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS   1000
#define I2C_MASTER_FREQ_HZ      CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_NUM_MAX_DEVICES     3

#ifdef __cplusplus
extern "C" {
#endif

class I2CManager {
public:
    I2CManager();
    ~I2CManager();

    void init();
    void deInit();
    void scanI2CDevices();
    void addDevice(uint8_t address);
    esp_err_t readRegFromDevice(uint8_t dev_address, uint8_t reg_address, uint8_t* data, size_t length);
    esp_err_t readRegFromDeviceWithHandle(i2c_master_dev_handle_t dev_handle, uint8_t reg_address, uint8_t* data, size_t length);
    void writeRegToDevice(uint8_t dev_address, uint8_t reg_address, uint8_t* data, size_t length);
    i2c_master_dev_handle_t* isDeviceInConfig(uint8_t address);
    esp_err_t writeRegToDeviceWithHandle(i2c_master_dev_handle_t dev_handle, uint8_t reg_address, uint8_t* data, size_t length);

private:
    int devices_added = 0;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_config_t bus_config;
    i2c_device_config_t dev_config;
    i2c_master_dev_handle_t dev_handle;
    std::vector<i2c_device_config_t> deviceConfigs = std::vector<i2c_device_config_t>(I2C_NUM_MAX_DEVICES);
    std::vector<i2c_master_dev_handle_t> deviceHandles = std::vector<i2c_master_dev_handle_t>(I2C_NUM_MAX_DEVICES);
};

#ifdef __cplusplus
}
#endif

#endif // I2CMANAGER_HPP
