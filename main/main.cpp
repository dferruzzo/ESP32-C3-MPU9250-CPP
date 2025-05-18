// main.cpp

#include "I2CManager.h"
#include "MPU9250.h"
#include "NVSWrapper.h"
#include "sdkconfig.h"
#include <cinttypes>
#include <cstdio>
#include <iostream>

extern "C" {
#include "esp_chip_info.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "esp_flash.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
}

void exampleUsage() {
    try {
        NVSWrapper nvs;

        // Store primitive types
        nvs.write("temperature", 23.5f);
        nvs.write("counter", 42);

        // Store strings
        nvs.writeString("wifi_ssid", "MyNetwork");

        // Store vectors
        //std::vector<int> readings = {10, 20, 30, 40};
        //nvs.writeVector("sensor_readings", readings);

        // Retrieve data
        float temp;
        if (nvs.read("temperature", temp)) {
            std::cout << "Temperature: " << temp << std::endl;
        }

        std::string ssid;
        if (nvs.readString("wifi_ssid", ssid)) {
            std::cout << "WiFi SSID: " << ssid << std::endl;
        }

        //std::vector<int> saved_readings;
        //if (nvs.readVector("sensor_readings", saved_readings)) {
        //    std::cout << "Readings: ";
        //    for (auto val : saved_readings) {
        //        std::cout << val << " ";
        //    }
        //    std::cout << std::endl;
        //}
    } catch (const std::exception& e) {
        std::cerr << "NVS Error: " << e.what() << std::endl;
    }
}

extern "C" void app_main(void) {
  // Inicializa o barramento I2C
  I2CManager i2cManager;
  // i2cManager.scanI2CDevices();
  i2cManager.addDevice(MPU9250_ADDRESS);
  i2cManager.addDevice(MPU9250_MAGNETOMETER_ADDR);

  MPU9250 mpu9250(&i2cManager);

  // Testando as configuracões do giro.
  /*
  mpu9250.gyrConfig(MPU9250_GYRO_FS_SEL_1000,
                  MPU9250_FCHOICE_B_GYRO_FILTER_ENABLED,
                  MPU9250_GYRO_DLPF_CFG_20HZ);
*/
  // mpu9250.gyrCalibrate();

  // Testando conf do acelerômetro.
  // mpu9250.accConfig(MPU9250_ACCEL_FS_SEL_4,
  //		    MPU9250_ACCEL_NO_FIL_BW_1046Hz);

  // mpu9250.accCalibrate();
  /*
      while(1){
          mpu9250.gyrRead();
          //mpu9250.gyrGetRead();
          mpu9250.accRead();
          //mpu9250.accGetRead();
          mpu9250.temRead();
          //mpu9250.temGetRead();
          mpu9250.magRead();
          //mpu9250.magGetRead();
          mpu9250.printDataToTerminal();

          vTaskDelay(pdMS_TO_TICKS(250));
      }
  */
  // Testando o NVS
  exampleUsage();

  //  i2cManager.deInit();
}
