# TODO

- [x] Test: void I2CManager::writeRegToDevice(uint8_t dev_address, uint8_t reg_address, uint8_t* data, size_t length);
- [x] Implement: esp_err_t I2Cmanager::writeRegFromDeviceWithHandle(i2c_master_dev_handle_t dev_handle, uint8_t reg_address, uint8_t* data, size_t length);
- [x] Configure MPU9250 gyroscope.
- [x] Read gyro.
- [x] Calibrate gyro (only bias).
- [x] Configure MPU9250 accelerometer.
- [x] Read acc.
- [x] Read temperature.
- [x] Calibrate temp.
- [x] Configure MPU9250 magnetometer.
- [x] Read mag.
- [x] python plot app.
- [x] Install Eigen.
- [x] Test Eigen.
- [x] Calibrate acc.
    - [x] Calcular matriz Y.
    - [x] Calcular matriz W.
- [x] Gravar dados de calibração na EEPROM?
    - [x] Revisar a função bool handleError(esp_err_t err, bool ignoreNotFound = false); no header NVSWrapper.h, pode estar produzindo um erro. CANCELADO!
    - [x] Utilizando pl_nvs component.
    - [x] Implementar as funções que gravam os dados no NVS.
    - [x] Gravar bias do gyro. Na calibração verificar se há armazanado bias.
    - [x] Gravar dados de calibração do aceleómetro. Na calibração verificar se há dados armazenados.
- [ ] Calibrate mag.
    - [ ] Gravar dados no NVS. Na calibração verificar se há dados guardados.
- [ ] Kalman filter.
- [ ] sensor fusion.

---

## No Windows - Docker - WSL 

[ESP-IDF Docker Container Documentation](https://docs.espressif.com/projects/vscode-esp-idf-extension/en/latest/additionalfeatures/docker-container.html)

Utilize o `.devcontainer.json` para carregar o Docker container.

No PowerShell:

```bash
 usbipd attach --wsl --busid 1-2 --auto-attach
```

para ter acesso ao usb no Windows dentro do container.