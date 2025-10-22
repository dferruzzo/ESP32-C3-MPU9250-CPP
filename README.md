# ESP32-C3 MPU9250 using IDF

```text
5.0v ───┬─────────────────────────────┬─────────── 
3.3v ───┼────────────────┬────────────┼───────────
GND  ───┼────┬─────┬─────┼────┬───────┼────┬────── 
        │    │     │     │    │       │    │  
     ┌──┴────┴──┐  │  ┌──┴────┴──┐ ┌──┴────┴───┐
     │ 5v   GND │  │  │ 3.3v GND │ │ 5v   GND  │
     │          │  │  │ (IN)     │ │(IN)       │    
     │      3.3v├──┘  │          │ │           │    
     │     (OUT)│     │          │ │           │    
     │          │     │          │ │ oLED 0.96 │
     │ ESP32-C3 │     │ MPU-9250 │ │  display  │
     │          │     │          │ │           │
     │ SDA  SCL │     │ SDA  SCL │ │ SDA  SCL  │
     └──┬────┬──┘     └──┬────┬──┘ └──┬─────┬──┘
SDA  ───┴────┼───────────┴────┼───────┴─────┼────
SCL  ────────┴────────────────┴─────────────┴────    
```

---

## References

- oLED 0.96 inch I2C Display Module: https://www.robocore.net/display/display-oled-96-i2c-branco?srsltid=AfmBOoqyZVngfxVxPmDzGPSCy0RQe2KHnopfTRR7fxBTcZMxMTMocY7K
- MPU-9250 Product Specification: https://www.robocore.net/sensor-robo/acelerometro-giroscopio-magnetometro-mpu9250?srsltid=AfmBOoo23jLymdK4WUGxHYMZZzHECQpcHair-yioYxLZM4AKs-1ExNW6

---

## TODO

- [x] TODO: Test: void I2CManager::writeRegToDevice(uint8_t dev_address, uint8_t reg_address, uint8_t* data, size_t length);
- [x] TODO: Implement: esp_err_t I2Cmanager::writeRegFromDeviceWithHandle(i2c_master_dev_handle_t dev_handle, uint8_t reg_address, uint8_t* data, size_t length);
- [x] TODO: Configure MPU9250 gyroscope.
- [x] TODO: Read gyro.
- [x] TODO: Calibrate gyro (only bias).
- [x] TODO: Configure MPU9250 accelerometer.
- [x] TODO: Read acc.
- [x] TODO: Read temperature.
- [x] TODO: Calibrate temp.
- [x] TODO: Configure MPU9250 magnetometer.
- [x] TODO: Read mag.
- [x] TODO: python plot app.
- [x] TODO: Install Eigen.
- [x] TODO: Test Eigen.
- [x] TODO: Calibrate acc.
    - [x] TODO: Calcular matriz Y.
    - [x] TODO: Calcular matriz W.
- [x] TODO: Gravar dados de calibração na NVS (Non-volatile storage)?
    - [x] TODO: Revisar a função bool handleError(esp_err_t err, bool ignoreNotFound = false); no header NVSWrapper.h, pode estar produzindo um erro. CANCELADO!
    - [x] TODO: Utilizando pl_nvs component.
    - [x] TODO: Implementar as funções que gravam os dados no NVS.
    - [x] TODO: Gravar bias do gyro. Na calibração verificar se há armazanado bias.
    - [x] TODO: Gravar dados de calibração do aceleómetro. Na calibração verificar se há dados armazenados.
- [ ] TODO: Calibrate mag.
    - [ ] TODO: Go to MPU9250 Internal Master mode (BYPASS_EN = 0) <<<---!!!
    - [x] TODO: Gravar dados no NVS. Na calibração verificar se há dados guardados.
    - [x] TODO: Verificar as unidades dos dados do magnetómetro (µT) e fazer ajustes se necessário.
    - [ ] TODO: Criar função magConfig()
    - [ ] TODO: Verificar ST2: Status 2: HOFL, BITM.
    - [ ] TODO: Verificar CNTL1: MODE e BIT output setting.
    - [ ] Implementar `checkDataDiversity()` para os dados coletados (distancia euclidiana entre amostras deve ser maior que 10 uT)
- [ ] TODO: Kalman filter.
- [ ] TODO: sensor fusion.

---

## No Windows - Docker - WSL 

[ESP-IDF Docker Container Documentation](https://docs.espressif.com/projects/vscode-esp-idf-extension/en/latest/additionalfeatures/docker-container.html)

Utilize o `.devcontainer.json` para carregar o Docker container.

No PowerShell:

Obtenha o `BUSID` do seu dispositivo

```bash
usbipd list
```

```bash
usbipd attach --wsl --busid 1-2 --auto-attach
```

para ter acesso ao usb no Windows dentro do container.
