# ESP32-C3 MPU9250 using IDF

## Description

O ESP32-C3 utliza o sensor MPU9250 via I2C para ler os dados do acelerômetro,
girocópio e magnatómetro. O magnetómetro AK8963 é configurado no modo `slave`,
sendo lido diretamente pelo MPU9250.

Os três sensores são calibrados e os dados de calibração são armazenados na 
NVS (Non-Volatile Storage). Os dados lidos e calibrados são enviado no terminal 
através da porta serial (115200 bps).

## Wiring
```text
5.0v ───┬────────────────────────── 
3.3v ───┼────────────────┬─────────
GND  ───┼────┬─────┬─────┼────┬────
     ┌──┴────┴──┐  │  ┌──┴────┴──┐ 
     │ 5v   GND │  │  │ 3.3v GND │
     │          │  │  │ (IN)     │     
     │      3.3v├──┘  │          │     
     │     (OUT)│     │          │     
     │          │     │          │ 
     │ ESP32-C3 │     │ MPU-9250 │ 
     │          │     │          │ 
     │ SDA  SCL │     │ SDA  SCL │ 
     └──┬────┬──┘     └──┬────┬──┘ 
SDA  ───┴────┼───────────┴────┼─────
SCL  ────────┴────────────────┴─────    
```
---

## References

- MPU-9250 Product Specification: https://www.robocore.net/sensor-robo/acelerometro-giroscopio-magnetometro-mpu9250?srsltid=AfmBOoo23jLymdK4WUGxHYMZZzHECQpcHair-yioYxLZM4AKs-1ExNW6

---

## TODO

- [x] TODO: Calibrate mag. *O FATOR DE ESCALA FOI RETIRADO POR ENQUANTO. APENAS OFFSET FOI UTILIZADO*.
    - [x] TODO: Gravar dados no NVS. Na calibração verificar se há dados guardados.
    - [x] TODO: Verificar as unidades dos dados do magnetómetro (µT) e fazer ajustes se necessário.
    - [x] TODO: Go to MPU9250 Internal Master mode (BYPASS_EN = 0)
    - [x] TODO: Verificar ST2: Status 2: HOFL, BITM.
    - [x] TODO: Verificar CNTL1: MODE e BIT output setting.
    - [x] TODO: Criar função magConfig()
    - [x] TODO: Refazer a função magRead()
    - [x] TODO: Refazer a calibração do magnetómetro magCalibrate().
        - [x] TODO: Coletar dados em posições diferentes (criar uma func.).
        - [x] TODO: Plotar dados via /dev/tty* (COM) do mag em python `magplot.py`.
        - [x] TODO: Coletar amostras com distancia euclidiana mínima de 1uT (ou valor similiar apropriado).
        - [x] TODO: Verificar o calculo dos parâmetros da calibração (hard iron e soft iron) off-line no python. Tirei o fator de escala, dexei só o offset.
        - [x] TODO: Armazenar os dados coletados.
        - [x] TODO: store calibration parameters in NVS.
- [x] TODO: Create a function to read all raw data (accel, gyro, mag) at once.
- [x] TODO: Calculate the time spend in the previous  function.
- [x] TODO: Organizar todas as calibrações.
- [ ] TODO: Kalman filter.
- [ ] TODO: sensor fusion.

---

## No Windows - Docker - WSL 

[ESP-IDF Docker Container Documentation](https://docs.espressif.com/projects/vscode-esp-idf-extension/en/latest/additionalfeatures/docker-container.html)

Utilize o `.devcontainer.json` para carregar o Docker container.

No PowerShell:

Obtenha o `BUSID` do seu dispositivo
```sh
usbipd list
```
Anexe o dispositivo USB ao WSL
```sh
usbipd attach --wsl --busid 1-2 --auto-attach
```
para ter acesso ao usb no Windows dentro do container.

No ambiente linux, dar acesso ao `/dev/ttyACM0` ou `/dev/ttyUSB0`.
```bash
sudo chmod a+rw /dev/ttyACM0
```
---

## Ambiente virtual Python

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
``` 

---


