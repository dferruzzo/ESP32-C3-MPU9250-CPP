# Magnetômetro MPU-9250 AK8963

---

## Configuração do Magnetômetro

* [ ] Configurar o MPU-9250 com `Auxiliary I2C Master Mode` para ler o magnetômetro AK8963 embutido.
* [ ] Configurar o AK8963 para o modo de medição contínua com resolução de 16 bits.
* [ ] Registers 73 to 96 – External Sensor Data Registers.
* [ ] Configurar a taxa de amostragem do magnetômetro para 100 Hz.
* [ ] Considerar uso do FIFO do MPU-9250.
* [x] I2C needs pull-up resistors (4.7k - 10k ohm) on SDA and SCL lines.

---

## Configuração do AK8963

* *Register 0x24(hex), 36(dec) – I2C_MST_CNTL - I2C Master Control*

    |7|6|5|4|3|2|1|0|

    |7|: MULT_MST_EN = 0 // 0: single read/write, 1: multi read/write. 
    |6|: WAIT_FOR_ES = 0 // 0: do not wait, 1: wait for external sensor data ready signal.
    |5|: SLV_3_FIFO_EN = 0 // 0: disable, 1: enable slave 3 FIFO.
    |4|: I2C_MST_P_NSR = 0 // 0: stop I2C master after transaction, 1: do not stop I2C master.
    |3-0|: I2C_MST_CLK[3:0] = 0000 = 0 // I2C master clock speed = 348 kHz (refer to Table 8 in the MPU-9250 Register Map and Descriptions document).
 
    __value = 00000000(b) = 0x0D(hex) = 13(dec) (400Hz)__

---

### I2C slave 0 Control - Registers 37 to 39

---

* *Register 0x25(hex), 37(dec) – I2C_SLV0_ADDR*

    |7|6|5|4|3|2|1|0|

    |7|: I2C_SLV0_RNW = 1 // 0: write, 1: read.
    |6|5|4|3|2|1|0|: I2C_SLV0_ADDR[6:0] = 0011110 = 0x0C // I2C address of AK8963 with read bit (0x0C is the 7-bit address of AK8963).

    __value = 10001110(b) = 0x8E(hex) = 142(dec): Read from AK8963__
    __value = 00001110(b) = 0x0E(hex) = 14(dec): Write to AK8963__

---

* *Register Ox26(hex), 38(dec) – I2C_SLV0_REG*

    |7|6|5|4|3|2|1|0|

    |7-0|: I2C_SLV0_REG[7:0] = 000000010 = 0x02 // Starting register address to read from AK8963 (ST1 register).

    __value = 00000010(b) = 0x02(hex) = 2(dec)__

---

* *Register 27(hex), 39(dec) – I2C_SLV0_CTRL*

    |7|6|5|4|3|2|1|0|

    |7|: I2C_SLV0_EN = 1
    |6|: I2C_SLV0_BYTR_SW = 0 // 0: no swap, 1: swap bytes.
    |5|: I2C_SLV0_REG_DIS = 0 // 0: use register address, 1: do not use register address. 
    |4|: I2C_SLV0_GRP = 0 // 0: do not group, 1: group with other slaves.
    |3|2|1|0|: I2C_SLV0_LEN[3:0] = 1000 = 8 bytes (Status 1 + 6 data bytes + Status 2) // Number of bytes to be read from I2C slave 0 

    __value = 10001000(b) = 0x88(hex) = 136(dec)__

---

* *Register 0x63(hex), 99(dec) - I2C Slave 0 Data Out* 
    
    I2C_SLV0_DO is used to write data to the slave device. This register is not used when reading data from the AK8963.

---

* *Register 0x6A(hex), 106(dec) - User Control*

    |7|: Reserved                                VALUE: 0   
    |6|: FIFO_EN:       1 - ENABLE, 0 - DISABLE, VALUE: 0
    |5|: I2C_MST_EN:    1 - ENABLE, 0 - DISABLE, VALUE: 1
    |4|: I2C_IF_DIS:    1 - ENABLE, 0 - DISABLE, VALUE: 0
    |3|: Reserved                                VALUE: 0
    |2|: FIFO_RST:      1 - ENABLE, 0 - DISABLE, VALUE: 0
    |1|: I2C_MST_RST:   1 - ENABLE, 0 - DISABLE, VALUE: 0
    |0|: SIG_COND_RST:  1 - ENABLE, 0 - DISABLE, VALUE: 0

    __value: 00100000 = 0x20__     

--- 
## Calibração do Magnetômetro

[TechReport (Pedley2014): Magnetic Calibration (magnetic.c) Technical Note, Freescale, 2014](https://community.nxp.com/pwmxy87654/attachments/pwmxy87654/sensors/2638/1/Magnetic%20Calibration.pdf)
