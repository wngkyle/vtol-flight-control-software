# EleVTOL Flight Control Software

### Table of Content
- [Overview](#overview)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Mission & Goals](#mission-&-goals)
- [Software Design](#software-degisn)
- [Hardware Design](#hardware-desgin)

### Overview


### Hardware Requirements
- [Teensy 4.1](https://www.pjrc.com/store/teensy41.html)
  - Teensy 4.1, armed with an ARM Cortex-M7 processor running at 600MHz, serves as the backbone of this drone project's flight control system. Its processing power enables rapid data handling, digital signal processing, and swift communication which are critical factors for smooth flight operations. Compared to alternatives like Arduino Mega or Nano, which operate at much lower speeds (up to 16MHz), Teensy 4.1 has a significantly superior clock speed and performance. Moreover, its extensive connectivity options, including multiple USB, serial, I2C, SPI, CAN ports, FlexIO, and Ethernet, provide enhanced compatibility with diverse sensors and communication protocols. This versatility ensures seamless integration between the software and hardware, making our flight control system more adaptable to various technologies. With 8MB of flash memory and 1MB of RAM, Teensy 4.1 also surpasses the storage and memory capacities of Arduino Mega and Uno by a substantial margin. This expanded memory capacity allows for storing extensive code and efficient handling of data-heavy operations.
  - [IMXRT1060 Manual, Rev 3](https://www.pjrc.com/teensy/IMXRT1060RM_rev3.pdf), [IMXRT1060 Manual, Rev 2](https://www.pjrc.com/teensy/IMXRT1060RM_rev2.pdf), [IMXRT1060 Datasheet](https://www.pjrc.com/teensy/IMXRT1060CEC_rev0_1.pdf), [W25Q64JV-DTR Datasheet](https://www.pjrc.com/teensy/winbond_w25q64jvxgim.pdf)
- [GY-521 MPU6050 IMU]
  - [MPU-6050 Product Specification](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/mpu-6000-datasheet1.pdf)
  - [MPU-6050 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)


### Software Requirements


### Mission & Goals


### Software Design

### Hardware Design
