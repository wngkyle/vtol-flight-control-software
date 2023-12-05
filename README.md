# EleVTOL Flight Control Software

### Table of Content
- [Overview](#overview)
- [Mission & Goal](#mission-&-goal)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Software Design](#software-design)
- [Hardware Design](#hardware-design)

### Overview

 - The aspiration for urban airborne transportation has captivated minds for decades, envisaging a realm of flying cars and personal aerial vehicles. Recent focus on eVTOL (electric vertical take-off and landing) aircraft signifies an unprecedented spotlight on the future of urban air mobility. Despite considerable strides within the industry, the complexity of this pursuit has thwarted the emergence of a truly revolutionary product. Present eVTOL vehicles, constrained by current technological limitations, exhibit significant drawbacks and lack the compelling incentives necessary for widespread adoption. This project endeavors to not only learn from the successful strategies of eVTOL pioneers but also to innovate and prototype our aircraft, striving to overcome existing limitations and pioneer a transformative solution.

### Mission & Goal

#### - Our mission is to make air travel accessible, seamless, and sustainable.

### Hardware Requirements
- [Teensy 4.1](https://www.pjrc.com/store/teensy41.html)
  - Teensy 4.1, armed with an ARM Cortex-M7 processor running at 600MHz, serves as the backbone of this drone project's flight control system. Its processing power enables rapid data handling, digital signal processing, and swift communication which are critical factors for smooth flight operations. Compared to alternatives like Arduino Mega or Nano, which operate at much lower speeds (up to 16MHz), Teensy 4.1 has a significantly superior clock speed and performance. Moreover, its extensive connectivity options, including multiple USB, serial, I2C, SPI, CAN ports, FlexIO, and Ethernet, provide enhanced compatibility with diverse sensors and communication protocols. This versatility ensures seamless integration between the software and hardware, making our flight control system more adaptable to various technologies. With 8MB of flash memory and 1MB of RAM, Teensy 4.1 also surpasses the storage and memory capacities of Arduino Mega and Uno by a substantial margin. This expanded memory capacity allows for storing extensive code and efficient handling of data-heavy operations.
  - [IMXRT1060 Manual, Rev 3](https://www.pjrc.com/teensy/IMXRT1060RM_rev3.pdf), [IMXRT1060 Manual, Rev 2](https://www.pjrc.com/teensy/IMXRT1060RM_rev2.pdf), [IMXRT1060 Datasheet](https://www.pjrc.com/teensy/IMXRT1060CEC_rev0_1.pdf), [W25Q64JV-DTR Datasheet](https://www.pjrc.com/teensy/winbond_w25q64jvxgim.pdf)
- GY-521 MPU6050 IMU
  - [MPU-6050 Product Specification](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/mpu-6000-datasheet1.pdf)
  - [MPU-6050 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)


### Software Requirements


### Software Design

### Hardware Design
- Aircraft Concept Overview
  - This project adopts a direct-lift VTOL aircraft configuration using pre-built EDFs. Unlike quadcopter-like setups, it aligns closely with fixed-wing aircraft principles. All engines are utilized for every flight phase, eliminating redundant weight. The initial design features a tri-engine setup functioning as a tricopter for VTOL, transitioning to propel the aircraft forward for level flight after reaching a safe altitude. Development is ongoing for the control mechanism facilitating this transition.
- Sizing and Design Parameters
  - The focus on urban air mobility necessitates a compact size. The aircraft's size constraint matches that of a standard parking space, ensuring a balance between adaptability and performance. Proposed solutions in the preliminary design phase involve folding wings and a streamlined fuselage to accommodate a single passenger, enabling the aircraft to fit into a parking space in both its landed and VTOL configurations.
  - The initial aircraft design incorporates a tapered wing with a low aspect ratio. Based on current estimates of wing geometry and gross weight, the projected aircraft, when carrying a passenger/pilot of up to 100 [kg], is anticipated to maintain level flight at 87 [kph]. For the scaled-down engineering sample that is to be built for this project, this value stands at 30 [kph]. It's important to note that the lift coefficient is currently underestimated, and the lift contribution from the fuselage and tail is disregarded in this calculation. Consequently, these velocity estimations are deemed conservative due to these factors not being fully accounted for.
