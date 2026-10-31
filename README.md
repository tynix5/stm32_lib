# stm32_lib
Collection of low-level drivers and high-level signal processing libraries for use in STM32 embedded systems

## Battery Management System (BMS) - WIP
A battery monitoring library for all types of batteries, monitoring battery voltage through the use of resistor voltage divider networks. Under development as of now to add support for different types of libraries and charging features instead of monitoring features. Possible improvements also include adding support for current measurements to determine true LiPo battery level.

## Encoder - WIP
A library designed to handle encoder events such as rotations and presses. Can be used to calculate velocity and maintains absolute encoder position. Need to add support for GPIO detection, interrupt detection, and different PPT modes. TIM support is currently available. 

## IIS2MDC - WIP
A library designed to support the IIS2MDC magnetometer sensor over SPI. Needs to be re-written for separation between the HAL layer and application layer.

## LSM6DS3TR
Designed for the LSM6DS3TR 6-DOF IMU (accelerometer + gyro) over SPI. Complete as of now, may need to add callbacks for interrupts later in the future or DMA support for reading/writing registers.

## nRF24L01
A multi-node library for the nRF24L01. Initial testing is complete with support for ACKs, ACKs with payload, and no ACKs. Static and dynamic payload lengths are both successful as well. May need to test the mesh capabilities with multiple transmitters and a single receiver.

## Quaternion
A library designed to support operations involving quaternions. Simple and easy to use.

## SSD1306
Designed for the SSD1306 OLED driver. Adds support for DMA and blocking I2C transactions, as well as multiple displays on the same bus. No need for further testing or support for now.