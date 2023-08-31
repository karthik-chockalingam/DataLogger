# DataLogger
This project is designed around an ARM Cortex-M7 based STM32 microcontroller, featuring a comprehensive multi-threaded application running on the FreeRTOS operating system. 
The Data Logger communicates over Ethernet using the TCP protocol and hosts a TCP server, which interacts seamlessly with the Data Logger GUI TCP client.

![IMG_20230831_213329](https://github.com/karthik-chockalingam/DataLogger/assets/140360934/a5c25ae6-7de9-481a-989e-ba287d84ccc5)

## Hardware setup
- Ethernet for TCP/IP communication
- ADC peripheral for capturing analog input
- GPIO peripheral for capturing digital input
- RTC peripheral for appending time stamp to data measurement
- SPI for interfacing external SD card module

## Software setup
- FreeRTOS multi-threading application
- LWIP TCP/IP stack for hosting TCP server
- FATFS stack for file system and file IO operations

## Testing
- Analog input via 10K potentiometer connected to 3.3V
- Digital input via PWM based square wave generator 
