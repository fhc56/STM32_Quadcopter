\# STM32 Quadcopter



A custom STM32F103-based quadcopter project including flight-control firmware, remote-controller firmware, custom PCB design, and hardware implementation.



!\[Drone](Drone\_Fig.jpg)



\## Project Overview



This project is a self-developed quadcopter system based on STM32F103.



The project currently includes:



\- Flight controller firmware

\- Remote controller firmware

\- Custom flight controller PCB

\- MPU6050 attitude sensing

\- BMP280 barometer

\- Bluetooth communication

\- Motor PWM control

\- FreeRTOS integration



\## Hardware



\- STM32F103

\- MPU6050

\- BMP280

\- HC-08 Bluetooth module

\- 8520 coreless motors

\- Custom flight controller PCB



\## Software



\- STM32 Standard Peripheral Library

\- FreeRTOS

\- C language

\- Keil MDK

\- Quaternion attitude calculation

\- Mahony attitude estimation

\- PID control



\## Repository Structure



```text

STM32\_Quadcopter

├── Drone\_Code

│   └── Flight controller firmware

├── RmtCtrler\_Code

│   └── Remote controller firmware

├── Drone\_FlightCtrl\_Board.eprj2

│   └── Flight controller PCB project

├── Drone\_Fig.jpg

│   └── Quadcopter image

└── README.md

