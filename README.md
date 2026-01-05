Integrated Linux Device Drivers Project

Overview

This project demonstrates the development and integration of multiple Linux device drivers into a single kernel module on an embedded Linux platform. The goal of this project is to gain hands-on experience with Linux kernel programming, hardware communication protocols, and driver integration concepts.

The project was developed as a self-driven learning initiative to strengthen practical embedded Linux skills.

Features

Integrated multiple hardware interfaces in a single kernel module

Implemented device drivers for:

OLED Display (SSD1306)

BMP280 Temperature & Pressure Sensor

SPI-based SD Card Interface

UART Loopback using serdev framework

User-space interaction through a character device

Device control based on user input

Hardware Used

  Raspberry Pi 3B+
  SSD1306 OLED Display (I2C)
  BMP280 Sensor (I2C)
  SD Card Module (SPI)
  UART Interface (Loopback testing)
  
Software & Tools

  Embedded C
  Linux Kernel Programming
  Linux Kernel Modules
  Device Tree Overlays
  GCC
  Make
  Linux OS (Raspberry Pi OS 64-bit)
  
Driver Architecture

  The project consists of multiple driver components organized into a single main driver source file.

Driver Components

OLED display driver logic and font tables
BMP280 sensor driver implementation
SPI-based SD card communication logic
UART loopback driver using serdev

integrated_driver.c – Main driver file handling:

Module initialization and exit

Character device creation

Read/write operations

Control logic for different peripherals

Functional Description

The kernel module registers a character device

User-space applications send commands to the driver

Based on user input:

Sensor data is read from BMP280

Data is displayed on SSD1306 OLED

SPI communication is triggered for SD card operations

UART loopback communication is tested

Synchronization and safe user-kernel data transfer is handled using copy_from_user and copy_to_user

How to Build
make

How to Load the Driver
sudo insmod integrated_driver.ko

How to Remove the Driver
sudo rmmod integrated_driver

Debugging
dmesg | tail

Learning Outcomes

Through this project, I gained hands-on experience in:

Linux kernel module development

Device driver architecture and integration

SPI, I2C, and UART communication

Device Tree configuration

Debugging kernel-level issues

Interfacing sensors and displays on embedded Linux

Future Improvements

Improve error handling and return status

Add ioctl support for better user-driver communication

Separate drivers into loadable independent modules

Power management and performance optimization

Disclaimer

This project is intended for learning and demonstration purposes. It is not optimized for production use.

Author

Dinesh Loya
Embedded Engineer
GitHub: 
