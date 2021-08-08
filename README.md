2020 I483 Sensor Device
====

# Description

This repository contains an example application for 2020 I483 which measures
temperature, humidity and pressure with BME280 and converts the measurements
on ESP32 and then broadcasts the data as a BLE advertisement as well as send
it to the cloud in Ultralight 2.0 format simultaneously.

# Schematics

```
VCC --------------------- [ESP32 16 3V3   ]
GND --------------------- [ESP32 17 GND   ]

GND - [LED1] - [47 OHM] - [ESP32 24 GPIO18]
GND - [LED2] - [47 OHM] - [ESP32 25 GPIO19]

GND ---- [BME280 6 SDO]
VCC ---- [BME280 5 CSB]
         [BME280 4 SDA] - [ESP32 26 GPIO21]
         [BME280 3 SCL] - [ESP32 27 GPIO22]
GND ---- [BME280 2 GND]
VCC ---- [BME280 1 VCC]
```
