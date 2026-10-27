# Automated Box Sorting System

Group project that classifies boxes as small (P), medium (M) or large (G) and stores them with two robotic arms. An ESP32 runs the decision logic and a web control panel, and an Arduino Mega drives the stepper motors.

<!-- ![Overview](images/hero.jpg) -->

## Overview

[TODO: 2 to 3 sentences on the goal and the main result.]

## System architecture

[TODO: validate this whole section against the final code before publishing.]

- ESP32 (PAT-DAELN v2.1 board): decision logic, FreeRTOS tasks, Wi-Fi web server and WebSocket control panel
- Arduino Mega 2560 with RAMPS 1.4 shield: executes the motor commands, connected to the ESP32 over I2C (address 0x08)
- Sensors: three KY-032 infrared barriers for size classification
- Actuators: seven stepper motors (NEMA 17, 28BYJ-48, AK85H)
- Interface: OLED display, RGB LED and buzzer alarm

## Results

[TODO: 47 of 47 cycles completed. Describe the test conditions.]

## Repository structure

```
firmware/esp32/          ESP32 source code
firmware/arduino-mega/   Arduino Mega source code
web-panel/               control panel served by the ESP32
hardware/                wiring diagrams and bill of materials
images/                  photos and screenshots
```

## How to reproduce

[TODO: tools, versions, libraries, upload steps.]

## Credits

Developed for the Microcontrollers course at UTFPR.
[TODO: teammates, written as they want to be credited.]

## Author

Felipe Martins, Mechatronics Engineering student at UTFPR, double degree at ENSTA Bretagne.
[LinkedIn](https://www.linkedin.com/in/felipe-m-k-m-martins)
