# STM32-Based Smart 4WD Robot Car with Bluetooth and UART Remote Control

## Project Overview
This project implements a four-wheel-drive (4WD) robot car controlled via Bluetooth and UART. The system is powered by STM32 microcontrollers, combining embedded systems expertise with motor control, remote communication, and basic obstacle detection.

## Features
- **Remote Control**: Control the robot car via Bluetooth commands sent over UART.
- **Motor Control**: Use PWM signals to drive and adjust motor speed and direction.
- **Obstacle Detection**: Basic distance measurement using an ultrasonic sensor (HCSR05).
- **Sound Effects**: Generate melodies and emergency tones via a buzzer using PWM frequency modulation.

## Hardware Components
1. **Microcontrollers**:
   - STM32F411E-DISCOVERY (Car controller)
   - STM32H747I-DISCO (Remote controller)
2. **Actuators**:
   - Four DC motors for 4WD functionality
3. **Sensors**:
   - HCSR05 Ultrasonic Distance Sensor
4. **Communication**:
   - Bluetooth module
   - UART serial communication
5. **Audio**:
   - Buzzer for sound effects and music

## Software Implementation
### System Architecture
The project is implemented using the STM32 HAL library. The main program handles:
- **Initialization**: Configures GPIO, UART, PWM, and timers.
- **Control Logic**: Interprets UART commands for controlling motor speed, direction, and sound effects.
- **Distance Measurement**: Uses the HCSR05 ultrasonic sensor to detect obstacles.

### Core Functionalities
#### Motor Control
- PWM signals control motor speed and direction.
- Functions:
  - `Forward_Speed(speed)`
  - `Backward_Speed(speed)`
  - `Left_Speed(speed)`
  - `Right_Speed(speed)`

#### Ultrasonic Distance Sensing
- Measures distance using the HCSR05 sensor.
- Stops the car and triggers an emergency tone when an obstacle is detected within 5 cm.

#### Sound Effects
- Play predefined melodies such as "Twinkle Twinkle Little Star" and "Happy Birthday" using the buzzer.
- Functions:
  - `PlayTwinkleTwinkle()`
  - `PlayHappyBirthday()`
  - `PlayEmergencyTone()`

#### Bluetooth Command Set
- **Movement**:
  - `w`: Move forward
  - `s`: Move backward
  - `a`: Turn left
  - `d`: Turn right
- **Speed Adjustment**:
  - `h`: Increase speed
  - `l`: Decrease speed
- **Sound Control**:
  - `1`: Play "Twinkle Twinkle Little Star"
  - `2`: Play "Happy Birthday"
  - `3`: Play "Ode to Joy"
  - `0`: Stop sound
- **Stop**:
  - `p`: Stop all movement

## Setup and Usage
### Prerequisites
- STM32CubeIDE installed on your computer
- Hardware setup as listed above

### Steps to Run
1. **Hardware Setup**:
   - Connect the motors, sensors, and Bluetooth module to the STM32F411E-DISCOVERY board.
   - Ensure the buzzer is connected to the correct GPIO pin.
2. **Load Firmware**:
   - Open the project in STM32CubeIDE.
   - Compile and flash the code onto the STM32 microcontroller.
3. **Bluetooth Configuration**:
   - Pair the Bluetooth module with your device.
   - Use a terminal app to send UART commands.
4. **Operate the Car**:
   - Use the command set to control the car.
   - Monitor behavior, including obstacle detection and sound effects.

## Project Directory Structure
```
project-root/
├── Core/
│   ├── Inc/
│   │   └── main.h
│   ├── Src/
│       └── main.c
├── Drivers/
│   └── STM32F4xx_HAL_Driver/
├── README.md
├── Makefile
└── .project
```

## Future Improvements
- Implement autonomous navigation using sensors and predefined paths.
- Enhance the sound module with dynamic melody generation.
- Add more advanced communication protocols (e.g., Wi-Fi or ZigBee).

## Acknowledgments
This project was developed as part of a course final evaluation, combining embedded systems concepts and hands-on hardware integration.
