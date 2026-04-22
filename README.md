# Low-Power Wireless Pulse Oximeter

## Overview
This repository contains the software libraries and driver implementations for a custom, low-power wireless pulse oximeter. Designed as part of a comprehensive class project, this device integrates a custom power module, custom-designed PCBs, and an RF module to monitor and wirelessly transmit patient biometric data.

At the core of this software is the interface between a **TI MSP430FR2476** microcontroller and the **SparkFun MAX32664 Biometric Sensor Hub** (which utilizes the **MAX30101** Pulse Oximeter and Heart-Rate Sensor).

## Hardware Architecture
Our custom system relies on the following primary hardware components:
*   **Microcontroller:** Texas Instruments MSP430FR2476 (chosen for ultra-low power capabilities).
*   **Biometric Sensor:** SparkFun MAX32664 Sensor Hub paired with the MAX30101 optical sensor.
*   **Display:** HT1621 LCD Controller for on-device visual feedback.
*   **Radio Module:** CC1101 Sub-1 GHz RF Transceiver for wireless data transmission to a base station.
*   **Custom Hardware:** Team-designed PCBs and a custom power delivery module.

## Features
*   **Robust I2C Interface:** Custom implementation of I2C read/write sequences with an automatic bus-recovery mechanism to prevent system hangs during edge-case communication errors.
*   **Biometric Processing:** Native extraction of heart rate (BPM), blood oxygen levels (SpO2), and algorithmic confidence levels via the MAX32664 sensor hub.
*   **Low-Power Operation:** Makes heavy use of the MSP430's Low-Power Modes (LPM3) alongside timer interrupts to minimize active duty cycles.
*   **Unified Module Control:** Codebase ties together the HT1621 display, CC1101 radio, and MAX biometric sensors into a single, cohesive main loop.

## Repository Structure
*   `sensor-library.h` / `sensor-library.cpp`: The core C++ driver for the MAX30101 and MAX32664. Handles initialization, I2C register configuration, application/bootloader mode switching, and data polling.
*   *Dependencies:* 
    *   `lcd_library.h`: Handles the interface with the HT1621 display.
    *   `rf_library.h`: Manages packet transmission and reception via the CC1101 radio module.

## Usage Example
Below is a high-level look at how the sensor hub is initialized and read within the main application loop:

```cpp
#include "sensor-library.h"

// Initialize the Bio Sensor Hub with the assigned Reset and MFIO pins
SparkFun_Bio_Sensor_Hub bioHub(&P1DIR, &P1OUT, RST, 
                               &P1DIR, &P1OUT, &P1REN, MFIO, 
                               BIO_ADDRESS);

void setup() {
    // Begin communication with the sensor
    uint8_t result = bioHub.begin();
    
    if (result == SFE_BIO_SUCCESS) {
        // Configure the sensor to capture both Sensor and BPM data
        bioHub.configSensorBpm(MODE_ONE);
    }
}

void loop() {
    bioData body;
    
    // Poll the sensor for the latest biometric data
    body = bioHub.readSensorBpm();
    
    if (body.heartRate != 0 && body.oxygen != 0) {
        // Data is valid!
        // 1. Display on the HT1621 LCD
        display_three_digits(OXYGEN, body.oxygen);
        display_three_digits(HEART_RATE, body.heartRate);
        
        // 2. Transmit via the CC1101 Radio Module
        radio.sendPacket(body.oxygen, body.heartRate);
    }
    
    // Enter low-power delay
    delay_ms(2000); 
}
```

## Development Environment
*   **IDE:** Code Composer Studio (CCS) v12+
*   **Compiler:** TI MSP430 Compiler

## Acknowledgments
This library relies on hardware data-sheets and architecture guidelines provided by Texas Instruments, Maxim Integrated, and SparkFun Electronics. The MAX32664 implementation is heavily based on the SparkFun Bio Sensor Hub Library.