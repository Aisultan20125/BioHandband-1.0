# 🛠️ Bio-Headband 1.0: Hardware Architecture & BOM

This directory contains the physical architecture specifications, component pinouts, and visual documentation of the Bio-Headband 1.0 prototype assembly.

## 📦 Bill of Materials (BOM) & Pinout Configuration
The hardware layer is engineered around the ATmega328P microcontroller, integrating multiple biometric sensors for synchronous data acquisition.

| Component | Specification / Model | Arduino Nano Pin Interface |
| :--- | :--- | :--- |
| **Microcontroller** | Arduino Nano (16 MHz, ATmega328P) | Core Processing Unit |
| **EEG Amplifier** | Single-channel EEG (Bitronics Lab) | `Analog A0` |
| **Kinematic Sensor**| 6-axis IMU MPU6050 | `I2C (SDA/SCL)` |
| **PPG Sensor** | Optical Pulse Sensor (Bitronics Lab) | `Analog A1` |
| **Actuator** | Passive Piezo Buzzer | `Digital D9` |

## 🔌 Circuit Assembly
*(Ensure to upload `circuit_assembly.png` to this folder showing the breadboard wiring from the presentation).*

**Power Management:** The system operates via a step-down voltage regulator ensuring stable 5V logic for all connected biometric modules, minimizing signal noise critical for accurate EEG readings.
