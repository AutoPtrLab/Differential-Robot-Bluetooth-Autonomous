# 🤖 2WD Differential Robot: Bluetooth & Autonomous Navigation

A robotics project consisting of assembling and programming a 2-Wheel-Drive (2WD) differential car. The project is divided into two main phases: manual teleoperation via Bluetooth, followed by fully autonomous navigation.

### 🛒 Hardware Components (Bill of Materials)

To replicate this project, you will need the following parts:

*   **Microcontroller:** Texas Instruments MSP430F5529 LaunchPad
*   **Wireless Communication:** HC-05 Bluetooth Module
*   **Motor Driver:** TB6612FNG Dual Motor Driver (More efficient than standard L298N)
*   **Chassis & Actuators:** 2WD Robot Chassis Kit (Includes 2x DC Motors and wheels)
*   **Power Supply (Logic):** Standard USB Power Bank 
    * *(Includes the custom 3x 210Ω parallel resistor circuit to prevent auto-shutdown)*
*   **Power Supply (Motors):** External Battery Pack (to independently power the TB6612FNG and motors)
*   **Prototyping:** Standard Breadboard and jumper wires

---

## 🎮 Phase 1: Manual Control (Bluetooth)

### 🎥 Robot in Action

<img width="278" height="450" alt="robot" src="https://github.com/user-attachments/assets/a1d4474d-0b57-4111-bd42-6c51fad623fa" />

**Want to hear the motors and see the full test runs?** 

[![Watch Short 1](https://img.shields.io/badge/YouTube-Test_Run_1-red?style=for-the-badge&logo=youtube)](https://youtube.com/shorts/GyfELeh25M4?si=cqAGC58dihnduHg8)
[![Watch Short 2](https://img.shields.io/badge/YouTube-Test_Run_2-red?style=for-the-badge&logo=youtube)](https://youtube.com/shorts/uWUYhGvj8_0?si=G7VaVpL7Wf5WP6yT)

### 📲 Android App Download

To facilitate communication, I developed a simple control interface using MIT App Inventor. You can download the compiled Android application (.apk) directly from the Releases section:

[![Download APK](https://img.shields.io/badge/Download-Android_APK-green?style=for-the-badge&logo=android)](https://github.com/AutoPtrLab/Differential-Robot-Bluetooth-Autonomous/releases/tag/Bluetooth_Controller)

### ⚡ Electric Scheme

> **Hardware Note:** In the real assembly, we use three 210-ohm resistors in parallel. This ensures a continuous current draw so the power bank registers the consumption and does not shut down automatically.

<img width="551" height="524" alt="Electric Scheme" src="https://github.com/user-attachments/assets/0b02fc35-ef71-4e24-bf30-ce1b9a09d736" />

### 🛠️ Real Assembly

<img width="512" height="350" alt="Real Assembly Top" src="https://github.com/user-attachments/assets/568e49f3-5aba-4d31-a733-9d8b42f94278" />

<img width="240" height="363" alt="Real Assembly Side" src="https://github.com/user-attachments/assets/1dc87419-bfee-4feb-a7da-03bf31cf41c2" />

### 💻 Programming

The programming was done in C using Code Composer Studio (CCS) 12.7.

The software architecture follows a foreground-background pattern. The MSP430 continuously checks if the HC-05 has received new data (via an interrupt flag) and executes actions based on the user's commands. 
The communication protocol is straightforward: the app sends a string of commands terminated by a special character (`#` by default).

#### 🎮 Command Map

**Independent Motor Control:**
*   `LF#` / `LB#` : Left Motor Forward / Backward
*   `RF#` / `RB#` : Right Motor Forward / Backward
*   `LN#` / `RN#` : Left / Right Motor Stop (Neutral)
*   `LS#` / `RS#` : Left Brake / Right Brake

**Speed Control (PWM):**
*   `C<value>#` : Adjusts the speed of the motors based on the slider position. For example, `C75#` sets the PWM duty cycle to 75.
