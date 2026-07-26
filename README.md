# 🤖 2WD Differential Robot: Bluetooth & Autonomous Navigation

A robotics project consisting of assembling and programming a 2-Wheel-Drive (2WD) differential car. The project is divided into two main phases: manual teleoperation via Bluetooth, followed by fully autonomous navigation.

## 🎮 Phase 1: Manual Control (Bluetooth)

### ⚡ Electric Scheme

> **Hardware Note:** In the real assembly, we use three 210-ohm resistors in parallel. This ensures a continuous current draw so the power bank registers the consumption and does not shut down automatically.

<img width="551" height="524" alt="Electric Scheme" src="https://github.com/user-attachments/assets/0b02fc35-ef71-4e24-bf30-ce1b9a09d736" />

### 🛠️ Real Assembly

<img width="1012" height="759" alt="Real Assembly Top" src="https://github.com/user-attachments/assets/568e49f3-5aba-4d31-a733-9d8b42f94278" />

<img width="556" height="663" alt="Real Assembly Side" src="https://github.com/user-attachments/assets/1dc87419-bfee-4feb-a7da-03bf31cf41c2" />

### Programming

The programming was made in the C language using CCS 12.7 

This program consist only in a foreground-background pattern where the MSP430 is continually checking if the HC-05 has receive any new info (checking the flag that the interruption sets) , and depending of the commands that the user sends it responds one way or another.
The protocol is rather simple, the user sends a string with the comands and ends it with a special character (default is #)

#### 🎮 Command Map

*   **Independent Motor Control:**
    *   `LF#` / `LB#` : Left Motor Forward / Backward
    *   `RF#` / `RB#` : Right Motor Forward / Backward
    *   `LN#` / `RN#` : Left / Right Motor Stop (Neutral)
    *   `LS#` / `RS#` : Left Brake / Right Brake
*   **Speed Control (PWM):**
    *   `C<value>#` : Adjusts the speed of the motors based on the slider position. F


To facilitate myself the communication I made a simple interface using MIT App inventor 
