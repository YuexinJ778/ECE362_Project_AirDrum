# 🥁 AirDrum (ECE362 Project)

AirDrum is a gesture-controlled embedded percussion system that allows users to play drums in the air.  
An IMU-enabled stick detects motion in real time and triggers corresponding drum sounds through a speaker, while a VGA display provides live visual feedback.

---

## 📌 Project Overview

AirDrum integrates multiple hardware peripherals on the RP2350 platform:

- **I²C** for IMU communication  
- **PWM** for audio waveform generation  
- **VGA** for graphical drum kit display  
- **ADC** for analog volume control  

The system converts motion data into synchronized audio and visual output in real time.

---

## ⚙️ How It Works

1. The user swings the stick.
2. The IMU (LSM6DSOX) sends accelerometer and gyroscope data to the RP2350 via I²C.
3. The firmware analyzes motion strength and direction to determine which drum is hit.
4. The RP2350 generates the corresponding drum sound using timer-driven PWM.
5. The VGA display highlights the triggered drum icon.
6. A potentiometer connected to the ADC adjusts the output volume smoothly.

---


---

## 🔌 Hardware Components

- RP2350 Proton Board (Main Controller)
- LSM6DSOX IMU Module (Accelerometer + Gyroscope)
- VGA Circuit + Monitor
- Speaker
- Potentiometer (Volume Control)
- Breadboard and basic passive components

---

## 🎛️ Key Features

- Real-time motion sensing
- Deterministic PWM-based audio generation
- VGA drum kit UI with visual hit feedback
- Smooth analog volume adjustment
- Multi-peripheral embedded system integration

---

## 🛠️ Implementation Highlights

- Implemented I²C communication for continuous IMU data polling
- Designed threshold-based hit detection using motion magnitude and direction
- Generated distinct drum sounds (snare, toms, hi-hats) using PWM timers
- Integrated VGA rendering with synchronized audio events
- Performed structured debugging for hardware–software interaction

---

## 🖥️ Schematics & PCB

The project includes:

- Mini RP2350 schematics  
- PWM + equalizer circuit  
- VGA + 7-segment output schematics  
- Custom PCB layout and 3D design  

---

## 👥 Team

ECE362 Project – Team 23  
Zixuan Fei  
Yuexin Jiang  
Yu-Hsien Liu  
Boyang Wu  

---

## 🎥 Demo
video demo: https://youtu.be/benzvqpY0oE?feature=shared

Poster:
[AirDrum.pdf](https://github.com/user-attachments/files/25248640/AirDrum.pdf)


