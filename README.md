# 🛸 Autonomous UAV Friend or Foe (IFF) System

A Secure, Decentralized, and Patented Identification System for UAV Swarms.

> **📱 [Click here to view all project details](https://autonomousiffsystem.streamlit.app/)** - Interactive dashboard with real-time demonstrations

## 🎯 Project Overview

Managing secure airspaces has become increasingly complex with the rapid rise of UAVs. Traditional Identification Friend or Foe (IFF) systems are often too heavy, power-hungry, or costly for small-scale tactical drones.

This project delivers an indigenous, low-cost UAV platform featuring:

- **BYCAST-IFF Protocol**: A patented bidirectional authentication system designed to solve "Authentication Deadlocks" in decentralized networks.
- **Indigenous Flight Controller (FC)**: A 32-bit ESP32-centric architecture that eliminates proprietary "black-box" dependencies.
- **GPS-Guided Payload Logic**: A Raspberry Pi-controlled mechanism for precision logistics and disaster relief.

## ✨ Key Performance Metrics

| Metric | Achieved Result | Industry Target |
|--------|-----------------|-----------------|
| Authentication Success Rate | 91.3% (Tactical) | > 85% |
| Payload Delivery Accuracy | < 5 Meters | 5 Meters |
| Authentication Latency | ~847 ms (Avg) | < 1500 ms |
| Deadlock Events | Zero | Minimal |
| Replay Attack Defense | 94.2% Detection | High Security |

## 🏗️ System Architecture

The platform utilizes a Two-Tier Edge Architecture to balance hard-real-time stability with high-level cognitive mission planning.

- **Tier 1 (Real-Time Edge)**: An ESP32 dual-core processor manages the PID stabilization loop (250Hz) and AES-256 encrypted communication.
- **Tier 2 (Cognitive Edge)**: A Raspberry Pi 4B handles GPS distance calculations and mission logic, ensuring heavy processing never interferes with flight stability.

## 🛡️ Patented Technical Innovations

### 1. BYCAST-IFF Protocol (Role Alternation)

In decentralized swarms, "Authentication Deadlocks" occur when two nodes attempt to interrogate each other simultaneously. Our patented solution utilizes a state-machine that automatically swaps Initiator and Responder roles every 15 seconds, ensuring one node is always listening while the other transmits.

### 2. Prime-Number TDMA Slotting

To prevent RF collisions in dense environments, we implemented a deterministic hashing algorithm based on prime number multiplication.

$$T_{offset}(ID_{drone}) = (ID_{drone} \cdot P_{hash}) \mod T_{cycle}$$ 

## 🛠️ Hardware & Tools

| Component | Specification |
|-----------|---------------|
| Microcontroller | ESP32-WROOM-32 (240MHz Dual-Core) |
| Comm. Module | LoRa E32 433T30D (433MHz, 1W) |
| Mission Unit | Raspberry Pi 4B (Python 3.11) |
| Sensors | MPU-6050 (IMU), BMP280 (Barometer), NaVIC GPS |
| Design Tools | EasyEDA (PCB), Arduino IDE (C++), SolidWorks (CAD) |

## 📜 IPR Notice & Patents

The technical logic, algorithms, and source code of this project are protected under Indian Intellectual Property Rights.

- **Patent 1**: A Flight Controller System for Autonomous Hexacopter Applications (Application No: 202511063731 A).
- **Patent 2**: Autonomous Friend or Foe Identification System (Issue No. 29/2025).

---

Developed for Thapar Institute of Engineering & Technology | Capstone Group
