# Autonomous UAV System for Subterranean 3D Mapping & Gas Detection

<p align="center">
  <img src="https://img.shields.io/badge/Platform-MATLAB%20%7C%20Simulink-blue?style=for-the-badge&logo=mathworks" />
  <img src="https://img.shields.io/badge/Drone-Parrot%20Mambo-orange?style=for-the-badge" />
  <img src="https://img.shields.io/badge/Communication-LoRa%20Mesh-green?style=for-the-badge" />
  <img src="https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge" />
</p>

> A fully autonomous drone system designed for **GPS-denied subterranean environments** such as underground mines. It performs **real-time hazardous gas detection** and **3D spatial mapping** using a suite of gas sensors, a TriEye depth sensor, ultrasonic obstacle avoidance, and a self-healing LoRa mesh communication network.

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Sensor Suite](#sensor-suite)
- [Breadcrumb Communication Network](#breadcrumb-communication-network)
- [Repository Structure](#repository-structure)
- [Software & Tools](#software--tools)
- [Getting Started](#getting-started)
- [How It Works](#how-it-works)
- [License](#license)

---

## Overview

Underground mines present extreme challenges — **no GPS**, **hazardous gases**, and **limited communication range**. This project addresses all three by deploying an autonomous UAV that:

1.  **Navigates autonomously** using image processing, ultrasonic sensors, and a Stateflow-based path-planning algorithm — no GPS required.
2.  **Detects hazardous gases** (CO, CO₂, O₂, CH₄) in real-time using MQ-9, NDIR, and AO-O₂ sensors.
3.  **Maps the environment in 3D** using a TriEye short-wave infrared (SWIR) depth sensor.
4.  **Maintains communication** deep underground via a self-healing **LoRa breadcrumb mesh network**.

The entire flight control system is built on **MATLAB/Simulink** using the **Parrot Minidrone Support Package**, with a **Raspberry Pi** acting as the onboard sensor hub.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                        AUTONOMOUS UAV                                │
│                                                                      │
│  ┌──────────────┐   ┌──────────────────┐   ┌─────────────────────┐  │
│  │ Parrot Mambo  │   │  Raspberry Pi    │   │  Breadcrumb Module  │  │
│  │ (Flight       │◄──│  (Sensor Hub)    │──►│  (LoRa Payload)     │  │
│  │  Controller)  │   │                  │   │                     │  │
│  │              │   │  • MQ-9 (CO/CH₄) │   │  • STM32F401CCU6    │  │
│  │  Simulink    │   │  • NDIR (CO₂)    │   │  • SX1278 LoRa      │  │
│  │  Stateflow   │   │  • AO-O₂ (O₂)   │   │  • LiPo Battery     │  │
│  │  Image Proc. │   │  • TriEye (3D)   │   │                     │  │
│  └──────────────┘   └──────────────────┘   └────────┬────────────┘  │
│                                                      │               │
└──────────────────────────────────────────────────────┼───────────────┘
                                                       │ LoRa 433 MHz
                              ┌─────────────────────────┘
                              ▼
                    ┌───────────────────┐      ┌───────────────────┐
                    │ Breadcrumb Node 1 │─────►│ Breadcrumb Node 2 │──► ...
                    │   (Repeater)      │ LoRa │   (Repeater)      │
                    └───────────────────┘      └───────────────────┘
                                                        │
                                                        ▼
                                              ┌───────────────────┐
                                              │   BASE STATION    │
                                              │  (Data Logging &  │
                                              │   Monitoring)     │
                                              └───────────────────┘
```

---

## Sensor Suite

| Sensor | Interface | Parameter Measured | Threshold / Purpose |
|---|---|---|---|
| **MQ-9** | SPI (via MCP3008 ADC) | CO & CH₄ concentration | CO > 200 ppm → Alert |
| **NDIR** | UART | CO₂ concentration | CO₂ > 5000 ppm → Alert |
| **AO-O₂** | I²C | O₂ percentage | O₂ < 19.5% → Alert |
| **TriEye (SWIR)** | I²C | Depth / 3D point cloud | 3D Map generation |
| **Ultrasonic** | GPIO | Distance to obstacles | Obstacle avoidance |
| **IMU** | — | Acceleration & Gyroscope | Flight stabilization |

---

## Breadcrumb Communication Network

The drone operates in environments where **Wi-Fi and cellular signals cannot reach**. To maintain a data link to the surface, the system deploys **breadcrumb repeater modules** as payloads during flight:

- Each breadcrumb contains an **STM32F401CCU6 microcontroller**, an **SX1278 LoRa radio module**, and a **LiPo battery**.
- The drone monitors its **signal strength (RSSI in dBm)**. When the signal drops below a set threshold, it **automatically drops a breadcrumb** to extend the mesh.
- Breadcrumb nodes form a **self-healing LoRa mesh network** — if one node fails, data is re-routed through alternate paths.
- Data packets (gas readings + 3D coordinates) **hop from node to node** until they reach the base station.

### Packet Structure

```c
typedef struct {
    uint8_t   source_id;       // Originator or last repeater ID
    uint8_t   destination_id;  // Final destination (base station = 0xFF)
    uint8_t   packet_type;     // Data type identifier
    float     gas_value;       // Gas sensor reading
    float     pos_x;           // 3D map X-coordinate
    float     pos_y;           // 3D map Y-coordinate
    float     pos_z;           // 3D map Z-coordinate
    uint8_t   checksum;        // Data integrity check
} LoRa_Packet;
```

---

## Repository Structure

```
├── Code for Flight Controller.mak         # Simulink-generated Makefile for Parrot Mambo
│                                           # (cross-compiles flight control system to ARM)
│
├── Raspberry pi.py                         # Sensor acquisition script (Raspberry Pi)
│                                           # Reads MQ-9, NDIR, AO-O₂, TriEye sensors
│                                           # and streams data to Simulink via TCP socket
│
├── Raspberry_pi_to_simulink_connection.cpp # Simulink S-Function (MEX)
│                                           # Receives live sensor data from Raspberry Pi
│                                           # Outputs 4 gas values + 4 alert flags
│
├── Raspberry Pi to Breadcrumb Repeater Code.py  # LoRa transmitter code (Raspberry Pi)
│                                                # Packs gas + position data into binary
│                                                # packets and transmits via SX1278 LoRa
│
├── BreadCrumb Repeater Code STM32F401CCU6.c     # Breadcrumb repeater firmware (STM32)
│                                                # Listens → Validates → Forwards packets
│                                                # through the LoRa mesh network
│
├── Team Genesis.zip                        # Complete project archive (all source files,
│                                           # Simulink models, Stateflow charts, etc.)
│
├── LICENSE                                 # MIT License
└── README.md                               # This file
```

---

## Software & Tools

| Tool | Purpose |
|---|---|
| **MATLAB R2024b** | Core development environment |
| **Simulink** | Flight control system design, signal processing |
| **Stateflow** | Path-planning state machine (takeoff, navigate, return, land) |
| **Simulink Support Package for Parrot Minidrones** | Drone hardware interface & code generation |
| **Sourcery G++ Lite (ARM)** | Cross-compilation toolchain for Parrot Mambo |
| **STM32CubeIDE** | Breadcrumb repeater firmware development |
| **Python 3** | Raspberry Pi sensor scripts & LoRa communication |

---

## Getting Started

### Prerequisites

- **MATLAB R2024b** (or compatible) with the following toolboxes/packages:
  - Simulink
  - Stateflow
  - Simulink Support Package for Parrot Minidrones
  - Image Processing Toolbox
- **Sourcery G++ Lite** cross-compiler (ARM, for flight controller deployment)
- **STM32CubeIDE** (for breadcrumb repeater firmware)
- **Python 3** with `spidev`, `smbus`, `pyserial` (for Raspberry Pi scripts)
- **Raspberry Pi** with SPI, I²C, and UART enabled

### Setup

1. **Clone the repository:**
   ```bash
   git clone https://github.com/scarletnova20/Autonomous-UAV-System-for-Subterranean-3D-mapping-and-gas-detection.git
   ```

2. **Extract the project archive:**
   Extract `Team Genesis.zip` to access the complete Simulink models, Stateflow charts, and supporting files.

3. **Open the Simulink model:**
   Open the `flightControlSystem` model in MATLAB/Simulink.

4. **Configure the Parrot support package:**
   Follow the [MathWorks Parrot Minidrone setup guide](https://www.mathworks.com/help/supportpkg/parrot/index.html) to connect your drone.

5. **Deploy sensor scripts to Raspberry Pi:**
   - Copy `Raspberry pi.py` to the Raspberry Pi.
   - Install dependencies: `pip install spidev smbus pyserial`
   - Run: `python3 Raspberry\ pi.py`

6. **Flash breadcrumb firmware:**
   - Open `BreadCrumb Repeater Code STM32F401CCU6.c` in STM32CubeIDE.
   - Configure `REPEATER_ID` and `UPSTREAM_NODE_ID` for each repeater unit.
   - Build and flash to each STM32F401CCU6 board.

---

## How It Works

### 1. Autonomous Navigation (Simulink + Stateflow)
The flight controller runs a **Stateflow state machine** that manages the drone's flight phases:
- **Takeoff** → **Move Forward** → **Obstacle Check** → **Turn Left/Right** → **Return** → **Land**
- Image processing converts the onboard camera feed from **RGB to grayscale**, enabling path-following against a reference image.
- Ultrasonic sensors provide real-time **obstacle detection and avoidance**.

### 2. Gas Detection & Monitoring
The Raspberry Pi continuously polls all gas sensors via SPI, UART, and I²C, then streams the readings to MATLAB/Simulink over a **TCP socket** connection. Inside Simulink, a custom **S-Function** (`gas_monitor_sfun`) parses the data and raises alerts when thresholds are exceeded:

| Gas | Dangerous Level | Action |
|---|---|---|
| CO | > 200 ppm | ALERT |
| CO₂ | > 5000 ppm | ALERT |
| O₂ | < 19.5% | ALERT |
| CH₄ | > 50,000 ppm | ALERT |

### 3. 3D Mapping
The **TriEye SWIR depth sensor** captures distance measurements that are combined with the drone's position data to build a **3D point cloud** of the subterranean environment.

### 4. Breadcrumb Mesh Communication
As the drone moves deeper into the mine, breadcrumb modules are automatically deployed. Each breadcrumb:
1. **Receives** a data packet from the upstream node.
2. **Validates** the packet source and destination.
3. **Stamps** its own ID and **forwards** the packet downstream toward the base station.

---

## License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.
