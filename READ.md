# 🌊 Subsea Pipeline Inspection ROV: Design, Simulation & Embedded Control

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/docs/harmonic)
[![Docker](https://img.shields.io/badge/Docker-Containerized-2496ED.svg)](https://www.docker.com/)
[![micro-ROS](https://img.shields.io/badge/micro--ROS-ESP32-brightgreen.svg)](https://micro.ros.org/)
[![Hardware](https://img.shields.io/badge/Hardware-Raspberry%20Pi%204%20%7C%20ESP32-red.svg)]()
[![License](https://img.shields.io/badge/License-MIT-green.svg)]()

---

## 📖 Description

An end-to-end mechatronic design, physics simulation, and embedded control framework for an underwater **Remotely Operated Vehicle (ROV)** dedicated to offshore pipeline crack and structural defect inspection.

<!-- 📸 PASTE SCREENSHOT HERE: (Figure 3.1 / Page 41) - Simulation de l'environnement sous-marin avec Gazebo -->

<!-- 📸 PASTE SCREENSHOT HERE: (Figure 2.4 / Page 29) - Vue isométrique et vue de dessus du ROV -->

This project delivers an integrated solution spanning:
1. **Mechanical & Hydrodynamic CAD:** 3D-modeled frame with a 6-thruster configuration (4 vectored at 45° for planar translation & yaw, 2 vertical thrusters).
2. **Physics & Sensor Simulation:** Full hydrodynamic simulation in **Gazebo Harmonic** bridged with **ROS 2 Humble**, including active Sonar (`gpu_ray`), Forward Camera, and USBL acoustic positioning (`NavSat`).
3. **Acoustic Signal Processing:** Analytical modeling and detection thresholding of sonar pings across pipeline states in MATLAB.
4. **Embedded Control & DevOps:** Deployment of a portable **Docker** containerized ROS 2 stack on a **Raspberry Pi 4**, interfaced via **micro-ROS** over UART with an **ESP32** driving 3× L298N H-bridges.

### 1. Hardware & Electronics Architecture
The electronics bay integrates power management and logic control across a Raspberry Pi 4, ESP32, and 3 dual H-bridge motor drivers:

<!-- 📸 PASTE SCREENSHOT HERE: (Figure 2.9 / Page 40) - Schéma électronique du prototype (Raspberry Pi, ESP32, L298N) -->

<!-- 📸 PASTE SCREENSHOT HERE: (Figure 2.6 / Page 31) - Vue de dessus de la base inférieure et implantation des composants électroniques -->

### 2. Thruster Configuration (Vectored 45°)
The 4 horizontal thrusters are oriented at 45° to provide full planar translation ($X, Y$) and yaw rotation without requiring dedicated lateral thrusters:

| Movement | Thruster Front-Left | Thruster Front-Right | Thruster Back-Left | Thruster Back-Right |
| :--- | :---: | :---: | :---: | :---: |
| **Forward (Surge +)** | ⊖ | ⊖ | ⊕ | ⊕ |
| **Reverse (Surge -)** | ⊕ | ⊕ | ⊖ | ⊖ |
| **Yaw Left (Turn -)** | ⊖ | ⊕ | ⊕ | ⊖ |
| **Yaw Right (Turn +)** | ⊕ | ⊖ | ⊖ | ⊕ |
| **Depth Up / Down** | \- | \- | \- | \- | *(Controlled via 2 vertical thrusters)* |

<!-- 📸 PASTE SCREENSHOT HERE: (Figure 2.5 / Page 30) - Vues isométriques de la base supérieure du ROV -->

<!-- 📸 PASTE SCREENSHOT HERE: (Figure 2.7 / Page 33) - Modélisation 3D d'un propulseur et hélice tripale -->

### 3. Sonar Acoustic Modeling (MATLAB)
To validate acoustic defect detection, simulated active sonar signals ($f_0 = 2\text{ kHz}$) were modeled under three pipeline conditions:

<!-- 📸 PASTE SCREENSHOT HERE: (Figure 2.8 / Page 37) - Simulation MATLAB des signaux sonar reçus selon l'état du pipeline -->

* **Healthy Pipeline:** High specular reflection ($A_{\text{max}} > 0.5$, $\text{SNR} > 12\text{ dB}$).
* **Corroded Pipeline:** Diffuse acoustic scattering ($0.2 < A_{\text{max}} \le 0.5$, $\text{SNR} = 5\text{--}12\text{ dB}$).
* **Critical Breach / Crack:** Complete wave dispersion/absorption ($A_{\text{max}} \le 0.2$, $\text{SNR} < 5\text{ dB}$).

### 4. Technical Implementation Details
* **Gazebo Harmonic & Sensor Plugins (`bluerov2_gz`):**
  * **Buoyancy Plugin:** Configured `gz-sim-buoyancy-system` with hydrodynamic density parameters for realistic submerged neutral buoyancy.
  * **Camera Plugin:** Configured $800 \times 600$ @ 30 FPS optical sensor ($80^\circ$ FOV) published on `/camera`.
  * **Sonar Plugin:** Implemented `gpu_ray` scanner ($256 \times 100$ beams, $10\text{ Hz}$, $0.1\text{--}10\text{ m}$ range) publishing point clouds on `/sonar_sensor/points`.
  * **USBL Positioning:** Modeled acoustic positioning with Gazebo `navsat` publishing coordinates on `/bluerov2/usbl/position`.
  * **ROS-GZ Bridge:** Configured YAML translation matrix bridging topics bidirectionally between Gazebo Harmonic and ROS 2 Humble.
* **Embedded Control & micro-ROS Deployment (`ESP32`):**
  * **Communication Architecture:** Interfaced Raspberry Pi 4 (micro-ROS Agent) and ESP32 via serial UART.
  * **Motor Driver Firmware:** C++/Arduino firmware implementing `drive_motor()` for 8-bit PWM speed control (1 kHz LEDC) and direction toggling across 3× L298N drivers.

<!-- 📸 PASTE SCREENSHOT HERE: (Figure 3.12 / Page 51) - Architecture de communication entre ESP32 et Raspberry Pi -->

<!-- 📸 PASTE SCREENSHOT HERE: (Figure 3.15 / Page 54) - Test et validation de communication ROS2 et micro-ROS -->

<!-- 📸 PASTE SCREENSHOT HERE: (Figure 3.16 / Page 55) - Test des moteurs avec ESP32 (Banc de test physique) -->

---

## 🎯 Motivation

Subsea oil and gas pipelines represent critical global energy infrastructure, yet they are subjected to extreme hydrostatic pressures, corrosive marine environments, and seabed shifts. Traditional inspection methodologies present severe limitations:
* **Smart PIGs (Pipeline Inspection Gauges):** Costly to deploy, require dedicated launch/receive traps, and cannot inspect external walls or out-of-service pipelines.
* **Human Divers:** Strictly limited to shallow waters (< 50 m) with severe safety risks and heavy logistical constraints.
* **Optical Inspection Alone:** Severely degraded in turbid underwater conditions and low-light environments.

This project addresses these challenges by developing a cost-effective, modular inspection ROV equipped with active acoustic sonar modeling, real-time video feedback, and georeferenced defect tracking to enable safe, reproducible, and non-destructive external pipeline integrity assessments.

---

## ⚡ Quick Start

### 1. Run via Docker
```bash
git clone https://github.com/sirinesioud-hash/ROV_Conception_and_simulation.git
cd ROV_Conception_and_simulation

# Build Docker Image
docker build -t rov_inspect:latest -f docker/Dockerfile .

# Launch Container with GUI / GPU support
docker run -it --rm \
    --net=host \
    --ipc=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    rov_inspect:latest
    ## 🕹️ Usage

### 1. Teleoperation & Thruster Controls
Control the ROV directly from your keyboard using the `keyboard_controller_node`:
* **Directional Keys (`↑`, `↓`, `←`, `→`):** Forward, backward, and yaw left/right rotation.
* **`Z` / `S` Keys:** Vertical depth control (ascend / descend via vertical thrusters).
* **`Space`:** Emergency stop (immediately sets all thrusters to 0).
* **`B`:** Trigger manual inspection snapshot on `/take_snapshot`.

### 2. Real-time Multi-sensor Visualization (RViz2)
Live camera feeds and 3D Sonar point clouds (`PointCloud2`) are displayed simultaneously to assist the operator during inspection missions:

<!-- 📸 PASTE SCREENSHOT HERE: (Figure 3.9 / Page 48) - RVIZ2 (Flux caméra et nuage de points sonar) -->

### 3. Defect Detection & Georeferenced Logging
When a defect is spotted, the `inspection_node` captures synchronized telemetry (USBL GPS coordinates, camera frame, and sonar point cloud) into a structured inspection report:

<!-- 📸 PASTE SCREENSHOT HERE: (Figure 3.10 / Page 49) - Visualisation du rapport, simulation du pipeline endommagé et nuage de points -->

### 4. ROS 2 Computational Graph
<!-- 📸 PASTE SCREENSHOT HERE: (Figure 3.8 / Page 47) - RQT_Graphe -->
## 🤝 Contributing

Contributions, bug reports, and feature proposals are welcome! Please follow these standard steps:
1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/NewFeature`)
3. Commit your Changes (`git commit -m 'Add NewFeature'`)
4. Push to the Branch (`git push origin feature/NewFeature`)
5. Open a Pull Request

### 👥 Authors & Academic Context
Developed as a 2nd-Year Engineering Project in **Mechatronics Engineering** at the **National Engineering School of Carthage (ENICarthage)**.

* **Sirine SIOUD** — *Electronic Architecture, ROS 2 & Gazebo Simulation, micro-ROS Embedded Deployment & Unit Testing*
* **Moncef Shawqi BEN DHIAB** — *Mechanical Design (SolidWorks CAD), Additive Manufacturing & MATLAB Sonar Analysis*
* **Academic Supervisor:** *Chekib GHORBEL*

### 📜 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.