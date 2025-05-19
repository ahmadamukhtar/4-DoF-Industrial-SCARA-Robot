# **N-DoF Teleoperational System for a 4-DoF Industrial SCARA Robot**

*A Modular CAN Bus-Based Control Architecture*

🎬 **SCARA Robot Demo Video:**
[Watch on Google Drive](https://drive.google.com/file/d/1_Rjsqo9XiC1aGFfsBElCld4F-KgaRdup/view?usp=sharing)

---

## 📌 **Overview**

This project implements a **teleoperated control system** for a **4-DoF SCARA robot** using a **distributed CAN bus architecture**. Key features include:

✅ Real-time joint and Cartesian control via a **C# GUI**
✅ Modular design (scalable to N-DoF) with **STM32 master-slave controllers**
✅ PID-based motor control with encoder feedback (Omron E6B2-CWZ3E)
✅ Digital twin simulation using **V-REP**
✅ Industrial-grade robustness with CAN bus, trapezoidal velocity profiles, and safety limits

📄 **Research Paper:** [View PDF](https://drive.google.com/file/d/1TaON4kpfjoUx2aGAXLBC6Rx7xFMzxIFO/view?usp=sharing)

---

## 🧠 **System Design Diagrams**

### 🔹 Single Joint Control Design

![Single Joint Control](https://github.com/user-attachments/assets/092f27b8-41de-4caa-8427-3c36ecd12dc9)

### 🔹 Overall System Architecture

![System Architecture](https://github.com/user-attachments/assets/8f77a8c7-51a6-4bb6-a9e0-299f9dd24c34)

---

## 🚀 **Key Features**

| Feature                  | Description                                                         |
| ------------------------ | ------------------------------------------------------------------- |
| 📱 GUI Control           | C# application for joint/tool space control with real-time feedback |
| 🔌 CAN Bus Communication | STM32F767ZI (Master) + STM32F103C6T6 (Slaves)                       |
| 🤖 Kinematics            | DH-based forward/inverse kinematics in MATLAB and Embedded C        |
| 🎛️ PID Motor Control    | Closed-loop control with encoder feedback                           |
| 📊 Digital Twin          | V-REP simulation for testing and validation                         |
| 🛡️ Safety Features      | Joint limits, emergency stop, and collision avoidance               |

---

## 🛠️ **Hardware Setup**

### **Key Components:**

* **Robotic Arm:** 4-DoF SCARA (Shoulder, Elbow, Prismatic, Tool Roll)

* **Controllers:**

  * **Master:** STM32F767ZI (FreeRTOS)
  * **Slaves:** STM32F103C6T6 (one per joint)

* **Motor Drivers:** IBT-2 (43A peak)

* **Encoders:** Omron E6B2-CWZ3E (1000 PPR)

* **Communication:** MCP2515 CAN Bus

* **Power Supply:** 250W multi-output (12V / 5V / 3.3V)

### **Additional Hardware Details:**

* **Motor Driver (IBT-2):** Utilized for controlling the motors in the SCARA robot, capable of handling peak currents up to 43A for efficient operation.
* **Encoder (Omron E6B2-CWZ3E):** Provides precise feedback with a resolution of 1000 pulses per revolution (PPR), crucial for accurate motor control.
* **Master Controller (STM32F767ZI):** Central control unit running **FreeRTOS**, which coordinates the actions of all connected slave controllers.
* **Slave Controllers (STM32F103C6T6):** Dedicated microcontrollers for controlling individual joints of the SCARA arm.

### **Hardware Images:**

#### 🔸 Single DC Motor Control Card

![DC Motor Card](https://github.com/user-attachments/assets/e4365a5b-704c-4e05-b07a-346041688872)

#### 🔸 Slave Controller Stack (3-DoF)

![Slave Controller Stack](https://github.com/user-attachments/assets/a199cb1e-f04a-4eff-b90a-0f6c1b4205f9)

---

## 💻 **Software Setup**

### **Dependencies:**

* **GUI:** Visual Studio (C#)
* **Firmware:** STM32CubeIDE (FreeRTOS)
* **Simulation:** V-REP (CoppeliaSim)
* **Kinematics & Visualization:** MATLAB

### **Installation Steps:**

1. **Flash Firmware to Boards**

   ```bash
   git clone https://github.com/yourusername/scara-can-control.git
   cd scara-can-control/firmware
   ```

   Open the STM32CubeIDE project and flash to Master and Slave boards.

2. **Run GUI**

   ```bash
   cd scara-can-control/gui
   ```

   Open the Visual Studio solution and build the GUI.

3. **Simulation in V-REP**

   * Import `scara_model.ttm` into V-REP
   * Run the simulation script `simulation_script.lua`

---

## 🎯 **Demo & Usage**

### 🔹 Joint Space Control

![GUI Joint Control](assets/gui_joint.png)
Adjust joint angles via sliders with real-time feedback.

### 🔹 Tool (Cartesian) Space Control

![GUI Cartesian Control](assets/gui_cartesian.png)
Enter (x, y, z) coordinates – inverse kinematics maps to joint angles.

### 🔹 CAN Bus Command Structure (Embedded C)

```c
typedef struct {
  uint16_t joint_id;
  float target_angle;
  uint8_t control_mode;  // 0 = position, 1 = velocity
} CAN_Command;
```

---

## 📈 **Performance Metrics**

| Parameter            | Value        |
| -------------------- | ------------ |
| Positioning Accuracy | ±0.1 mm      |
| Max Payload          | 2 kg         |
| Communication Rate   | 1 Mbps (CAN) |
| Control Frequency    | 1 kHz (PID)  |

---

## 🔮 **Future Work**

* [ ] Machine vision integration (OpenCV / ROS)
* [ ] Haptic feedback for force reflection
* [ ] Upgrade to EtherCAT for lower latency
* [ ] Autonomous pick-and-place with path planning

---

## 📜 **License**

This project is licensed under the **MIT License**.
See [LICENSE](LICENSE) for more details.

---

## 🙌 **Credits & Citation**

```bibtex
@article{mukhtar2024scara,
  title={N-DoF Teleoperational System for 4-DoF Industrial SCARA Robot},
  author={Mukhtar, Ahmad and Ali, Sardar Muhammad},
  journal={GitHub Repository},
  year={2024},
  url={https://github.com/yourusername/scara-can-control}
}
```

**Developed by:**
[Ahmad Mukhtar](mailto:ahmadamukhtar860@gmail.com)
[Sardar Muhammad Ali](mailto:alisardar0211@gmail.com)
**Affiliation:** Pakistan Institute of Engineering and Applied Sciences (PIEAS)

---

This updated **README** now effectively integrates your hardware details and offers a streamlined, high-level overview that still provides all the critical information in an accessible format.
