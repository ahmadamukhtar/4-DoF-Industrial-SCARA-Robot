# **N-DoF Teleoperational System for 4-DoF Industrial SCARA Robot**  
*A Modular CAN Bus-Based Control Architecture*  


**Scara Robot Demo Video:** [Video link](https://drive.google.com/file/d/1_Rjsqo9XiC1aGFfsBElCld4F-KgaRdup/view?usp=sharing) 
**Single DC Motor Card:** [Image link](https://drive.google.com/file/d/10862qc_z-TdehtUcaU7PtxMIyGcl1ge_/view?usp=sharing) 
**Single DC Motor Card:** [Image link](https://drive.google.com/file/d/1YYd2-KgDj5CAyKqFfTfsqcmL1sgpU0G8/view?usp=sharing) 


 

## **📌 Overview**  
This project implements a **teleoperated control system** for a **4-DoF SCARA robot** using a **distributed CAN bus architecture**. The system features:  
✅ **Real-time joint & Cartesian control** via a **C# GUI**  
✅ **Modular design** (scalable to N-DoF) with **STM32 master-slave controllers**  
✅ **PID-based motor control** with **encoder feedback** (Omron E6B2-CWZ3E)  
✅ **Digital twin simulation** in **V-REP** for validation  
✅ **Industrial-grade robustness** (CAN bus, trapezoidal velocity profiles, safety limits)  

**Research Paper:** [PDF Link](https://drive.google.com/file/d/1TaON4kpfjoUx2aGAXLBC6Rx7xFMzxIFO/view?usp=sharing) 

---

## **🚀 Key Features**  
| **Feature**               | **Description** |
|---------------------------|----------------|
| **📱 GUI Control** | C# application for joint/tool space control with real-time feedback |
| **🔌 CAN Bus Communication** | STM32F767ZI (Master) + STM32F103C6T6 (Slaves) |
| **🤖 Forward/Inverse Kinematics** | DH-parameter-based MATLAB & embedded C solutions |
| **🎛️ PID Motor Control** | Closed-loop control with encoder feedback |
| **📊 Digital Twin** | V-REP simulation for pre-deployment testing |
| **🛡️ Safety Features** | Joint limits, emergency stop, collision avoidance |

---

## **🛠️ Hardware Setup**  
### **Components**  
- **Robotic Arm**: 4-DoF SCARA (Shoulder, Elbow, Prismatic, Tool Roll)  
- **Controllers**:  
  - **Master**: STM32F767ZI (FreeRTOS)  
  - **Slaves**: STM32F103C6T6 (per-joint control)  
- **Motor Drivers**: IBT-2 (43A peak)  
- **Encoders**: Omron E6B2-CWZ3E (1000 PPR)  
- **Communication**: MCP2515 CAN Bus  
- **Power Supply**: 250W Multi-output (12V/5V/3.3V)  

### **Wiring Diagram**  
![Hardware Setup](assets/wiring_diagram.png) *(Add your diagram here)*  

---

## **💻 Software Setup**  
### **Dependencies**  
- **GUI**: Visual Studio (C#)  
- **Microcontrollers**: STM32CubeIDE (FreeRTOS)  
- **Simulation**: V-REP (CoppeliaSim)  
- **MATLAB**: Workspace visualization & kinematics validation  

### **Installation**  
1. **Flash Firmware**  
   ```bash
   git clone https://github.com/yourusername/scara-can-control.git
   cd scara-can-control/firmware
   open STM32CubeIDE project and flash to master/slave boards
   ```

2. **Run GUI**  
   ```bash
   cd scara-can-control/gui
   open Visual Studio solution and build
   ```

3. **V-REP Simulation**  
   - Import `scara_model.ttm` into V-REP  
   - Run `simulation_script.lua`  

---

## **🎯 Demo & Usage**  
### **1. Joint Space Control**  
![GUI Joint Control](assets/gui_joint.png)  
- Set desired angles for each joint via sliders.  

### **2. Tool Space Control**  
![GUI Cartesian Control](assets/gui_cartesian.png)  
- Input (x,y,z) coordinates → Inverse kinematics computes joint angles.  

### **3. CAN Bus Monitoring**  
```c
// Example CAN message (Master → Slave)
typedef struct {
  uint16_t joint_id;
  float target_angle;
  uint8_t control_mode;  // 0=position, 1=velocity
} CAN_Command;
```

---

## **📈 Performance Metrics**  
| **Parameter**       | **Value**       |
|---------------------|-----------------|
| Positioning Accuracy | ±0.1mm         |
| Max Payload         | 2kg            |
| Communication Rate  | 1Mbps (CAN)    |
| Control Frequency   | 1kHz (PID loop)|

---

## **🔮 Future Work**  
- [ ] **Machine Vision Integration** (OpenCV/ROS)  
- [ ] **Haptic Feedback** for force reflection  
- [ ] **EtherCAT Upgrade** for lower latency  
- [ ] **Autonomous Pick-and-Place** with path planning  

---

## **📜 License**  
MIT License - See [LICENSE](LICENSE) for details.  

---

## **🙌 Credits & Citation**  
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
[Ahmad Mukhtar](mailto:ahmadamukhtar860@gmail.com) | [Sardar Muhammad Ali](mailto:alisardar0211@gmail.com)  
**Affiliation:** Pakistan Institute of Engineering and Applied Sciences (PIEAS)  

---

**🌟 Star this repo if you found it useful!**  
**🐛 Report issues [here](#)**  

*(Replace placeholder links/GIFs with your actual content. Customize further based on your project’s specifics!)*  

