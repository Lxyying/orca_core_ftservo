# OrcaHand-Feetech: 基于飞特 ST3215 舵机的 OrcaHand 改进方案

[English](#english) | [中文](#chinese)

<a name="english"></a>
## English Description

### Project Overview
This repository hosts a modified implementation of the [OrcaHand](https://www.orcahand.com/) dexterous manipulator. The primary objective of this project is to optimize hardware costs and simplify the control architecture by transitioning the actuation system from the original Dynamixel servos to **Feetech ST3215-C001** serial bus servos.

This repository includes the Python control drivers adapted for the Feetech communication protocol, along with the necessary STL files for manufacturing.

### Technical Modifications
* **Actuation System**: The original Dynamixel servos have been replaced with Feetech ST3215-C001 serial bus servos.
* **Kinematic Structure**: The wrist degree of freedom (DOF) has been eliminated to enhance structural rigidity. The wrist connection is now a fixed configuration.

### 3D Models and Attribution
The `.stl` files required for assembly are located in the `3DPrint_stl/` directory.
This project strictly adheres to open-source licenses. The attribution for the 3D models provided in this repository is as follows:

| File Name | Description | Source / Attribution |
| :--- | :--- | :--- |
| `ORCA_Fingers-Right.stl`<br>`RatchetConnection.STL`<br>`RatchetCover.STL`<br>`RatchetGear.STL` | **Finger Assembly Parts**<br>Original design files used without modification. | **[OrcaHand Official](https://www.orcahand.com/dashboard)**<br>Credit to the OrcaHand Team. |
| `Tower_ftservo.STL` | **Servo Housing (Tower)**<br>Modified geometry to accommodate Feetech ST3215 servos. | **[Bilibili Creator @404](https://www.bilibili.com/video/BV1xJx8zGEgn)**<br>Based on the modification design shared by this creator. |
| `TopTower-1.STL`<br>`TopTower-2.STL` | **Wrist Connection Interface**<br>Custom modification by the project team.<br> wrist DOF removed for fixed constraint. | **Team Development**<br>Derived from original OrcaHand geometry. |

### Usage Instructions

#### 1. Hardware Configuration
* Connect the Feetech ST3215 servos to the TTL serial interface board.
* Verify the serial port assignment (e.g., `COMx` on Windows or `/dev/ttyUSBx` on Linux).

#### 2. Environment Setup
```bash
# Clone the repository
git clone [https://github.com/Lxyying/orca_core_ftservo.git](https://github.com/Lxyying/orca_core_ftservo.git)

# Install dependencies
pip install -r requirements.txt