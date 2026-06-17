<h1 align="center"> Autonomous PID Wall-Following Rover</h1>

<div align="center">
  <strong>An advanced closed loop control system built for precision navigation and dynamic obstacle avoidance.</strong><br>
  Engineered by <strong>Vivan J</strong> & <strong>Brendan R</strong>
</div>

<br>

## 📋 Project Overview
This repository contains the C++ firmware and control logic for a custom differential drive robotic rover. Moving beyond basic delay-based navigation, this project implements a **Proportional Integral Derivative (PID) controller** to maintain a mathematically precise 15cm distance from a parallel wall. 

The architecture features a **priority-based hardware interrupt system**. A secondary forward-facing ultrasonic sensor constantly monitors the path ahead; if an obstacle is detected within 20cm, the PID loop is suspended, and the rover executes a rapid differential point turn to clear the path before resuming autonomous wall-following.

---

## ⚙️ Hardware Architecture
The physical build is optimized for high-torque delivery and logic stability.

| Component | Specification | Function in System |
| :--- | :--- | :--- |
| **Microcontroller** | Arduino Nano (ATmega328P) | Brains of the operation; computes PID math at 50Hz. |
| **Motor Driver** | L298N Dual H-Bridge | Handles high-current switching and differential steering. |
| **Sensors** | 2x HC-SR04 Ultrasonic | Spatial awareness (Front: Collision, Left: PID Error tracking). |
| **Power Plant** | 7.4V Li-Ion (2x 18650) | High-discharge series pack for maximum gear motor torque. |
| **Chassis** | 2WD Acrylic Base | Differential drive configuration with front caster wheel. |

---

## 🔌 Core Wiring & Power Distribution
Proper power distribution is critical in this build. To prevent the Nano from experiencing voltage brownouts during high torque motor spikes, we utilized a specific power routing strategy:

1. **The 7.4V Rail:** The raw Li-ion battery power routes through a physical rocker switch directly into the **12V Terminal** of the L298N motor driver.
2. **Logic Power Injection:** We bypass the Arduino Nano's internal voltage regulator (`Vin`) entirely. The L298N's onboard regulator steps the 7.4V down to a clean 5V, which is fed directly into the Nano's **`5V` Pin**.
3. **Common Ground:** The battery, motor driver, Arduino, and sensors share a unified ground bus to ensure PWM and I/O signals are properly referenced.

### Pin Mapping
| Arduino Pin | Hardware Connection | Purpose |
| :---: | :--- | :--- |
| `D5`, `D6` | L298N `ENA`, `ENB` | PWM Speed Control (0-255) |
| `D3`, `D2` | L298N `IN1`, `IN2` | Left Motor Direction |
| `D7`, `D4` | L298N `IN3`, `IN4` | Right Motor Direction |
| `D9`, `D10` | Left HC-SR04 (`TRIG`, `ECHO`) | Side Wall Distance (PID Input) |
| `D11`, `D12` | Front HC-SR04 (`TRIG`, `ECHO`)| Collision Detection (Interrupt Input)|

---

## 🧠 Control Logic & Engineering Solutions

### 1. The PID Controller
The rover calculates its cross track error (`Current Distance - 15cm Setpoint`) and applies the PID formula to adjust the differential speed of the left and right wheels. 
* **Kp (Proportional):** The primary steering force pushing the rover back to 15cm.
* **Kd (Derivative):** Acts as a damping force, analyzing the rate of approach to prevent the rover from overshooting the target and oscillating.
* **Ki (Integral):** Corrects minor steady state drift over time.

### 2. Failsafe: The "Infinite Spin" Trap
If the rover loses the wall entirely (distance > 80cm), standard PID controllers will latch onto the last extreme error value and spin in circles forever.
* **The Fix:** We implemented a bounding failsafe. If the left sensor returns a timeout or a distance >80cm, the PID integral memory is wiped clean (`integral = 0`), and the rover defaults to a straight drive "Search Mode" until it regains a wall reference.

---

## 🚀 Getting Started

### Prerequisites
* Arduino IDE installed.
* `HCSR04` library by Martin Sosic (available in Arduino Library Manager).

### Installation
1. Clone this repository: `git clone https://github.com/YourUsername/PID_WALL_FOLLOWING_ROVER.git`
2. Open `pid_rover.ino` in the Arduino IDE.
3. Ensure your Arduino Nano is disconnected from the 7.4V battery pack before connecting via USB.
4. Compile and upload to the board.
5. Disconnect USB, flip the battery switch, and place the rover 15cm from a left-hand wall!

---

## 📜 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
