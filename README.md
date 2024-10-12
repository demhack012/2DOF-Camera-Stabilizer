# 2DOF Camera Stabilizer using LabVIEW

## Project Overview
This project demonstrates a 2 Degrees of Freedom (DOF) camera stabilizer controlled using an **Arduino Nano**, an **IMU6050** sensor, and **2 servo motors**. The control system is implemented in **LabVIEW** with the help of the **LINX Toolkit** for hardware communication.

The stabilizer adjusts the orientation of the camera in two axes, stabilizing it in real-time based on the sensor's input.

## Components Required
- **Arduino Nano**
- **IMU6050** (6-axis accelerometer and gyroscope)
- **2 Servo Motors** (for the two axes of stabilization)
- **LabVIEW** (with LINX Toolkit)
- **Connecting wires and power supply**

## Project Structure
The project consists of two main parts:
1. **Arduino Code**: Reads data from the IMU6050 sensor and sends the data to LabVIEW.
2. **LabVIEW Code**: Processes the sensor data and sends control signals to the servo motors to adjust the camera's orientation.

## Setup Instructions

### Step 1: Hardware Connections
1. Connect the **IMU6050** to the **Arduino Nano** using I2C protocol (SCL to A5, SDA to A4).
2. Connect the two **servo motors** to the digital pins of the Arduino Nano (D5 and D6).
3. Power the Arduino Nano via USB or an external power supply.

### Step 2: Install Required Software
- Download and install **LabVIEW** if not already installed.
- Install the **LINX Toolkit** for LabVIEW to enable communication with the Arduino Nano.
  - You can find the LINX Toolkit in the LabVIEW VI Package Manager.

### Step 3: Arduino Code
1. Open the Arduino IDE and upload the provided Arduino code to your **Arduino Nano**.
2. The code reads sensor data from the IMU6050 and communicates the data over serial to LabVIEW.

### Step 4: LabVIEW Project
1. Open the **LabVIEW Project** included in this repository.
2. Ensure the Arduino Nano is connected to the correct COM port.
3. Run the LabVIEW program, which will read the IMU data and control the servos to stabilize the camera.

## How It Works
- The IMU6050 measures the pitch and roll of the camera.
- These values are sent to LabVIEW via the serial interface.
- LabVIEW processes the data and calculates the appropriate angles for the servo motors.
- The servo motors then adjust the camera’s orientation in real-time to maintain stability.

## Usage
1. Upload the provided **Arduino code** to your **Arduino Nano**.
2. Connect your Arduino Nano to the PC running **LabVIEW**.
3. Open and run the **LabVIEW Project**.
4. The camera will automatically stabilize based on real-time IMU data.

## Future Improvements
- Implementing 3DOF stabilization.

