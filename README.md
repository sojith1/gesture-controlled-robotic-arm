Wireless Gesture Controlled 4-DOF Robotic Arm

📌 Overview

This project enables real-time wireless control of a 4-DOF robotic arm using hand gesture tracking.

The system uses:

- MediaPipe Hand Tracking on PC

- TCP Wireless Communication

- Raspberry Pi GPIO Servo Control

- Direct GPIO-based PWM control (No external driver)

Hand movement is captured on a PC camera and transmitted wirelessly to a Raspberry Pi, which replicates the motion on a 4-DOF robotic arm.


🏗 System Architecture

PC (Gesture Detection)
→ TCP Socket
→ WiFi Network
→ Raspberry Pi
→ GPIO PWM
→ 4 DOF Servo Arm


🔧 Hardware Used

- Raspberry Pi 4B

- 4x Servo Motors

- External 5V Power Supply

- Jumper Wires

- 4-DOF Robotic Arm


🧠 Software Stack

- Python 3.9+

- OpenCV

- MediaPipe

- Socket Programming

- RPi.GPIO


🚀 Setup Instructions

1️⃣ Clone Repository
git clone https://github.com/yourusername/wireless-gesture-robotic-arm.git
cd wireless-gesture-robotic-arm

2️⃣ PC Setup
cd pc_client
pip install -r ../requirements.txt
python gesture_sender.py

3️⃣ Raspberry Pi Setup
cd raspberry_pi_server
pip install -r ../requirements.txt
python arm_receiver.py



📊 Research Contribution

This project demonstrates:

- Low-latency gesture-to-actuator mapping

- Real-time wireless robotic control

- Direct GPIO servo PWM without external driver

- Practical implementation of computer vision in robotics


📈 Future Improvements

- Add Kalman filtering for smoother motion

- Replace GPIO PWM with hardware PWM (pigpio)

- Add inverse kinematics layer

- Add force feedback

- Convert to ROS2 architecture
