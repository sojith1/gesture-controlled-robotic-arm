# Gesture Controlled 4DOF Robotic Arm

This project uses MediaPipe pose estimation to control a 4DOF robotic arm wirelessly using Raspberry Pi.

## Features
- Real-time pose detection
- Wireless TCP communication
- 4 servo control via GPIO
- Angle smoothing
- FPS monitoring

## Requirements
- Python 3.12
- MediaPipe 0.10.32
- OpenCV
- Raspberry Pi

## How It Works
Human arm movement → MediaPipe → WiFi → Raspberry Pi → Servo movement
