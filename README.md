# IMU-Tilt-and-Motion-Detector
Small ESP32 project that uses an MPU-6050 to measure tilt angle and fast movement.

🚀 What This Does

Reads raw accelerometer data

Computes pitch & roll using atan2()

Streams results over Serial

🛠 Requirements

ESP32

MPU6050 module

4 jumper wires

▶️ Usage

Upload the sketch and open Serial Monitor at 115200 baud.

📘 Lessons Learned

How to compute angles using atan2() and normalized acceleration vectors.

Soldering tiny MPU6050 header pins.

ESP32 pin modes and I²C wiring.

Hardware debugging: first MPU6050 unit was faulty – errors can come from anywhere.

Wiring takes longer than coding.
