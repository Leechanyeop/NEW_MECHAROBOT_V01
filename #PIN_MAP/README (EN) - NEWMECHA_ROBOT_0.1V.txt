README (EN) - NEWMECHA_ROBOT_0.1V
System Modules
- MOTOR DRIVE – Robot driving and movement
- SENSOR_recog – Sensor recognition
- motion_track – Motion control when obstacles are detected
- CRC_tel – Communication node between server and robot
- vision_CAM – Vision camera for object recognition
- QR_visible – QR code recognition and coordinate calculation

Communication Flow
SERVER → ESP32
- Destination Path Setting
Examples:
- Go from A to B
- Travel from A to B and stop by C along the way
- Arrive at B, pass through D, then return to A
Digital Map Example:

    _______________ B
A --------|---------------- C
    |_______________ D
- ESP32 reads QR values sent from the server and converts them into real-time navigation paths.
- The server receives shared data from the robot, including sensor readings and obstacle detection video.

ESP32 → SERVER
- At each tracing branch, ESP32 recognizes QR codes and sends the QR values to the server.
- When obstacles are detected, ESP32 uses vision and ultrasonic sensors to inspect and transmit real-time video.
- ESP32 sends various sensor data from the robot back to the server.

This document provides a clear overview of the NEWMECHA_ROBOT_0.1V system, its modules, and the communication workflow between the server and ESP32.

