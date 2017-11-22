# ST_Drone_FCU_F401
Toy drone project for STEVAL-FCU001V1 flight controller unit demo board.
Official documentation of the evaluation board (and link to Online Distributors to buy it) can be found in STMicroelectronics website:
http://www.st.com/content/st_com/en/products/evaluation-tools/solution-evaluation-tools/sensor-solution-eval-boards/steval-fcu001v1.html

The project is intended for developers, makers, universities, research institutes, drone enthusiasts, who are approaching Drone design for the first time and are looking for a project easy to understand and to customize. It's not intended as a commercial FCU and the features implemented cannot be compared to major FCU projects present in the market (Betaflight, Cleanflight, Ardupilot, PX4, ...) sustained by a large commmunity.
The STEVAL-FCU001V1 and relative FW project is also a good reference board to evaluate performance of 6-axis IMU sensors, pressure sensor, magnetometer, BLE module, in real flight condition.
Any contributor to the project (for any aspect, FW project, Smartphone application, original frame design, ...) in the Makers community is welcome.

The FW project has been validated on a mini drone platform with following characteristics:
- 65mm propeller
- 8520 DC motors
- 1S Lipo battery 500-600mAh
- Overall weight below 70g
- External Remocon RX module with PWM input (50Hz, 1-2msec Ton)

Tests have been done also with an external ESC configuration (please checkHW User Manual of the board  documentation for HW modifications needed to the circuit to bypass the DC motor driver Mosfet onboard) and FPV250 frame, but not yet extensively.

Known bugs with current release and future implementations in plan:
- If RF connection is lost DC motors are still rotating according to last value sent by Remocon. To insert a timeout to switch off the motors in case of RF connection lost.
- BLE communication with external app (at first for Android) under debugging.
- USB Virtual COM not yet implemented
- Sensor calibration must be performed at each power on of the FCU board. In future implementation the sensor calibration offset value should be stored in Flash.
- Pressure sensor and Magnetometer (for e-Compass) data can be read in actual release, but they are not used in the stabilization algorithm of the drone. In future implementations altitude control and RTH (Return To Home) features may be added.
