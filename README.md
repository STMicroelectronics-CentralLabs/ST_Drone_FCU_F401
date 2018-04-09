# ST_Drone_FCU_F401
Toy drone project for STEVAL-FCU001V1 flight controller unit demo board.
Official documentation of the evaluation board (and link to Online Distributors to buy it) can be found in STMicroelectronics website:
http://www.st.com/content/st_com/en/products/evaluation-tools/solution-evaluation-tools/sensor-solution-eval-boards/steval-fcu001v1.html

ST Drone Zone community and forum:
https://community.st.com/community/drone-zone

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

Latest Official version with BLE remocon Android app (170318)
-------------------------------------------------------------
New official Release with BLE remocon (new Android app available).

Major new features:
- Drone controlled by BLE app
- In case BLE connectivity is used, Sensor calibration procedure and Arming/Disarming commands sent by the app
- Disarming and motor off in case of BLE connectivity lost
- Datalog of FCU data sent by BLE (Motion MEMS sensor raw data, Pressure sensor raw data, Magnetometer sensor raw data, Battery level, RSSI level FCU side, Arming status)
- LED1 and LED2 ON when sensor calibration done and motor armed

Future implementations in plan:
- USB Virtual COM not yet implemented
- Sensor calibration must be performed at each power on of the FCU board. In future implementation the sensor calibration offset value should be stored in Flash
- Pressure sensor and Magnetometer (for e-Compass) data can be read in actual release, but they are not used in the stabilization algorithm of the drone. In future implementations altitude control and RTH (Return To Home) features may be added.


First Official version (221117)
--------------------------------
Known bugs with current release and future implementations in plan:
- If RF connection is lost DC motors are still rotating according to last value sent by Remocon. To insert a timeout to switch off the motors in case of RF connection lost.
- BLE communication with external app (at first for Android) under debugging.
- USB Virtual COM not yet implemented
- Sensor calibration must be performed at each power on of the FCU board. In future implementation the sensor calibration offset value should be stored in Flash.
- Pressure sensor and Magnetometer (for e-Compass) data can be read in actual release, but they are not used in the stabilization algorithm of the drone. In future implementations altitude control and RTH (Return To Home) features may be added.



Beta version with BLE remocon feature (301117)
(app for Android devices - OLD version, new one already released)
-----------------------------------------------------------------
New features:
- BLE remocon or external remocon selectable by #define in rc.h
- with BLE remocon motors switched OFF if connection is lost
- Solved issue of drift at liftoff by changing to correct PID parameters for minidrone frame (modified 18th Dec 2017)

Known bugs or features to be added in future releases:
- BLE connectivity still under debugging, not yet extensively tested
- Android app not yet extensively tested
- Data from FCU sensors to Android app not yet implemented
- Information of motors armed to Android app not yet implemented
- Only 4 channels (4 joystick channels for flight control, THR, AIL, ELE, RUD) sent via BLE managed by FCU. Other channels to be added.

