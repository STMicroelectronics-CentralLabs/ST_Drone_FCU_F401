# ST Drone

This repository contains the ST Drone app source code.

Android GUI to pilot the ST drone by Bluetooth low energy connection. 
The ST drone is a STEVAL-FCU001V1 board based on stm32 processor and ST sensors 
such as 9 axis gyro, accelerometer, magnetometer, pressure, temperatures and Bluetooth low energy.
The GUI allows to pilot drone after the Bluetooth low energy connection. 
The drone is driven by touchable analog cursors. The GUI is composed by the following screens:
- Pilot GUI
- Bluetooth low energy connector GUI
- Two settings GUI to set drone parameters and to change configurations
- Help GUI in English or Italian language to explains all buttons and features 

For more information about world ST drone: https://community.st.com/community/drone-zone  
For more information about STEVAL-FCU001V1: http://www.st.com/en/evaluation-tools/steval-fcu001v1.html
To download the firmware source code free:  https://github.com/STMicroelectronics-CentralLabs/ST_Drone_FCU_F401
Tags (sorted by importance): Drone, STEVAL-FCU001V1, stm32, sensor, IoT, STMicroelectronics

The application is built on top of BlueST SDK, a library that provides an implementation of BlueST protocol 
and helps to easily export the data via Bluetooth® Low Energy. 
The SDK source code is freely available on [github](https://github.com/STMicroelectronics-CentralLabs/)

## Download the source

Since the project uses git submodules, <code>--recursive</code> option must be used to clone the repository:

```Shell
git clone --recursive https://github.com/STMicroelectronics-CentralLabs/ST_Drone_FCU_F401
```

or run
```Shell
git clone https://github.com/STMicroelectronics-CentralLabs/ST_Drone_FCU_F401
git submodule update --init --recursive
```

## License

Copyright (c) 2017  STMicroelectronics – All rights reserved
The STMicroelectronics corporate logo is a trademark of STMicroelectronics

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or other materials provided
with the distribution.

- Neither the name nor trademarks of STMicroelectronics International N.V. nor any other
STMicroelectronics company nor the names of its contributors may be used to endorse or
promote products derived from this software without specific prior written permission.

- All of the icons, pictures, logos and other images that are provided with the source code
in a directory whose title begins with st_images may only be used for internal purposes and
shall not be redistributed to any third party or modified in any way.

- Any redistributions in binary form shall not include the capability to display any of the
icons, pictures, logos and other images that are provided with the source code in a directory
whose title begins with st_images.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.

